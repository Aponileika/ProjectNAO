# -*- coding: utf-8 -*-
"""Local language model client, talking to Ollama over plain HTTP.

Why this exists: Gemini is a network round trip away, rate limited, and slow
enough that the robot's replies land after the moment has passed.  A model
running on this machine answers in a couple of seconds and never hits a quota.

The trade is quality and speed of generation.  On CPU-only hardware a 3B model
produces roughly five tokens a second, which is about the pace NAO speaks at.
That is only smooth if speech starts at the FIRST SENTENCE instead of waiting
for the whole reply, so this client streams and hands back each sentence the
moment it is complete - see generate_stream().

Python 2.7 only, to match the rest of the app.  No third-party packages: the
app's bundled interpreter has no pip, so this is stdlib sockets and json.
"""

import json
import re
import socket
import urllib2

DEFAULT_HOST  = "127.0.0.1:11434"
DEFAULT_MODEL = "qwen2.5:3b-instruct"

# How long Ollama keeps the model resident.  Loading it costs ~12s on this
# hardware, so anything shorter than a wander re-pays that cost on the next
# line the robot speaks.
KEEP_ALIVE = "30m"

_SENTENCE_END = re.compile(r'[.!?]["\'\)\]]?(\s|$)')

# Markdown, stage directions and decoration are all meaningless out loud and
# Acapela reads several of them literally.
_STRIP_CHARS   = re.compile(r'[*#`_~<>\[\]{}|]')
_PARENTHETICAL = re.compile(r'\([^)]*\)')
_UNICODE_PUNCT = [
    (u"—", u", "), (u"–", u", "), (u"…", u"..."),
    (u"‘", u"'"), (u"’", u"'"),
    (u"“", u""),  (u"”", u""),
]


def clean_for_speech(text):
    """Reduce model output to something a TTS engine can read aloud."""
    if not text:
        return ""
    if not isinstance(text, unicode):
        try:
            text = text.decode("utf-8", "ignore")
        except Exception:
            return ""
    for bad, good in _UNICODE_PUNCT:
        text = text.replace(bad, good)
    text = _PARENTHETICAL.sub(" ", text)
    text = _STRIP_CHARS.sub("", text)
    text = text.encode("ascii", "ignore")        # Acapela mangles the rest
    text = re.sub(r'\s+', ' ', text)
    # Dash-to-comma substitution above can leave " , "; tidy the spacing so
    # the TTS engine does not pause in the wrong place.
    text = re.sub(r'\s+([,.!?;:])', r'\1', text)
    return text.strip()


class OllamaClient(object):
    """Minimal Ollama client: availability, models, blocking and streaming."""

    def __init__(self, host=DEFAULT_HOST, model=DEFAULT_MODEL):
        self.host       = host
        self.model      = model
        self.last_error = ""

    # ------------------------------------------------------------------
    # Server discovery
    # ------------------------------------------------------------------

    def _url(self, path):
        return "http://%s%s" % (self.host, path)

    def _host_port(self):
        if ":" in self.host:
            h, p = self.host.rsplit(":", 1)
            try:
                return h, int(p)
            except ValueError:
                return h, 11434
        return self.host, 11434

    def list_models(self, timeout=3):
        """Names of the models Ollama has pulled.  [] if the server is down."""
        try:
            r = urllib2.urlopen(self._url("/api/tags"), timeout=timeout)
            data = json.loads(r.read())
            return [m.get("name", "") for m in data.get("models", []) if m.get("name")]
        except Exception, e:
            self.last_error = str(e)
            return []

    def available(self, timeout=3):
        """True when the server answers AND the configured model is pulled."""
        names = self.list_models(timeout=timeout)
        if not names:
            return False
        if self.model in names:
            return True
        # Ollama tolerates a missing ":latest"; match on the bare name too.
        base = self.model.split(":")[0]
        return any(n == self.model or n.split(":")[0] == base for n in names)

    # ------------------------------------------------------------------
    # Generation
    # ------------------------------------------------------------------

    def _payload(self, prompt, system, stream, max_tokens, temperature):
        body = {
            "model":      self.model,
            "prompt":     prompt,
            "stream":     bool(stream),
            "keep_alive": KEEP_ALIVE,
            "options": {
                "num_predict": max_tokens,
                "temperature": temperature,
                # Short spoken lines: discourage the model from running on.
                "stop": ["\n\n"],
            },
        }
        if system:
            body["system"] = system
        return json.dumps(body)

    def warm(self, system=None, timeout=90):
        """Force the model resident so the next real line is not stuck behind
        a cold load.

        Loading costs about twelve seconds on CPU-only hardware.  Paid at the
        moment the robot is asked to speak, that is the difference between a
        reply and an awkward silence, so it is paid up front instead - when
        the backend is switched on, or when a wander starts.

        Generates a single token; the output is discarded."""
        try:
            payload = self._payload("Hi", system, False, 1, 0.0)
            req = urllib2.Request(self._url("/api/generate"), data=payload,
                                  headers={"Content-Type": "application/json"})
            urllib2.urlopen(req, timeout=timeout).read()
            return True
        except Exception, e:
            self.last_error = str(e)
            return False

    def generate_text(self, prompt, system=None, max_tokens=64,
                      temperature=0.8, timeout=60):
        """Blocking generation.  Returns cleaned text, or "" on failure.

        Mirrors GeminiClient.generate_text closely enough to stand in for it,
        but returns "" rather than an error string so callers can simply fall
        back to a phrase bank when the local model is unavailable."""
        payload = self._payload(prompt, system, False, max_tokens, temperature)
        req = urllib2.Request(self._url("/api/generate"), data=payload,
                              headers={"Content-Type": "application/json"})
        try:
            r = urllib2.urlopen(req, timeout=timeout)
            data = json.loads(r.read())
            return clean_for_speech(data.get("response", ""))
        except Exception, e:
            self.last_error = str(e)
            return ""

    def generate_stream(self, prompt, system=None, on_sentence=None,
                        max_tokens=64, temperature=0.8, timeout=60,
                        should_stop=None):
        """Stream a reply, invoking on_sentence(text) at each sentence end.

        This is the call that makes the robot sound fluent: the first sentence
        is ready in well under half the time the full reply takes, so speech
        can start while the rest is still being generated.

        on_sentence  - callback per completed sentence, already TTS-cleaned.
        should_stop  - optional predicate; return True to abandon generation
                       mid-flight (used when the wander loop is shutting down).

        Returns the full cleaned reply, or "" on failure.

        urllib2 is deliberately not used here: its file iteration reads ahead,
        which collapses streaming into one blocking read and defeats the point.
        """
        payload = self._payload(prompt, system, True, max_tokens, temperature)
        host, port = self._host_port()
        sock = None
        collected = []
        pending = ""
        try:
            sock = socket.create_connection((host, port), timeout)
            sock.settimeout(timeout)
            sock.sendall(
                "POST /api/generate HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n\r\n%s" % (self.host, len(payload), payload))

            for line in _iter_body_lines(sock):
                if should_stop is not None and should_stop():
                    break
                try:
                    obj = json.loads(line)
                except Exception:
                    continue
                if obj.get("error"):
                    self.last_error = str(obj["error"])
                    break
                tok = obj.get("response", "")
                if tok:
                    collected.append(tok)
                    pending += tok
                    # Emit as soon as a sentence closes so speech can begin.
                    if on_sentence:
                        while True:
                            m = _SENTENCE_END.search(pending)
                            if not m:
                                break
                            sentence = clean_for_speech(pending[:m.end()])
                            pending = pending[m.end():]
                            if sentence:
                                on_sentence(sentence)
                if obj.get("done"):
                    break

            # Whatever is left had no terminating punctuation.
            if on_sentence:
                tail = clean_for_speech(pending)
                if tail:
                    on_sentence(tail)
            return clean_for_speech("".join(collected))
        except Exception, e:
            self.last_error = str(e)
            return clean_for_speech("".join(collected))
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass


def _iter_body_lines(sock):
    """Yield newline-delimited body lines from a chunked HTTP response.

    Ollama streams with Transfer-Encoding: chunked, one JSON object per line.
    Decoding it by hand keeps the socket unbuffered; httplib would read ahead.
    """
    buf = ""
    body = ""
    headers_done = False
    chunked = False
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            return
        except Exception:
            return

        if data:
            buf += data
        if not headers_done:
            if "\r\n\r\n" not in buf:
                if not data:
                    return
                continue
            head, buf = buf.split("\r\n\r\n", 1)
            chunked = "chunked" in head.lower()
            headers_done = True

        if chunked:
            # Peel complete chunks off the front of the buffer.
            while True:
                if "\r\n" not in buf:
                    break
                size_line, rest = buf.split("\r\n", 1)
                size_line = size_line.strip()
                if not size_line:
                    buf = rest
                    continue
                try:
                    size = int(size_line, 16)
                except ValueError:
                    break
                if size == 0:
                    buf = ""
                    for ln in body.split("\n"):
                        if ln.strip():
                            yield ln.strip()
                    return
                if len(rest) < size + 2:
                    break                      # chunk still arriving
                body += rest[:size]
                buf = rest[size + 2:]          # skip the trailing CRLF
        else:
            body += buf
            buf = ""

        while "\n" in body:
            line, body = body.split("\n", 1)
            if line.strip():
                yield line.strip()

        if not data:
            return
