# -*- coding: utf-8 -*-
"""Contextual speech for the wander loop, generated before it is needed.

The phrase banks in backend/persona.py are fixed sentences picked at random.
They never repeat back to back, but they also never refer to anything that
actually happened - the robot says "still looking" whether it just started or
has been circling the same chair for two minutes.

This module puts a local language model behind the same call.  The model is
told what the robot has been doing and writes the next line itself, so the
speech follows the situation instead of a script.

The problem it has to solve is latency.  Generating a sentence takes two to
four seconds on this hardware, and a control loop cannot stop and wait for
that.  So nothing is generated on demand:

    * a background worker generates lines and parks them in a small buffer
      per category, while the robot is busy walking;
    * line() takes whatever is already sitting in the buffer and returns
      immediately, then asks for a refill;
    * if the buffer is empty - model still warming up, Ollama not running,
      generation failed - it returns "" and the caller falls back to the
      phrase bank.

The result is that speech is instant at the moment it is wanted, and the
robot degrades to its old scripted self rather than going quiet.

Python 2.7 only, to match the rest of the app.
"""

import Queue
import threading
import time
from collections import deque

from ..backend import persona


# How many ready-to-speak lines to keep per category.  Two is enough to cover
# a burst of the same event without the buffer running dry mid-wander.
BUFFER_DEPTH = 2

# Events kept as context for the next line.  Long histories make a small model
# ramble and cost prompt-eval time on every request.
MEMORY_DEPTH = 6

# Generation is capped short: these are spoken asides, not paragraphs.
MAX_TOKENS  = 40
TEMPERATURE = 0.9

# Queued the moment a wander starts, in the order the robot tends to need
# them: idle chatter and obstacle reactions carry most of a wander's speech.
OPENING_CATEGORIES = ("walking", "avoid_sonar", "avoid_bumper", "searching")

# Past-tense note recorded whenever a category is spoken, so the NEXT line has
# some idea what the robot has been through.  Asking for a line is itself the
# signal that the event happened, which saves threading note() calls through
# every branch of the wander loop.
EVENT_NOTES = {
    "avoid_sonar":      "you steered around an obstacle",
    "avoid_bumper":     "you bumped into something with your foot",
    "avoid_boundary":   "you reached the edge of your area and turned back",
    "stuck":            "you got wedged and had to work your way out",
    "remembered_block": "you returned to a spot you knew was blocked",
    "found_human":      "you spotted a person",
    "handshake":        "you offered your hand to shake",
    "found_target":     "you found what you were looking for",
    "fell":             "you fell over",
    "task_done":        "you finished and stopped",
    "walking":          "you kept walking",
    "searching":        "you kept searching",
    "searching_human":  "you kept looking for someone",
}

# What each phrase-bank category means, in words the model can act on.  Only
# categories listed here are generated; anything else falls through to the
# phrase bank untouched.
SITUATIONS = {
    "walk_start":      "You have just started walking around the room with no particular goal.",
    "search_start":    "You have just started searching the room for %(target)s.",
    "walking":         "You have been walking around the room for a little while.",
    "searching":       "You are still searching the room for %(target)s and have not found it yet.",
    "searching_human": "You are still looking around for a person and have not found anyone yet.",
    "avoid_sonar":     "Your sonar just picked up an obstacle ahead and you are steering around it.",
    "avoid_bumper":    "Your foot bumper just hit something on the floor and you are backing off.",
    "avoid_boundary":  "You just detected the edge of your safe area and are turning back.",
    "stuck":           "You are wedged against something and are working your way out.",
    "remembered_block":"You have arrived somewhere you already know is blocked, and you remember it.",
    "found_human":     "You have just spotted a person.",
    "handshake":       ("You are holding your hand out to a person you just met, "
                        "asking them to shake it. Ask them directly."),
    "found_target":    "You have just found %(target)s, which is what you were looking for.",
    "fell":            "You have fallen over and your motors have been released.",
    "task_done":       "You have finished walking and are coming to a stop.",
    "greeting":        "You have just been switched on and are greeting whoever is nearby.",
}

# Held constant so Ollama can reuse the cached prompt prefix; only the short
# situation line at the end changes between requests.
_STYLE = (
    "You are speaking one line out loud, right now, in the middle of doing "
    "something physical.\n"
    "Rules: one sentence, at most 14 words. No markdown, no emoji, no "
    "asterisks, no stage directions, no quotation marks. Do not describe "
    "yourself in the third person. Do not greet unless told to. Say something "
    "specific to the situation rather than a generic filler line."
)


class SpeechDirector(object):
    """Generates spoken lines ahead of time, with a phrase-bank fallback."""

    def __init__(self, client, enabled=False, on_error=None):
        self.client   = client
        self.enabled  = enabled
        self.on_error = on_error

        self._buffers = {}                  # category -> [line, ...]
        self._memory  = deque(maxlen=MEMORY_DEPTH)
        self._fmt     = {}                  # last-known format args (target...)
        self._lock    = threading.Lock()
        self._q       = Queue.Queue()
        self._stop    = threading.Event()
        self._worker  = None
        self._inflight = set()              # categories already queued

        self.generated = 0                  # counters, for the UI
        self.fallbacks = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Spin up the background generator.  Safe to call more than once."""
        if self._worker is not None and self._worker.is_alive():
            return
        self._stop.clear()
        self._worker = threading.Thread(target=self._run)
        self._worker.daemon = True
        self._worker.start()

    def stop(self):
        """Stop generating and drop anything queued."""
        self._stop.set()
        try:
            while True:
                self._q.get_nowait()
        except Queue.Empty:
            pass
        with self._lock:
            self._inflight.clear()

    def reset(self, target=""):
        """Clear buffers and memory at the start of a new wander.

        Also queues the categories a wander leans on most.  Generation is
        serial at roughly three seconds a line, so without a head start the
        opening stretch of every wander falls back to the phrase bank."""
        with self._lock:
            self._buffers.clear()
            self._inflight.clear()
            self._memory.clear()
            self._fmt = {"target": target or "anything in particular"}
        self.prefetch(*OPENING_CATEGORIES)

    # ------------------------------------------------------------------
    # Context
    # ------------------------------------------------------------------

    def note(self, event):
        """Record something that just happened, for the next line's context.

        Repeats are counted rather than appended: a wander says "you kept
        walking" far more often than anything else, and six copies of it would
        push the interesting events out of a six-slot memory."""
        if not event:
            return
        event = str(event)
        with self._lock:
            if self._memory:
                last = self._memory[-1]
                base = last.split(" (x")[0]
                if base == event:
                    count = 2
                    if " (x" in last:
                        try:
                            count = int(last.split(" (x")[1].rstrip(")")) + 1
                        except Exception:
                            count = 2
                    self._memory[-1] = "%s (x%d)" % (event, count)
                    return
            self._memory.append(event)

    def _context_block(self):
        with self._lock:
            recent = list(self._memory)
        if not recent:
            return "Nothing has happened yet."
        return "Recently: " + "; ".join(recent) + "."

    # ------------------------------------------------------------------
    # The call the control loop makes
    # ------------------------------------------------------------------

    def line(self, category, **fmt):
        """A generated line if one is ready, else "" - never blocks.

        Returning "" is the normal, expected path whenever the model is cold,
        missing or slow; the caller speaks a phrase-bank line instead."""
        if not self.enabled:
            return ""
        if category not in SITUATIONS:
            return ""

        if fmt:
            with self._lock:
                self._fmt.update(fmt)

        with self._lock:
            buf = self._buffers.get(category) or []
            text = buf.pop(0) if buf else ""
            self._buffers[category] = buf

        # Being asked for this category IS the event happening, so record it.
        # The line just handed back was written before this note existed - it
        # describes the moment, and the note colours whatever comes next.
        self.note(EVENT_NOTES.get(category))

        # Refill whether or not we got one, so the next call is more likely
        # to hit.  prefetch() is cheap and de-duplicates internally.
        self.prefetch(category)

        if text:
            self.generated += 1
        else:
            self.fallbacks += 1
        return text

    def prefetch(self, *categories):
        """Ask the worker to top up the buffers for these categories."""
        if not self.enabled or self._stop.is_set():
            return
        for category in categories:
            if category not in SITUATIONS:
                continue
            with self._lock:
                have = len(self._buffers.get(category) or [])
                if have >= BUFFER_DEPTH or category in self._inflight:
                    continue
                self._inflight.add(category)
            try:
                self._q.put_nowait(category)
            except Exception:
                with self._lock:
                    self._inflight.discard(category)

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------

    def _prompt_for(self, category):
        with self._lock:
            fmt = dict(self._fmt)
        situation = SITUATIONS[category]
        try:
            situation = situation % fmt
        except Exception:
            pass                            # placeholder with no value supplied
        return "%s\n\n%s\n\nSituation: %s\n\nSay one line:" % (
            _STYLE, self._context_block(), situation)

    def _run(self):
        system = persona.system_prompt()
        # Pay the cold-load cost here, on the worker, before anything is
        # waiting on it.  Twelve seconds of silence at the start of a wander
        # is exactly the problem this module exists to avoid.
        if not self._stop.is_set():
            try:
                self.client.warm(system=system)
            except Exception:
                pass
        while not self._stop.is_set():
            try:
                category = self._q.get(timeout=0.5)
            except Queue.Empty:
                continue
            if self._stop.is_set():
                break
            try:
                text = self.client.generate_text(
                    self._prompt_for(category), system=system,
                    max_tokens=MAX_TOKENS, temperature=TEMPERATURE)
                text = _first_sentence(text)
                if text:
                    with self._lock:
                        buf = self._buffers.setdefault(category, [])
                        if len(buf) < BUFFER_DEPTH:
                            buf.append(text)
                elif self.on_error and self.client.last_error:
                    self.on_error(self.client.last_error)
            except Exception, e:
                if self.on_error:
                    try:
                        self.on_error(str(e))
                    except Exception:
                        pass
            finally:
                with self._lock:
                    self._inflight.discard(category)
            # A brief pause keeps a cold model from being hammered by a burst
            # of refills while it is still loading.
            time.sleep(0.05)


def _first_sentence(text):
    """Keep the model to one spoken line even when it ignores the brief."""
    if not text:
        return ""
    out = text.strip()
    for i, ch in enumerate(out):
        if ch in ".!?" and i >= 8:
            return out[:i + 1].strip()
    return out
