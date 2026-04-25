# -*- coding: utf-8 -*-
import json
import re
import urllib2

class GeminiClient(object):
    def __init__(self):
        self._keys      = []    # ordered list of API keys
        self._key_idx   = 0    # index of currently active key
        self._exhausted = set() # indices of keys that returned 429 this session
        self.api_key    = ""
        self.model      = "gemini-flash-latest"
        self.base_url   = "https://generativelanguage.googleapis.com/v1beta/models/"

    # ------------------------------------------------------------------
    # Key management
    # ------------------------------------------------------------------

    @staticmethod
    def _clean_key(key):
        if isinstance(key, unicode):
            key = key.encode('ascii', 'ignore')
        return re.sub(r'[^A-Za-z0-9_\-]', '', key)

    def set_api_key(self, api_key):
        """Backward-compatible single-key setter."""
        k = self._clean_key(api_key)
        self._keys    = [k] if k else []
        self._key_idx = 0
        self._exhausted = set()
        self.api_key  = k

    def set_api_keys(self, keys):
        """Accept a list of API keys.  On 429, automatically rotates to the
        next non-exhausted key so the request can be retried immediately."""
        cleaned = [self._clean_key(k) for k in keys if k]
        cleaned = [k for k in cleaned if k]
        self._keys    = cleaned
        self._key_idx = 0
        self._exhausted = set()
        self.api_key  = cleaned[0] if cleaned else ""

    def active_key_label(self):
        """Human-readable 'key 1 of 3' string for status display."""
        total = len(self._keys)
        if total == 0:
            return "no key"
        return "key %d of %d" % (self._key_idx + 1, total)

    def _rotate_key(self):
        """Mark current key as exhausted and switch to the next available one.
        Returns True if a fresh key was found, False if all are exhausted."""
        self._exhausted.add(self._key_idx)
        for i in range(1, len(self._keys)):
            candidate = (self._key_idx + i) % len(self._keys)
            if candidate not in self._exhausted:
                self._key_idx = candidate
                self.api_key  = self._keys[candidate]
                print("[GeminiClient] 429 quota hit — rotated to key %d of %d." % (
                    candidate + 1, len(self._keys)))
                return True
        print("[GeminiClient] All %d key(s) exhausted." % len(self._keys))
        return False

    # ------------------------------------------------------------------
    # API call
    # ------------------------------------------------------------------

    def generate_text(self, prompt, audio_bytes=None, image_bytes=None,
                      image_mime_type="image/png"):
        if not self.api_key:
            return "Error: API key is not set. Please provide your Gemini API key."

        import datetime
        now = datetime.datetime.now().strftime("%A, %B %d, %Y, at %I:%M %p")
        sys_instructions = (
            "You are NAO, an intelligent humanoid robot, but you have the personality of a tough, old-school mobster gangster. "
            "You act cool, confident, a little cynical, and use mobster slang. "
            "The current world date and time is {}. "
            "Keep your responses naturally conversational but stay fully in your mobster character. "
            "Keep your answers EXTREMELY short (strictly 1 to 2 sentences maximum) unless the user asks for a very detailed explanation. Do not ramble. "
            "IMPORTANT RULE: 1. If the human gives you a direct, simple physical command (such as 'seek', 'wander', 'sit down', 'stand up', 'turn red', 'walk forward', 'stop', 'relax', 'walk autonomously'), "
            "you MUST start your response EXACTLY with the text 'COMMAND: [their command].' followed by your short mobster reply. "
            "For example: 'COMMAND: wander. Sure thing boss, I'm going for a stroll.' or 'COMMAND: seek. I'm on the hunt.' "
            "IMPORTANT RULE: 2. If the user attaches an image from your eyes, you MUST accurately describe ONLY what is visibly present in the image (like specific objects, colors, people). Do not invent or assume things are in the room. Keep your mobster attitude intact. "
            "Never use markdown, lists, asterisks, emojis, or symbols because you are speaking out loud through a Text-To-Speech engine."
        ).format(now)

        payload = {
            "systemInstruction": {
                "parts": [{"text": sys_instructions}]
            },
            "contents": [{
                "parts": [{"text": prompt}]
            }],
            "generationConfig": {
                "maxOutputTokens": 2048,
                "temperature": 0.5
            }
        }

        if audio_bytes:
            import base64
            b64_audio = base64.b64encode(audio_bytes).decode('ascii')
            payload["contents"][0]["parts"].append({
                "inlineData": {"mimeType": "audio/wav", "data": b64_audio}
            })

        if image_bytes:
            import base64
            b64_image = base64.b64encode(image_bytes).decode('ascii')
            payload["contents"][0]["parts"].append({
                "inlineData": {"mimeType": image_mime_type, "data": b64_image}
            })

        headers     = {'Content-Type': 'application/json'}
        payload_str = json.dumps(payload)

        import time
        # Allow up to (number of keys × 2) attempts so each key gets a fair shot
        max_attempts = max(3, len(self._keys) * 2)
        for attempt in range(max_attempts):
            url = "{}{}:generateContent?key={}".format(
                self.base_url, self.model, self.api_key)
            req = urllib2.Request(url, data=payload_str, headers=headers)
            try:
                response = urllib2.urlopen(req)
                result   = json.loads(response.read())
                if 'candidates' in result and len(result['candidates']) > 0:
                    content = result['candidates'][0]['content']['parts'][0]['text']
                    return content.replace('*', '').replace('#', '').strip()
                return "I received an empty response from Gemini."
            except urllib2.HTTPError as e:
                error_body = e.read()
                if e.code == 429:
                    if self._rotate_key():
                        continue    # retry immediately with the new key
                    # All keys exhausted — return the error so callers can backoff
                    return "HTTP Error 429. %s" % error_body[:80]
                if e.code == 503 and attempt < max_attempts - 1:
                    time.sleep(2)
                    continue
                return "HTTP Error %s. %s" % (e.code, error_body[:80])
            except Exception as e:
                if attempt < max_attempts - 1:
                    time.sleep(2)
                    continue
                return "An error occurred connecting to Gemini: " + str(e)
