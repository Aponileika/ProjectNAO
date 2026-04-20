# -*- coding: utf-8 -*-
import json
import urllib2

class GeminiClient(object):
    def __init__(self):
        self.api_key = ""
        self.model = "gemini-flash-latest"
        self.base_url = "https://generativelanguage.googleapis.com/v1beta/models/"

    def set_api_key(self, api_key):
        import re
        # Remove any hidden null bytes, unicode zero-width spaces, or weird formatting from copy-pasting
        if isinstance(api_key, unicode):
            api_key = api_key.encode('ascii', 'ignore')
        api_key = re.sub(r'[^A-Za-z0-9_\-]', '', api_key)
        self.api_key = api_key

    def generate_text(self, prompt, audio_bytes=None):
        if not self.api_key:
            return "Error: API key is not set. Please provide your Gemini API key."

        import datetime
        now = datetime.datetime.now().strftime("%A, %B %d, %Y, at %I:%M %p")
        sys_instructions = (
            "You are NAO, an intelligent humanoid companion robot. "
            "The current world date and time is {}. "
            "You can engage in natural, conversational responses. "
            "Never use markdown, lists, asterisks, emojis, or symbols because you are speaking out loud through a Text-To-Speech engine."
        ).format(now)

        url = "{}{}:generateContent?key={}".format(self.base_url, self.model, self.api_key)
        headers = {'Content-Type': 'application/json'}
        
        payload = {
            "systemInstruction": {
                "parts": [{"text": sys_instructions}]
            },
            "contents": [{
                "parts": [{"text": prompt}]
            }],
            "generationConfig": {
                "maxOutputTokens": 1024,
                "temperature": 0.5
            }
        }
        
        if audio_bytes:
            import base64
            b64_audio = base64.b64encode(audio_bytes).decode('ascii')
            payload["contents"][0]["parts"].append({
                "inlineData": {"mimeType": "audio/wav", "data": b64_audio}
            })
            
        req = urllib2.Request(url, data=json.dumps(payload), headers=headers)
        try:
            response = urllib2.urlopen(req)
            result = json.loads(response.read())
            if 'candidates' in result and len(result['candidates']) > 0:
                content = result['candidates'][0]['content']['parts'][0]['text']
                # Strip markdown for speech
                return content.replace('*', '').replace('#', '').strip()
            return "I received an empty response from Gemini."
        except urllib2.HTTPError as e:
            error_body = e.read()
            return "HTTP Error %s. %s" % (e.code, error_body[:80])
        except Exception as e:
            return "An error occurred connecting to Gemini: " + str(e)
