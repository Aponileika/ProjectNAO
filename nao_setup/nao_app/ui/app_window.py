# -*- coding: utf-8 -*-
import Tkinter as tk
import tkFont
import threading
from nao_app.ui.widgets import make_card, make_btn, BG, FG, ACCENT, SUCCESS, ERROR, WARN, CARD_BG, BTN_BG, BTN_FG

class NaoAppWindow(object):
    LANGUAGES = [
        "English", "French", "German", "Spanish",
        "Italian", "Japanese", "Chinese", "Korean",
        "Portuguese", "Dutch", "Polish", "Czech",
        "Turkish", "Swedish", "Danish", "Norwegian",
        "Finnish", "Arabic", "Russian",
    ]

    LED_GROUPS = [
        "AllLeds", "FaceLeds", "EarLeds",
        "ChestLeds", "FeetLeds", "BrainLeds",
    ]

    LED_COLOURS = {
        "White":   0x00FFFFFF,
        "Red":     0x00FF0000,
        "Green":   0x0000FF00,
        "Blue":    0x000000FF,
        "Yellow":  0x00FFFF00,
        "Cyan":    0x0000FFFF,
        "Magenta": 0x00FF00FF,
        "Orange":  0x00FF8C00,
        "Off":     0x00000000,
    }

    POSTURES = [
        "Stand", "StandInit", "StandZero",
        "Sit", "SitRelax", "Crouch",
        "LyingBelly", "LyingBack", "Relax",
    ]

    def __init__(self, conn, vision, controller):
        self.conn = conn
        self.vision = vision
        self.controller = controller
        
        self.api_key_autofill = ""
        try:
            import json, os
            # App window is in NAO/nao_setup/nao_app/ui/app_window.py (4 levels deep)
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            conf_path = os.path.join(base_dir, "config.json")
            secrets_path = os.path.join(base_dir, "secrets.json")
            
            if os.path.isfile(conf_path):
                with open(conf_path, "r") as f:
                    cdata = json.load(f)
                    self.api_key_autofill = cdata.get("gemini_key", "")
                    
            if os.path.isfile(secrets_path):
                with open(secrets_path, "r") as f:
                    sdata = json.load(f)
                    if sdata.get("gemini_key"):
                        self.api_key_autofill = sdata.get("gemini_key")
        except:
            pass

        self._controller_expanded = False
        self._build_ui()
        
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("NAO Control Panel")
        self.root.configure(bg=BG)
        self.root.state("zoomed")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.font_title = tkFont.Font(family="Segoe UI", size=18, weight="bold")
        self.font_head  = tkFont.Font(family="Segoe UI", size=11, weight="bold")
        self.font_norm  = tkFont.Font(family="Segoe UI", size=10)
        self.font_small = tkFont.Font(family="Segoe UI", size=9)
        self.font_guide = tkFont.Font(family="Consolas",  size=10)

        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=16, pady=(10, 0))
        tk.Label(top, text="NAO Control Panel", font=self.font_title, bg=BG, fg=ACCENT).pack(side="left")
        self.status_var = tk.StringVar(value="Not connected")
        self.status_lbl = tk.Label(top, textvariable=self.status_var, font=self.font_norm, bg=BG, fg=ERROR)
        self.status_lbl.pack(side="right")

        canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        vsb = tk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        self._body = tk.Frame(canvas, bg=BG)
        self._body.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self._body, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(fill="both", expand=True, padx=12, pady=8)
        self._canvas = canvas

        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))
        def _on_canvas_resize(event):
            canvas.itemconfigure("bodywin", width=event.width)
        canvas.bind("<Configure>", _on_canvas_resize)
        canvas.create_window((0, 0), window=self._body, anchor="nw", tags="bodywin")

        self._body.columnconfigure(0, weight=1)
        self._body.columnconfigure(1, weight=1)

        left = tk.Frame(self._body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        self._card_connection(left)
        self._card_volume(left)
        self._card_language(left)
        self._card_speech(left)
        self._card_leds(left)
        self._card_posture(left)
        self._card_life(left)
        self._card_battery(left)

        right = tk.Frame(self._body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self._card_gemini(right)
        self._card_controller(right)
        self._card_camera(right)

    def _card_connection(self, parent):
        card = make_card(parent, "Connection", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="IP:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.ip_entry = tk.Entry(row, font=self.font_norm, width=16)
        self.ip_entry.insert(0, "192.168.0.123")
        self.ip_entry.pack(side="left", padx=4)
        tk.Label(row, text="Port:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.port_entry = tk.Entry(row, font=self.font_norm, width=6)
        self.port_entry.insert(0, "9559")
        self.port_entry.pack(side="left", padx=4)
        make_btn(row, "Connect", self._on_connect, width=10, font_norm=self.font_norm).pack(side="left", padx=6)

    def _card_volume(self, parent):
        card = make_card(parent, "Volume", self.font_head)
        self.vol_var = tk.IntVar(value=50)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="0", font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left")
        tk.Scale(row, from_=0, to=100, orient="horizontal", variable=self.vol_var, length=220, bg=CARD_BG, fg=FG, troughcolor=BTN_BG, highlightthickness=0, font=self.font_small).pack(side="left", padx=4)
        tk.Label(row, text="100", font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left")
        make_btn(row, "Set", self._on_volume_set, width=6, font_norm=self.font_norm).pack(side="left", padx=6)

    def _card_language(self, parent):
        card = make_card(parent, "Language", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self.lang_var = tk.StringVar(value="English")
        tk.Label(row, text="Language:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        m = tk.OptionMenu(row, self.lang_var, *self.LANGUAGES)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG, activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)
        make_btn(row, "Set", self._on_language_set, width=6, font_norm=self.font_norm).pack(side="left", padx=6)
        make_btn(row, "Get", self._on_language_get, width=6, font_norm=self.font_norm).pack(side="left")

    def _card_speech(self, parent):
        card = make_card(parent, "Speech Test", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="Say:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.speech_entry = tk.Entry(row, font=self.font_norm, width=22)
        self.speech_entry.insert(0, "Hello, I am NAO")
        self.speech_entry.pack(side="left", padx=4)
        make_btn(row, "Speak", self._on_speak, width=8, font_norm=self.font_norm).pack(side="left", padx=6)

    def _card_leds(self, parent):
        card = make_card(parent, "LEDs", self.font_head)
        row1 = tk.Frame(card, bg=CARD_BG)
        row1.pack(fill="x", pady=(0, 4))
        tk.Label(row1, text="Group:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.led_group_var = tk.StringVar(value="AllLeds")
        m = tk.OptionMenu(row1, self.led_group_var, *self.LED_GROUPS)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG, activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)

        row2 = tk.Frame(card, bg=CARD_BG)
        row2.pack(fill="x")
        for name in sorted(self.LED_COLOURS.keys()):
            hexval = self.LED_COLOURS[name]
            btn_col = "#%06x" % (hexval & 0xFFFFFF) if hexval != 0 else "#333333"
            fg_col = "#000000" if name in ("White", "Yellow", "Cyan", "Green") else "#FFFFFF"
            tk.Button(row2, text=name, width=7, font=self.font_small, bg=btn_col, fg=fg_col, relief="flat", command=lambda c=hexval: self._on_led_colour(c)).pack(side="left", padx=1, pady=2)

    def _card_posture(self, parent):
        card = make_card(parent, "Posture", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self.posture_var = tk.StringVar(value="StandInit")
        tk.Label(row, text="Posture:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        m = tk.OptionMenu(row, self.posture_var, *self.POSTURES)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG, activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)
        make_btn(row, "Go", self._on_posture, width=6, font_norm=self.font_norm).pack(side="left", padx=6)

    def _card_life(self, parent):
        card = make_card(parent, "Autonomous Life", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        make_btn(row, "Enable", self._on_life_on, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        make_btn(row, "Disable", self._on_life_off, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        self.life_status = tk.StringVar(value="")
        tk.Label(row, textvariable=self.life_status, font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left", padx=8)

    def _card_battery(self, parent):
        card = make_card(parent, "Battery", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        make_btn(row, "Refresh", self._on_battery, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        self.battery_var = tk.StringVar(value="--")
        tk.Label(row, textvariable=self.battery_var, font=self.font_norm, bg=CARD_BG, fg=SUCCESS).pack(side="left", padx=8)

    def _card_gemini(self, parent):
        card = make_card(parent, "AI Chat (Gemini)", self.font_head)
        
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=(0, 6))
        tk.Label(r1, text="API Key:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.gemini_key_entry = tk.Entry(r1, font=self.font_norm, show="*", width=20)
        if self.api_key_autofill:
            self.gemini_key_entry.insert(0, self.api_key_autofill)
        self.gemini_key_entry.pack(side="left", padx=4)
        
        r2 = tk.Frame(card, bg=CARD_BG)
        r2.pack(fill="x", pady=(0, 6))
        tk.Label(r2, text="Prompt: ", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.gemini_prompt_entry = tk.Entry(r2, font=self.font_norm, width=28)
        self.gemini_prompt_entry.pack(side="left", padx=4)
        
        r3 = tk.Frame(card, bg=CARD_BG)
        r3.pack(fill="x", pady=(0, 4))
        make_btn(r3, "Ask (Text)", self._on_gemini_ask, width=10, font_norm=self.font_norm).pack(side="left", padx=2)
        make_btn(r3, "Talk to NAO (5s)", self._on_gemini_voice_interactive, width=16, font_norm=self.font_norm).pack(side="left", padx=6)
        
        r4 = tk.Frame(card, bg=CARD_BG)
        r4.pack(fill="x")
        self.gemini_status = tk.StringVar(value="Ready.")
        tk.Label(r4, textvariable=self.gemini_status, font=self.font_small, bg=CARD_BG, fg=ACCENT).pack(side="left")

    def _card_controller(self, parent):
        card = make_card(parent, "PS5 Controller", self.font_head)
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=(0, 6))
        make_btn(r1, "Detect", self.controller.detect_controller, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        self.ctrl_indicator = tk.Label(r1, text="*", font=self.font_norm, bg=CARD_BG, fg=ERROR)
        self.ctrl_indicator.pack(side="left", padx=(4, 0))
        self.ctrl_status = tk.StringVar(value="No controller detected")
        tk.Label(r1, textvariable=self.ctrl_status, font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left", padx=4)

        r2 = tk.Frame(card, bg=CARD_BG)
        r2.pack(fill="x", pady=(0, 6))
        make_btn(r2, "Start", self.controller.start_controller, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        make_btn(r2, "Stop", self.controller.stop_controller, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        self.ctrl_toggle_btn = make_btn(r2, "Show", self._toggle_controller_details, width=10, font_norm=self.font_norm)
        self.ctrl_toggle_btn.pack(side="left", padx=4)

        self.ctrl_advanced = tk.Frame(card, bg=CARD_BG)
        
        r3 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r3.pack(fill="x", pady=(0, 4))
        tk.Label(r3, text="Deadzone:", font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left")
        self.deadzone_var = tk.DoubleVar(value=0.25)
        tk.Scale(r3, from_=0.10, to=0.50, resolution=0.05, orient="horizontal", variable=self.deadzone_var, length=180, bg=CARD_BG, fg=FG, troughcolor=BTN_BG, highlightthickness=0, font=self.font_small, command=self._on_ctrl_params_changed).pack(side="left", padx=4)

        r4 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r4.pack(fill="x", pady=(0, 4))
        tk.Label(r4, text="Speed:", font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left")
        self.speed_var = tk.DoubleVar(value=0.50)
        tk.Scale(r4, from_=0.10, to=1.00, resolution=0.05, orient="horizontal", variable=self.speed_var, length=180, bg=CARD_BG, fg=FG, troughcolor=BTN_BG, highlightthickness=0, font=self.font_small, command=self._on_ctrl_params_changed).pack(side="left", padx=4)

        r5 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r5.pack(fill="x", pady=(0, 4))
        self.ctrl_axes_var = tk.StringVar(value="Fwd: 0.00   Rot: 0.00")
        tk.Label(r5, textvariable=self.ctrl_axes_var, font=self.font_guide, bg=CARD_BG, fg=ACCENT).pack(side="left", padx=4)

        r6 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r6.pack(fill="x", pady=(0, 6))
        self.gyro_var = tk.StringVar(value="Tilt X: --   Y: --   Stability: --")
        self.gyro_lbl = tk.Label(r6, textvariable=self.gyro_var, font=self.font_guide, bg=CARD_BG, fg=FG)
        self.gyro_lbl.pack(side="left", padx=4)

        guide = make_card(self.ctrl_advanced, "Button Guide", self.font_head)
        guide_lines = [
            ("Left Stick",       "Forward / Backward only"),
            ("Right Stick",      "Turn left / right"),
            ("[X]  Cross",       "StandInit posture"),
            ("[O]  Circle",      "Sit down"),
            ("[/\\] Triangle",   "Relax (servos off)"),
            ("[ ]  Square",      "Stop controller"),
        ]
        for lbl, desc in guide_lines:
            row = tk.Frame(guide, bg=CARD_BG)
            row.pack(fill="x", anchor="w")
            tk.Label(row, text=lbl,  font=self.font_guide, bg=CARD_BG, fg=ACCENT, width=18, anchor="w").pack(side="left")
            tk.Label(row, text=desc, font=self.font_guide, bg=CARD_BG, fg=FG, anchor="w").pack(side="left")

        self._set_controller_details(False)

    def _on_ctrl_params_changed(self, val=None):
        self.controller.update_settings(self.deadzone_var.get(), self.speed_var.get())

    def _set_controller_details(self, show):
        self._controller_expanded = bool(show)
        if self._controller_expanded:
            self.ctrl_advanced.pack(fill="x", pady=(0, 0))
            self.ctrl_toggle_btn.config(text="Hide")
        else:
            self.ctrl_advanced.pack_forget()
            self.ctrl_toggle_btn.config(text="Show")

    def _toggle_controller_details(self):
        self._set_controller_details(not self._controller_expanded)

    def _card_camera(self, parent):
        card = make_card(parent, "Camera", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x", pady=(0, 6))
        make_btn(row, "Start", self._on_camera_start, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        make_btn(row, "Stop", self._on_camera_stop, width=10, font_norm=self.font_norm).pack(side="left", padx=4)
        self.cam_status_var = tk.StringVar(value="Stopped")
        tk.Label(row, textvariable=self.cam_status_var, font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left", padx=6)

        row2 = tk.Frame(card, bg=CARD_BG)
        row2.pack(fill="x", pady=(0, 4))
        self.cam_detect_var = tk.StringVar(value="Human presence: --")
        self.cam_detect_lbl = tk.Label(row2, textvariable=self.cam_detect_var, font=self.font_small, bg=CARD_BG, fg=FG)
        self.cam_detect_lbl.pack(side="left", padx=4)

        self.cam_preview_lbl = tk.Label(card, text="No feed", bg="#000000", fg=FG, anchor="center")
        self.cam_preview_lbl.pack(fill="x", padx=4, pady=(0, 2))

    def _set_status(self, msg, ok=True):
        self.status_var.set(msg)
        self.status_lbl.config(fg=SUCCESS if ok else ERROR)

    def _require_connection(self):
        if not self.conn.connected:
            self._set_status("Not connected - click Connect first", False)
            return False
        return True

    def _on_connect(self):
        ip = self.ip_entry.get().strip()
        port = int(self.port_entry.get().strip())
        self._set_status("Connecting to %s:%d ..." % (ip, port), True)
        self.root.update_idletasks()
        ok, msg = self.conn.connect(ip, port)
        if ok:
            try:
                if self.conn.audio: self.vol_var.set(self.conn.audio.getOutputVolume())
                if self.conn.tts: self.lang_var.set(self.conn.tts.getLanguage())
                if self.conn.tts: self.conn.tts.say("Connected")
            except Exception:
                pass
            self._set_status(msg, True)
        else:
            self._set_status(msg, False)

    def _on_volume_set(self):
        if not self._require_connection() or not self.conn.audio: return
        vol = self.vol_var.get()
        try:
            self.conn.audio.setOutputVolume(vol)
            self._set_status("Volume set to %d%%" % vol)
        except Exception as e:
            self._set_status("Volume error: %s" % e, False)

    def _on_language_set(self):
        if not self._require_connection() or not self.conn.tts: return
        lang = self.lang_var.get()
        try:
            self.conn.tts.setLanguage(lang)
            self._set_status("Language -> %s" % lang)
        except Exception as e:
            self._set_status("Language error: %s" % e, False)

    def _on_language_get(self):
        if not self._require_connection() or not self.conn.tts: return
        try:
            lang = self.conn.tts.getLanguage()
            self.lang_var.set(lang)
            self._set_status("Current language: %s" % lang)
        except Exception as e:
            self._set_status("Language error: %s" % e, False)

    def _on_speak(self):
        if not self._require_connection() or not self.conn.tts: return
        text = self.speech_entry.get().strip()
        if not text: return
        def _say():
            try: self.conn.tts.say(str(text))
            except Exception: pass
        threading.Thread(target=_say).start()
        self._set_status("Speaking: \"%s\"" % text)

    def _on_led_colour(self, colour_hex):
        if not self._require_connection() or not self.conn.leds: return
        group = self.led_group_var.get()
        try:
            self.conn.leds.fadeRGB(group, colour_hex, 0.3)
            self._set_status("LEDs %s -> #%06x" % (group, colour_hex))
        except Exception as e:
            self._set_status("LED error: %s" % e, False)

    def _on_posture(self):
        if not self._require_connection() or not self.conn.posture: return
        name = self.posture_var.get()
        self._set_status("Going to posture: %s ..." % name)
        self.root.update_idletasks()
        def _go():
            try:
                if name == "Relax":
                    self.conn.motion.rest()
                else:
                    self.conn.posture.goToPosture(name, 0.8)
                self._set_status("Posture: %s" % name)
            except Exception as e:
                self._set_status("Posture error: %s" % e, False)
        threading.Thread(target=_go).start()

    def _on_life_on(self):
        if not self._require_connection() or not self.conn.life: return
        try:
            self.conn.life.setState("solitary")
            self.life_status.set("ON")
            self._set_status("Autonomous life enabled")
        except Exception as e:
            self._set_status("Life error: %s" % e, False)

    def _on_life_off(self):
        if not self._require_connection() or not self.conn.life: return
        try:
            self.conn.life.setState("disabled")
            self.life_status.set("OFF")
            self._set_status("Autonomous life disabled")
        except Exception as e:
            self._set_status("Life error: %s" % e, False)

    def _on_gemini_ask(self):
        if not self._require_connection() or not self.conn.tts:
            self._set_status("Cannot speak without NAO connected.", False)
            return

        api_key = self.gemini_key_entry.get().strip()
        prompt = self.gemini_prompt_entry.get().strip()

        if not api_key:
            self.gemini_status.set("Error: Need API Key!")
            return
        if not prompt:
            self.gemini_status.set("Enter a prompt!")
            return

        self.gemini_status.set("Thinking...")
        self.root.update_idletasks()

        def _fetch_and_say():
            try:
                from nao_app.ai.gemini_client import GeminiClient
                client = GeminiClient()
                client.set_api_key(api_key)
                
                # The Gemini AI client natively handles the NAO system prompt, 
                # keeping the UI logic perfectly clean.
                response_text = client.generate_text(prompt)
                
                if "Error" in response_text:
                    self.gemini_status.set("Gemini Error.")
                    self._set_status("Gemini Error: Check console.", False)
                    print(response_text)
                else:
                    self.gemini_status.set("Finished!")
                    self._set_status("Speaking Gemini response...")
                    
                    # Convert accurately to UTF-8 without newlines for the NAOqi 2.7 TTS engine
                    safe_text = response_text.replace('\n', ' ').encode('utf-8', 'ignore')
                    print("Gemini said: " + safe_text)
                    self.conn.tts.say(safe_text)
            except Exception as e:
                self.gemini_status.set("Failed: %s" % e)
                self._set_status("Gemini Failed: %s" % e, False)
                
        threading.Thread(target=_fetch_and_say).start()

    def _on_gemini_voice_interactive(self):
        if not self._require_connection() or not self.conn.audio_recorder:
            return
            
        api_key = self.gemini_key_entry.get().strip()
        if not api_key:
            self.gemini_status.set("Error: Need API Key!")
            return

        def _voice_flow():
            import time, urllib2
            try:
                # Tell NAO to use its built-in microphones to record a WAV file locally
                self.conn.audio_recorder.stopMicrophonesRecording()
                self.conn.audio_recorder.startMicrophonesRecording("/home/nao/gemini.wav", "wav", 16000, (0,0,1,0))
                self.gemini_status.set("Listening to you (5s)...")
                self.conn.leds.fadeRGB("FaceLeds", 0x00FF0000, 0.2) # Turn eyes red so we know it listens
                
                # Wait 5 seconds dynamically while letting UI update
                for i in range(5):
                    self.gemini_status.set("Listening to you (%ds left)..." % (5-i))
                    time.sleep(1)
                    
                self.conn.audio_recorder.stopMicrophonesRecording()
                self.conn.leds.fadeRGB("FaceLeds", 0x000000FF, 0.2) # Turn eyes blue to show processing
                self.gemini_status.set("Thinking...")
                
                # Use python's urllib to FTP into the robot and pull the raw WAV file securely
                ftp_url = "ftp://nao:nao@{}/gemini.wav".format(self.conn.ip)
                wav_bytes = urllib2.urlopen(ftp_url, timeout=10).read()
                
                from nao_app.ai.gemini_client import GeminiClient
                client = GeminiClient()
                client.set_api_key(api_key)
                
                # We let the user type a custom extra prompt, or use a default one
                prompt = self.gemini_prompt_entry.get().strip()
                if not prompt:
                    prompt = "Please listen to the attached audio recording of my voice. Answer what I say naturally."
                
                response_text = client.generate_text(prompt, audio_bytes=wav_bytes)
                
                self.conn.leds.fadeRGB("FaceLeds", 0x00FFFFFF, 0.2) # Back to normal white eyes
                
                if "Error" in response_text:
                    self.gemini_status.set("Gemini Voice Error.")
                    self._set_status(response_text, False)
                    print(response_text)
                else:
                    self.gemini_status.set("Finished!")
                    self._set_status("Speaking Gemini response...")
                    safe_text = response_text.replace('\n', ' ').encode('utf-8', 'ignore')
                    print("Gemini voice response: " + safe_text)
                    self.conn.tts.say(safe_text)
            except Exception as e:
                self.gemini_status.set("Voice Error: %s" % str(e)[:30])
                self._set_status("Voice Pipeline Failed: %s" % e, False)
                try: 
                    self.conn.leds.fadeRGB("FaceLeds", 0x00FFFFFF, 0.2)
                except: 
                    pass
        
        threading.Thread(target=_voice_flow).start()

    def _on_battery(self):
        if not self._require_connection() or not self.conn.memory: return
        try:
            level = self.conn.memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
            pct = int(level * 100)
            self.battery_var.set("%d%%" % pct)
            self._set_status("Battery: %d%%" % pct)
        except Exception as e:
            self._set_status("Battery error: %s" % e, False)

    def _on_camera_start(self):
        if not self._require_connection(): return
        self._set_controller_details(False)
        self.vision.start_camera(self.conn.ip, self.conn.port, self.root.after)
        self._poll_camera()
        
    def _poll_camera(self):
        if self.vision._cam_running:
            if self.vision._cam_photo is not None:
                self.cam_preview_lbl.config(image=self.vision._cam_photo, text="")
            else:
                self.cam_preview_lbl.config(text="No feed")
                
            # fetch the UI updates handled inside vision
            msg = self.vision.ui.get("cam_detect_msg", "")
            if msg:
                self.cam_detect_var.set(msg)
                color = self.vision.ui.get("cam_detect_color", FG)
                self.cam_detect_lbl.config(fg=color)
                
            self.root.after(150, self._poll_camera)

    def _on_camera_stop(self):
        self.vision.stop_camera(self.root.after_cancel, self.vision.ui.get("cam_after_id"))
        self.cam_preview_lbl.config(image="", text="No feed")
        self.cam_status_var.set("Stopped")
        self.cam_detect_var.set("Human presence: --")
        self.cam_detect_lbl.config(fg=FG)

    def _on_controller_detect_cb(self, success, text):
        if hasattr(self, 'ctrl_status'):
            self.ctrl_status.set(text)
            self.ctrl_indicator.config(fg=SUCCESS if success else ERROR)
            if success:
                self._set_status("Controller: %s" % text)
            else:
                self._set_status(text, False)

    def _on_close(self):
        self._on_camera_stop()
        self.controller.stop_controller()
        self.root.destroy()

    def run(self):
        # We need a polling mechanism if the controller loop tries to update UI variables 
        # But wait, controller sets variables directly in its loop?
        # Actually in the new structure it would need callbacks. The user wanted things separated.
        # We will poll controller updates
        self._poll_controller()
        self.root.mainloop()

    def _poll_controller(self):
        if self.controller._ctrl_running:
            self.ctrl_axes_var.set("Fwd: %.2f   Rot: %.2f" % (self.controller._cur_fwd, self.controller._cur_rot))
            # get gyro from controller
            tilt_x, tilt_y = self.controller._read_tilt()
            if self.controller._is_robot_standing():
                stab = self.controller._stability_factor(tilt_x, tilt_y)
                if stab >= 0.8:
                    stab_txt, stab_col = "OK", SUCCESS
                elif stab > 0.0:
                    stab_txt, stab_col = "CAUTION (%d%%)" % int(stab*100), WARN
                else:
                    stab_txt, stab_col = "DANGER", ERROR
            else:
                stab_txt, stab_col = "N/A (not standing)", FG
            
            self.gyro_var.set("Tilt X:%+.2f  Y:%+.2f   Stability: %s" % (tilt_x, tilt_y, stab_txt))
            self.gyro_lbl.config(fg=stab_col)
            
        self.root.after(100, self._poll_controller)

