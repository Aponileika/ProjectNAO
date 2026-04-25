# -*- coding: utf-8 -*-
import Tkinter as tk
import tkFont
import threading
from nao_app.ui.widgets import make_card, make_btn, BG, FG, ACCENT, SUCCESS, ERROR, WARN, CARD_BG, BTN_BG, BTN_FG

class NaoAppWindow(object):
    LANGUAGES = [
        "English", "German", "Spanish", "Dutch", "Polish", "Czech", "Swedish", "Danish", "Norwegian",
        "Finnish", "Russian",
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
        except Exception as e:
            print("[NaoAppWindow] Could not load config/secrets: %s" % e)

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
        self._card_autonomous(right)
        self._card_quick_command(right)
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
        make_btn(r3, "Stop Speech", self._on_stop_speech, width=10, font_norm=self.font_norm, bg="#bb3333", fg="white").pack(side="left", padx=2)
        
        r4 = tk.Frame(card, bg=CARD_BG)
        r4.pack(fill="x")
        self.gemini_status = tk.StringVar(value="Ready.")
        tk.Label(r4, textvariable=self.gemini_status, font=self.font_small, bg=CARD_BG, fg=ACCENT).pack(side="left")
        
        self.require_face = tk.BooleanVar(value=False)
        tk.Checkbutton(r4, text="Wait for Face", variable=self.require_face, font=self.font_small, bg=CARD_BG, fg=FG, selectcolor=CARD_BG, activebackground=CARD_BG, activeforeground=FG).pack(side="right")

    def _card_autonomous(self, parent):
        card = make_card(parent, "Autonomous Wander", self.font_head)
        
        info = tk.Label(card, text="NAO will walk, avoid walls using Sonar, and stop when it spots a human face.", 
                        font=self.font_small, bg=CARD_BG, fg=FG, wraplength=280, justify="left")
        info.pack(fill="x", pady=(0, 4))
        
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=2)
        make_btn(r1, "Wander & Seek", self._start_wander_seek, width=16, font_norm=self.font_norm).pack(side="left", padx=2)
        make_btn(r1, "Stop Auto", self._stop_wander_seek, width=10, font_norm=self.font_norm, bg="#bb3333", fg="white").pack(side="left", padx=2)
        
        r2 = tk.Frame(card, bg=CARD_BG)
        r2.pack(fill="x")
        self.auto_status = tk.StringVar(value="Idle.")
        tk.Label(r2, textvariable=self.auto_status, font=self.font_small, bg=CARD_BG, fg=ACCENT).pack(side="left")

    def _card_quick_command(self, parent):
        card = make_card(parent, "Quick Text Commands", self.font_head)
        
        info = tk.Label(card, text="Type simple verbs (e.g. 'turn red', 'walk forward', 'sit', 'say hello', 'stop')", 
                        font=self.font_small, bg=CARD_BG, fg=FG, wraplength=280, justify="left")
        info.pack(fill="x", pady=(0, 4))
        
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=2)
        
        self.quick_cmd_var = tk.StringVar()
        self.quick_cmd_entry = tk.Entry(r1, textvariable=self.quick_cmd_var, font=self.font_norm, width=28)
        self.quick_cmd_entry.pack(side="left", padx=4)
        
        make_btn(r1, "Do", self._on_quick_command, width=6, font_norm=self.font_norm).pack(side="left", padx=2)
        
        self.quick_cmd_entry.bind("<Return>", lambda e: self._on_quick_command())
        
        r2 = tk.Frame(card, bg=CARD_BG)
        r2.pack(fill="x")
        self.quick_status = tk.StringVar(value="Ready.")
        tk.Label(r2, textvariable=self.quick_status, font=self.font_small, bg=CARD_BG, fg=ACCENT).pack(side="left")

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
                    if name in ("Sit", "SitRelax", "Crouch"):
                        self.conn.posture.goToPosture("StandInit", 0.5)
                        self.conn.posture.goToPosture(name, 0.5)
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

    @staticmethod
    def _rgb_to_png(width, height, rgb_payload):
        """Encode raw 24-bit RGB bytes as a PNG using only Python 2.7 stdlib.
        No Pillow, no subprocess, no temp files required.
        Returns the PNG as a byte string."""
        import zlib
        import struct

        if isinstance(rgb_payload, bytearray):
            rgb_payload = bytes(rgb_payload)

        def _chunk(tag, data):
            crc = zlib.crc32(tag + data) & 0xffffffff
            return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)

        row_size = width * 3
        # PNG filter byte 0 (None) prepended to each row
        raw_rows = b''.join(
            b'\x00' + rgb_payload[y * row_size:(y + 1) * row_size]
            for y in range(height)
        )

        return (
            b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
            + _chunk(b'IDAT', zlib.compress(raw_rows, 6))
            + _chunk(b'IEND', b'')
        )

    def _capture_image_bytes(self):
        """Capture one frame from NAO's camera and return it as PNG bytes,
        or None if capture fails.  Works entirely within Python 2.7 stdlib —
        no Pillow, no subprocess, no temp files."""
        video = self.conn.video
        if not video:
            print("[Vision] No video proxy (conn.video is None - is NAO connected?)")
            return None

        w, h, payload = None, None, None

        # Re-use the live camera frame if the camera tab is already streaming,
        # to avoid fighting over the hardware subscription.
        if (getattr(self, 'vision', None)
                and getattr(self.vision, '_cam_running', False)
                and getattr(self.vision, '_last_raw_img', None)):
            self._set_status("Lifting live camera frame for AI...")
            w, h, payload = self.vision._last_raw_img
        else:
            self._set_status("Snapping a photo for the AI...")
            import time
            try: video.unsubscribe("gemini_snap")
            except Exception: pass
            try:
                cam_name = video.subscribe("gemini_snap", 2, 11, 5)
                video.getImageRemote(cam_name)      # discard first (dark) frame
                time.sleep(0.5)                     # let auto-exposure settle
                img_data = video.getImageRemote(cam_name)
                video.unsubscribe(cam_name)
                if img_data and len(img_data) >= 7:
                    w, h, payload = int(img_data[0]), int(img_data[1]), img_data[6]
                else:
                    print("[Vision] getImageRemote returned empty/short data: %s" % repr(img_data))
            except Exception as e:
                print("[Vision] Camera subscribe/capture error: %s" % e)
                return None

        if not (w and h and payload):
            print("[Vision] No valid pixel data (w=%s h=%s payload_len=%s)." % (
                w, h, len(payload) if payload else 0))
            return None

        try:
            png_bytes = self._rgb_to_png(w, h, payload)
            print("[Vision] Captured %dx%d -> %d bytes PNG" % (w, h, len(png_bytes)))
            return png_bytes
        except Exception as e:
            print("[Vision] PNG encoding failed: %s" % e)
            return None

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
                
                image_bytes = self._capture_image_bytes()
                if image_bytes is None:
                    self._set_status("No image captured - sending text prompt only.", False)

                response_text = client.generate_text(prompt, image_bytes=image_bytes)
                
                if "Error" in response_text:
                    self.gemini_status.set("Gemini Error.")
                    self._set_status("Gemini Error: Check console.", False)
                    print(response_text)
                else:
                    self._speak_when_face_found(response_text)
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
                self.conn.leds.fadeRGB("AllLeds", 0x00FF0000, 0.2) # Turn eyes red so we know it listens
                
                # Wait 5 seconds dynamically while letting UI update
                for i in range(5):
                    self.gemini_status.set("Listening to you (%ds left)..." % (5-i))
                    time.sleep(1)
                    
                self.conn.audio_recorder.stopMicrophonesRecording()
                self.conn.leds.fadeRGB("AllLeds", 0x000000FF, 0.2) # Turn eyes blue to show processing
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
                
                # Take a picture from the robot's eyes to send to Gemini
                image_bytes = self._capture_image_bytes()
                if image_bytes is None:
                    self._set_status("No image captured - sending audio only.", False)
                
                response_text = client.generate_text(prompt, audio_bytes=wav_bytes, image_bytes=image_bytes)
                
                self.conn.leds.fadeRGB("AllLeds", 0x00FFFFFF, 0.2) # Back to normal white eyes

                if "Error" in response_text:
                    self.gemini_status.set("Gemini Voice Error.")
                    self._set_status(response_text, False)
                    print(response_text)
                else:
                    self._speak_when_face_found(response_text)
            except Exception as e:
                self.gemini_status.set("Voice Error: %s" % str(e)[:30])
                self._set_status("Voice Pipeline Failed: %s" % e, False)
                try: 
                    self.conn.leds.fadeRGB("AllLeds", 0x00FFFFFF, 0.2)
                except Exception:
                    pass
        
        threading.Thread(target=_voice_flow).start()

    def _on_stop_speech(self):
        if not self._require_connection() or not self.conn.tts:
            return
        try:
            self.conn.tts.stopAll()
            self._set_status("Speech forcefully stopped.")
        except Exception as e:
            self._set_status("Failed to stop speech: %s" % e, False)

    def _speak_when_face_found(self, response_text):
        command_to_run = None
        if "COMMAND:" in response_text:
            try:
                parts = response_text.split("COMMAND:", 1)[1].split(".", 1)
                command_to_run = parts[0].strip().lower()
                response_text = parts[1].strip() if len(parts) > 1 else "Done."
            except Exception:
                pass
                
        self.gemini_status.set("Waiting for face...")
        self._set_status("Looking for you before answering...")
        
        face_found = False
        try:
            tracker = self.conn.get_proxy("ALTracker")
            if not self.require_face.get():
                face_found = True # Skip searching if requirement is disabled
            elif tracker and self.conn.face and self.conn.memory:
                # Force head stiffness to 1.0 so manual scanning actually moves the physical neck motors
                if self.conn.motion:
                    self.conn.motion.setStiffnesses("Head", 1.0)
                    
                # Wake up the face detection engine
                self.conn.face.subscribe("VoiceTracker")
                
                import time
                import math
                
                # Wait up to 4 seconds to spot someone
                for i in range(4):
                    val = self.conn.memory.getData("FaceDetected")
                    # Check if face data is valid
                    if val and isinstance(val, list) and len(val) >= 2 and isinstance(val[1], list) and len(val[1]) > 0:
                        # Found face! Hand control over to ALTracker so it looks at us while speaking
                        tracker.registerTarget("Face", 0.15)
                        tracker.setMode("Head")
                        tracker.track("Face")
                        face_found = True
                        break
                        
                    # Actively pan head side-to-side and up-and-down manually
                    # (ALTracker is OFF here so it doesn't fight our manual commands)
                    if self.conn.motion:
                        yaw = math.sin(i * 0.8) * 0.8    # Look side to side
                        # Math.cos forces the pitch to be out of phase with yaw, causing an oval/figure-8 sweep, and increased amplitude to 0.45 
                        pitch = math.cos(i * 0.5) * 0.45 # Distinctly look up and down
                        try: 
                            self.conn.motion.setAngles(["HeadYaw", "HeadPitch"], [yaw, pitch], 0.15)
                        except Exception: 
                            pass
                    
                    time.sleep(1)
                    
        except Exception as fe:
            print("Face tracking non-fatal error: " + str(fe))
            
        if not face_found:
            # Prepend the fallback phrase to the mobster's actual response
            response_text = "Even if I can't see you, I will do as you say this time. " + response_text

        self.gemini_status.set("Finished!")
        self._set_status("Speaking Gemini response...")
        safe_text = response_text.replace('\n', ' ').encode('utf-8', 'ignore')
        print("Gemini response: " + safe_text)
        
        if command_to_run:
            self._set_status("Firing voice command: " + command_to_run)
            def do_command():
                import time
                time.sleep(0.5) # Slight delay purely for dramatic effect
                self._on_quick_command(command_to_run)
            threading.Thread(target=do_command).start()
            
        self.conn.tts.say(safe_text)
        
        # Give it a second to finish speaking before dropping tracker
        try:
            import time
            time.sleep(1)
            tracker = self.conn.get_proxy("ALTracker")
            if tracker:
                tracker.stopTracker()
                tracker.unregisterAllTargets()
            if self.conn.face:
                self.conn.face.unsubscribe("VoiceTracker")
            # Relax the head motors to prevent overheating
            if self.conn.motion:
                self.conn.motion.setStiffnesses("Head", 0.0)
        except Exception as e:
            print("[NaoAppWindow] Face tracker stop error: %s" % e)

    def _on_battery(self):
        if not self._require_connection() or not self.conn.memory: return
        try:
            level = self.conn.memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
            pct = int(level * 100)
            self.battery_var.set("%d%%" % pct)
            self._set_status("Battery: %d%%" % pct)
        except Exception as e:
            self._set_status("Battery error: %s" % e, False)

    def _on_quick_command(self, cmd_text=None):
        if not self._require_connection(): return
        
        cmd = (cmd_text or self.quick_cmd_var.get()).strip().lower()
        if not cmd: return
        
        self.quick_status.set("Doing: " + cmd)
        
        def _execute_cmd():
            try:
                # Stop autonomous wandering if it is running so it doesn't fight the manual commands
                if getattr(self, "_seeking", False):
                    self._stop_wander_seek()
                    import time
                    time.sleep(0.5) # Wait a moment for loop to safely exit and motors to reset
                    
                # 1. Colors
                if "red" in cmd: self.conn.leds.fadeRGB("AllLeds", 0xFF0000, 0.2)
                elif "blue" in cmd: self.conn.leds.fadeRGB("AllLeds", 0x0000FF, 0.2)
                elif "green" in cmd: self.conn.leds.fadeRGB("AllLeds", 0x00FF00, 0.2)
                elif "white" in cmd: self.conn.leds.fadeRGB("AllLeds", 0xFFFFFF, 0.2)
                elif "off" in cmd and "led" in cmd: self.conn.leds.fadeRGB("AllLeds", 0x000000, 0.2)
                
                # 2. Postures
                words = cmd.replace('.', '').replace(',', '').split()
                if "sit" in words or "sit down" in cmd: 
                    self.conn.posture.goToPosture("StandInit", 0.5)
                    self.conn.posture.goToPosture("Sit", 0.5)
                elif "stand" in cmd: 
                    if hasattr(self.controller, "_stand_safely"): self.controller._stand_safely()
                    else: self.conn.posture.goToPosture("StandInit", 0.8)
                elif "crouch" in cmd:
                    self.conn.posture.goToPosture("StandInit", 0.5)
                    self.conn.posture.goToPosture("Crouch", 0.5)
                elif "relax" in cmd or "rest" in cmd: self.conn.motion.rest()
                
                # 3. Motion
                walk_cfg = getattr(self.controller, "_WALK_CONFIG", [])
                
                # Check for wandering first, as it conflicts with basic walking
                if "wander" in cmd or "autonomously" in cmd or "seek" in cmd:
                    # Let the existing autonomous thread logic handle this
                    import threading
                    threading.Thread(target=self._start_wander_seek).start()
                    return
                elif "forward" in cmd or "walk" in cmd: 
                    self.conn.motion.moveToward(0.3, 0.0, 0.0, walk_cfg)
                elif "backward" in cmd or "back" in cmd: 
                    self.conn.motion.moveToward(-0.3, 0.0, 0.0, walk_cfg)
                elif "left" in cmd: 
                    self.conn.motion.moveToward(0.0, 0.0, 0.4, walk_cfg)
                elif "right" in cmd: 
                    self.conn.motion.moveToward(0.0, 0.0, -0.4, walk_cfg)
                elif "stop" in cmd or "halt" in cmd: 
                    self.conn.motion.stopMove()
                    
                # 4. Speech
                if "say " in cmd:
                    phrase = cmd.split("say ", 1)[-1]
                    self.conn.tts.post.say(phrase)
                elif "speak " in cmd:
                    phrase = cmd.split("speak ", 1)[-1]
                    self.conn.tts.post.say(phrase)
                
                self.quick_cmd_var.set("") # Clear field
                self._set_status("Command processed: " + cmd)
                self.quick_status.set("Done.")
            except Exception as e:
                self.quick_status.set("Error!")
                self._set_status("Quick cmd failed: " + str(e), False)
                
        threading.Thread(target=_execute_cmd).start()

    def _start_wander_seek(self):
        if getattr(self, "_seeking", False): return
        if not self._require_connection() or not self.conn.motion:
            self.auto_status.set("Need connection!")
            return
        
        self._seeking = True
        self.auto_status.set("Wandering & Scanning...")
        self._set_status("Autonomous wander started. Use 'Stop Auto' to abort.")
        threading.Thread(target=self._wander_seek_thread).start()

    def _stop_wander_seek(self):
        if getattr(self, "_seeking", False):
            self._seeking = False
            self.auto_status.set("Stopping...")
            self._set_status("Stopping autonomous mode...")

    def _wander_seek_thread(self):
        try:
            self._set_status("Standing up safely...")
            
            # Use the robust safety parameters from the controller setup
            if hasattr(self.controller, "_stand_safely"):
                standing = self.controller._stand_safely()
                if not standing:
                    self.auto_status.set("Failed to stand.")
                    self._set_status("Failed to stand, aborting wander.", False)
                    self._seeking = False
                    return
            else:
                self.conn.motion.wakeUp()
                if self.conn.posture:
                    self.conn.posture.goToPosture("StandInit", 0.5)

            # Subscribe to Sonar for obstacle avoidance
            sonar = self.conn.get_proxy("ALSonar")
            if sonar:
                try: sonar.subscribe("WanderSeeker")
                except Exception: pass
            
            # Subscribe to Face Detection
            if self.conn.face:
                try: self.conn.face.subscribe("WanderSeekerFace")
                except Exception: pass
                
            tracker = self.conn.get_proxy("ALTracker")
            
            import time
            import math
            import random
            
            wander_turn_bias = 0.0
            last_bias_change = time.time()
            next_speech_time = time.time() + random.uniform(3.0, 20.0)
            
            phrases = [
                "Where is my human?",
                "I am lonely.",
                "Come out, come out, wherever you are.",
                "I know you're hiding somewhere in this joint.",
                "Where's this guy hiding?",
                "Gettin' kinda bored wandering around here.",
                "Show your face."
            ]
            
            while getattr(self, "_seeking", False):
                # 1. Check for faces
                val = None
                if self.conn.memory:
                    val = self.conn.memory.getData("FaceDetected")
                    
                if getattr(self, "_seeking", False) and val and isinstance(val, list) and len(val) >= 2 and isinstance(val[1], list) and len(val[1]) > 0:
                    # Found human!
                    self._seeking = False
                    self.conn.motion.stopMove()
                    self.auto_status.set("Found Human!")
                    
                    if tracker:
                        tracker.registerTarget("Face", 0.15)
                        tracker.setMode("Head")
                        tracker.track("Face")
                    
                    if self.conn.tts:
                        self.conn.tts.say("Well well well, there you are. I've been looking all over for you.")
                    
                    time.sleep(2)
                    if tracker:
                        tracker.stopTracker()
                        tracker.unregisterAllTargets()
                    break

                # 2. Check sonar for obstacles
                l_dist = 1.0
                r_dist = 1.0
                if getattr(self, "_seeking", False) and self.conn.memory:
                    try:
                        # ALSonar writes to Device/SubDeviceList/US/Left/Sensor/Value etc.
                        l_dist = self.conn.memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
                        r_dist = self.conn.memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
                    except Exception:
                        pass
                
                if not getattr(self, "_seeking", False):
                    break
                    
                # 3. Simple Obstacle Avoidance & Movement
                # Use the controller's safety and constraints
                max_fwd = 0.15 # Gentle speed, don't rush
                max_rot = 0.16 # Hardcoded max rotation from controller
                walk_config = getattr(self.controller, "_WALK_CONFIG", [])

                if l_dist < 0.60 or r_dist < 0.60:
                    # Very close to an object: Stop immediately so it doesn't hit it
                    self.conn.motion.stopMove()
                    
                    if self.conn.tts:
                        try:
                            # using post.say so the robot speaks asynchronously while moving
                            self.conn.tts.post.say("Whoa, blocked! Backing up to find a new route.")
                        except Exception: 
                            pass
                            
                    # Walk backwards to clear the space
                    if getattr(self, "_seeking", False):
                        self.conn.motion.moveToward(-0.15, 0.0, 0.0, walk_config)
                        for _ in range(15): # Give it 1.5 seconds of straight backward walking
                            if not getattr(self, "_seeking", False): break
                            time.sleep(0.1)
                            
                    # Pivot randomly to establish a truly new route
                    if getattr(self, "_seeking", False):
                        # Base pivot direction away from obstacle
                        base_speed = random.uniform(0.20, 0.35)
                        turn_dir = -base_speed if l_dist < r_dist else base_speed
                        
                        # 20% chance to completely fake out and spin the other way (helps break loops)
                        if random.random() < 0.20:
                            turn_dir *= -1.0
                            
                        self.conn.motion.moveToward(0.0, 0.0, turn_dir, walk_config)
                        
                        # Randomize how long it turns (between 1.5 seconds and 3.5 seconds)
                        # This ensures the new angle is drastically different every single time
                        pivot_ticks = int(random.uniform(15, 35))
                        for _ in range(pivot_ticks): 
                            if not getattr(self, "_seeking", False): break
                            time.sleep(0.1)
                else:
                    # Occasional random wandering speech
                    if time.time() > next_speech_time:
                        if self.conn.tts:
                            try: self.conn.tts.post.say(random.choice(phrases))
                            except Exception: pass
                        next_speech_time = time.time() + random.uniform(3.0, 20.0)

                    # Randomize wander drift periodically (every 4 to 8 seconds)
                    if time.time() - last_bias_change > random.uniform(4.0, 8.0):
                        wander_turn_bias = random.uniform(-0.15, 0.15)
                        last_bias_change = time.time()
                        
                    # Clear path, walk slightly forward and slowly sweep room with random bias
                    turn = (math.sin(time.time() * 0.5) * 0.08) + wander_turn_bias
                    self.conn.motion.moveToward(max_fwd, 0.0, turn, walk_config)
                    
                time.sleep(0.1)
                
        except Exception as e:
            self._set_status("Wander error: " + str(e)[:30], False)
            self.auto_status.set("Error: View console")
            print("Wander error: " + str(e))
            
        finally:
            self._seeking = False
            self.auto_status.set("Idle.")
            try:
                self.conn.motion.stopMove()
            except Exception: pass
            
            try:
                sonar = self.conn.get_proxy("ALSonar")
                if sonar: sonar.unsubscribe("WanderSeeker")
            except Exception: pass
            
            try:
                if self.conn.face: self.conn.face.unsubscribe("WanderSeekerFace")
            except Exception: pass


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

