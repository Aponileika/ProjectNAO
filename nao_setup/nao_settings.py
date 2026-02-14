#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""NAO Settings - Control Panel GUI.

A Tkinter GUI that lets you connect to NAO and control:
  - Volume, Language, Speech
  - LED colours
  - Posture, Autonomous life
  - Battery
  - PS5 controller with gyro-stabilised walking
"""
import os
import sys
import subprocess
import math

# ---- Auto-relaunch with bundled Python 2.7 if running Python 3 ----
if sys.version_info[0] != 2:
    _base = os.path.dirname(os.path.abspath(__file__))
    _root = os.path.dirname(_base)
    _bundled_py  = os.path.join(_root, 'Python', 'python.exe')
    _bundled_lib = os.path.join(
        _root,
        'pynaoqi-python2.7-2.8.6.23-win64-vs2015-20191127_152649',
        'lib')
    if os.path.isfile(_bundled_py):
        _env = os.environ.copy()
        _env['PYTHONPATH'] = _bundled_lib
        _env['PATH'] = _bundled_lib + os.pathsep + _env.get('PATH', '')
        _rc = subprocess.call(
            [_bundled_py, os.path.abspath(__file__)] + sys.argv[1:],
            env=_env)
        sys.exit(_rc)

# ---- Imports (Python 2.7) ------------------------------------------
import Tkinter as tk
import tkFont
import tkMessageBox
import socket
import threading
import time

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

ALProxy = None
try:
    from naoqi import ALProxy as _ALProxy
    ALProxy = _ALProxy
except Exception:
    pass


# =====================================================================
#  Colour palette  (Catppuccin Mocha inspired)
# =====================================================================
BG       = "#1e1e2e"
FG       = "#cdd6f4"
ACCENT   = "#89b4fa"
SUCCESS  = "#a6e3a1"
ERROR    = "#f38ba8"
WARN     = "#fab387"
CARD_BG  = "#313244"
BTN_BG   = "#45475a"
BTN_FG   = "#cdd6f4"


# =====================================================================
#  Helpers
# =====================================================================
def _make_proxy(name, ip, port):
    if ALProxy is None:
        return None
    try:
        return ALProxy(name, str(ip), int(port))
    except Exception:
        return None

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _lerp(current, target, factor):
    return current + (target - current) * factor


# =====================================================================
#  Main Application
# =====================================================================
class NaoSettingsApp(object):

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
        "LyingBelly", "LyingBack",
    ]

    # -----------------------------------------------------------------
    #  init
    # -----------------------------------------------------------------
    def __init__(self):
        self.ip   = "192.168.0.123"
        self.port = 9559
        self.connected = False

        # Proxy cache
        self.tts = self.leds = self.motion = self.posture = None
        self.memory = self.life = self.audio = self.system = None

        self._build_ui()

    # =================================================================
    #  Build the entire UI
    # =================================================================
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("NAO Control Panel")
        self.root.configure(bg=BG)
        self.root.state("zoomed")  # start maximised

        # ---- Fonts --------------------------------------------------
        self.font_title = tkFont.Font(family="Segoe UI", size=18, weight="bold")
        self.font_head  = tkFont.Font(family="Segoe UI", size=11, weight="bold")
        self.font_norm  = tkFont.Font(family="Segoe UI", size=10)
        self.font_small = tkFont.Font(family="Segoe UI", size=9)
        self.font_guide = tkFont.Font(family="Consolas",  size=10)

        # ---- Title bar + status line --------------------------------
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=16, pady=(10, 0))
        tk.Label(top, text="NAO Control Panel",
                 font=self.font_title, bg=BG, fg=ACCENT).pack(side="left")
        self.status_var = tk.StringVar(value="Not connected")
        self.status_lbl = tk.Label(top, textvariable=self.status_var,
                                   font=self.font_norm, bg=BG, fg=ERROR)
        self.status_lbl.pack(side="right")

        # ---- Scrollable body ----------------------------------------
        canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        vsb = tk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        self._body = tk.Frame(canvas, bg=BG)
        self._body.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self._body, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(fill="both", expand=True, padx=12, pady=8)
        self._canvas = canvas

        # Mousewheel scroll
        canvas.bind_all(
            "<MouseWheel>",
            lambda e: canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))

        # Stretch body when canvas resizes
        def _on_canvas_resize(event):
            canvas.itemconfigure("bodywin", width=event.width)
        canvas.bind("<Configure>", _on_canvas_resize)
        canvas.create_window((0, 0), window=self._body, anchor="nw",
                             tags="bodywin")

        # Two equal columns
        self._body.columnconfigure(0, weight=1)
        self._body.columnconfigure(1, weight=1)

        # ---- LEFT column --------------------------------------------
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

        # ---- RIGHT column  (PS5 controller) -------------------------
        right = tk.Frame(self._body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        self._card_controller(right)

    # -----------------------------------------------------------------
    #  Widget helpers
    # -----------------------------------------------------------------
    def _make_card(self, parent, title):
        card = tk.LabelFrame(
            parent, text="  " + title + "  ",
            font=self.font_head, bg=CARD_BG, fg=ACCENT,
            bd=1, relief="groove", padx=10, pady=8)
        card.pack(fill="x", pady=4)
        return card

    def _make_btn(self, parent, text, command, width=14):
        return tk.Button(
            parent, text=text, command=command, width=width,
            font=self.font_norm, bg=BTN_BG, fg=BTN_FG,
            activebackground=ACCENT, activeforeground=BG,
            relief="flat", cursor="hand2")

    # =================================================================
    #  LEFT column cards
    # =================================================================

    # ---- Connection -------------------------------------------------
    def _card_connection(self, parent):
        card = self._make_card(parent, "Connection")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="IP:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.ip_entry = tk.Entry(row, font=self.font_norm, width=16)
        self.ip_entry.insert(0, self.ip)
        self.ip_entry.pack(side="left", padx=4)
        tk.Label(row, text="Port:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.port_entry = tk.Entry(row, font=self.font_norm, width=6)
        self.port_entry.insert(0, str(self.port))
        self.port_entry.pack(side="left", padx=4)
        self._make_btn(row, "Connect", self._on_connect, width=10).pack(
            side="left", padx=6)

    # ---- Volume -----------------------------------------------------
    def _card_volume(self, parent):
        card = self._make_card(parent, "Volume")
        self.vol_var = tk.IntVar(value=50)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="0", font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left")
        tk.Scale(row, from_=0, to=100, orient="horizontal",
                 variable=self.vol_var, length=220,
                 bg=CARD_BG, fg=FG, troughcolor=BTN_BG,
                 highlightthickness=0,
                 font=self.font_small).pack(side="left", padx=4)
        tk.Label(row, text="100", font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self._make_btn(row, "Set", self._on_volume_set, width=6).pack(
            side="left", padx=6)

    # ---- Language ---------------------------------------------------
    def _card_language(self, parent):
        card = self._make_card(parent, "Language")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self.lang_var = tk.StringVar(value="English")
        tk.Label(row, text="Language:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        m = tk.OptionMenu(row, self.lang_var, *self.LANGUAGES)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG,
                 activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)
        self._make_btn(row, "Set", self._on_language_set, width=6).pack(
            side="left", padx=6)
        self._make_btn(row, "Get", self._on_language_get, width=6).pack(
            side="left")

    # ---- Speech test ------------------------------------------------
    def _card_speech(self, parent):
        card = self._make_card(parent, "Speech Test")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="Say:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.speech_entry = tk.Entry(row, font=self.font_norm, width=22)
        self.speech_entry.insert(0, "Hello, I am NAO")
        self.speech_entry.pack(side="left", padx=4)
        self._make_btn(row, "Speak", self._on_speak, width=8).pack(
            side="left", padx=6)

    # ---- LEDs -------------------------------------------------------
    def _card_leds(self, parent):
        card = self._make_card(parent, "LEDs")
        row1 = tk.Frame(card, bg=CARD_BG)
        row1.pack(fill="x", pady=(0, 4))
        tk.Label(row1, text="Group:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.led_group_var = tk.StringVar(value="AllLeds")
        m = tk.OptionMenu(row1, self.led_group_var, *self.LED_GROUPS)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG,
                 activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)

        row2 = tk.Frame(card, bg=CARD_BG)
        row2.pack(fill="x")
        for name in sorted(self.LED_COLOURS.keys()):
            hexval = self.LED_COLOURS[name]
            btn_col = "#%06x" % (hexval & 0xFFFFFF) if hexval != 0 else "#333333"
            fg_col = "#000000" if name in ("White", "Yellow", "Cyan", "Green") else "#FFFFFF"
            tk.Button(
                row2, text=name, width=7, font=self.font_small,
                bg=btn_col, fg=fg_col, relief="flat",
                command=lambda c=hexval: self._on_led_colour(c)
            ).pack(side="left", padx=1, pady=2)

    # ---- Posture ----------------------------------------------------
    def _card_posture(self, parent):
        card = self._make_card(parent, "Posture")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self.posture_var = tk.StringVar(value="StandInit")
        tk.Label(row, text="Posture:", font=self.font_norm,
                 bg=CARD_BG, fg=FG).pack(side="left")
        m = tk.OptionMenu(row, self.posture_var, *self.POSTURES)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG,
                 activebackground=ACCENT, highlightthickness=0, width=12)
        m.pack(side="left", padx=4)
        self._make_btn(row, "Go", self._on_posture, width=6).pack(
            side="left", padx=6)

    # ---- Autonomous life --------------------------------------------
    def _card_life(self, parent):
        card = self._make_card(parent, "Autonomous Life")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self._make_btn(row, "Enable",  self._on_life_on,  width=10).pack(side="left", padx=4)
        self._make_btn(row, "Disable", self._on_life_off, width=10).pack(side="left", padx=4)
        self.life_status = tk.StringVar(value="")
        tk.Label(row, textvariable=self.life_status, font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left", padx=8)

    # ---- Battery ----------------------------------------------------
    def _card_battery(self, parent):
        card = self._make_card(parent, "Battery")
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self._make_btn(row, "Refresh", self._on_battery, width=10).pack(side="left", padx=4)
        self.battery_var = tk.StringVar(value="--")
        tk.Label(row, textvariable=self.battery_var, font=self.font_norm,
                 bg=CARD_BG, fg=SUCCESS).pack(side="left", padx=8)

    # =================================================================
    #  RIGHT column: PS5 Controller card  (tall, prominent)
    # =================================================================
    def _card_controller(self, parent):
        card = self._make_card(parent, "PS5 Controller")

        # -- Detect
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=(0, 6))
        self._make_btn(r1, "Detect", self._on_controller_detect, width=10).pack(
            side="left", padx=4)
        self.ctrl_indicator = tk.Label(r1, text="*", font=self.font_norm,
                                       bg=CARD_BG, fg=ERROR)
        self.ctrl_indicator.pack(side="left", padx=(4, 0))
        self.ctrl_status = tk.StringVar(value="No controller detected")
        tk.Label(r1, textvariable=self.ctrl_status, font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left", padx=4)

        # -- Start / Stop
        r2 = tk.Frame(card, bg=CARD_BG)
        r2.pack(fill="x", pady=(0, 6))
        self._make_btn(r2, "Start", self._on_controller_start, width=10).pack(
            side="left", padx=4)
        self._make_btn(r2, "Stop",  self._on_controller_stop,  width=10).pack(
            side="left", padx=4)

        # -- Deadzone slider
        r3 = tk.Frame(card, bg=CARD_BG)
        r3.pack(fill="x", pady=(0, 4))
        tk.Label(r3, text="Deadzone:", font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.deadzone_var = tk.DoubleVar(value=0.25)
        tk.Scale(r3, from_=0.10, to=0.50, resolution=0.05,
                 orient="horizontal", variable=self.deadzone_var,
                 length=180, bg=CARD_BG, fg=FG, troughcolor=BTN_BG,
                 highlightthickness=0,
                 font=self.font_small).pack(side="left", padx=4)

        # -- Speed slider  (capped low for safety)
        r4 = tk.Frame(card, bg=CARD_BG)
        r4.pack(fill="x", pady=(0, 4))
        tk.Label(r4, text="Speed:", font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.speed_var = tk.DoubleVar(value=0.25)
        tk.Scale(r4, from_=0.10, to=0.40, resolution=0.05,
                 orient="horizontal", variable=self.speed_var,
                 length=180, bg=CARD_BG, fg=FG, troughcolor=BTN_BG,
                 highlightthickness=0,
                 font=self.font_small).pack(side="left", padx=4)

        # -- Live axis readout
        r5 = tk.Frame(card, bg=CARD_BG)
        r5.pack(fill="x", pady=(0, 4))
        self.ctrl_axes_var = tk.StringVar(value="Fwd: 0.00   Rot: 0.00")
        tk.Label(r5, textvariable=self.ctrl_axes_var, font=self.font_guide,
                 bg=CARD_BG, fg=ACCENT).pack(side="left", padx=4)

        # -- Gyro / tilt readout
        r6 = tk.Frame(card, bg=CARD_BG)
        r6.pack(fill="x", pady=(0, 6))
        self.gyro_var = tk.StringVar(value="Tilt X: --   Y: --   Stability: --")
        self.gyro_lbl = tk.Label(r6, textvariable=self.gyro_var,
                                 font=self.font_guide, bg=CARD_BG, fg=FG)
        self.gyro_lbl.pack(side="left", padx=4)

        # -- Button guide  (plain ASCII - no Unicode escapes)
        guide = self._make_card(card, "Button Guide")
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
            tk.Label(row, text=lbl,  font=self.font_guide, bg=CARD_BG,
                     fg=ACCENT, width=18, anchor="w").pack(side="left")
            tk.Label(row, text=desc, font=self.font_guide, bg=CARD_BG,
                     fg=FG, anchor="w").pack(side="left")

        # ---- Internal controller state ----
        self._joystick = None
        self._ctrl_running = False
        self._ctrl_thread  = None

        # Safety: hard velocity clamps (absolute maximums)
        self._MAX_FORWARD = 0.35
        self._MAX_ROTATE  = 0.25

        # Smoothing factor  (low = smoother, less jerky)
        # 0.08 means only 8% of new target blended per tick
        self._SMOOTHING = 0.08

        # Gyro tilt thresholds (radians)
        self._TILT_WARN   = 0.15   # ~8.6 deg  -> begin reducing speed
        self._TILT_DANGER = 0.25   # ~14.3 deg -> emergency stop

        # Current smoothed outputs
        self._cur_fwd = 0.0
        self._cur_rot = 0.0

    # =================================================================
    #  Status helpers
    # =================================================================
    def _set_status(self, msg, ok=True):
        self.status_var.set(msg)
        self.status_lbl.config(fg=SUCCESS if ok else ERROR)

    def _require_connection(self):
        if not self.connected:
            self._set_status("Not connected - click Connect first", False)
            return False
        return True

    # =================================================================
    #  Connection
    # =================================================================
    def _on_connect(self):
        self.ip   = self.ip_entry.get().strip()
        self.port = int(self.port_entry.get().strip())
        self._set_status("Connecting to %s:%d ..." % (self.ip, self.port), True)
        self.root.update_idletasks()

        # TCP check
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        try:
            s.connect((self.ip, self.port))
            s.close()
        except Exception:
            self._set_status(
                "TCP port %d unreachable on %s" % (self.port, self.ip), False)
            self.connected = False
            return

        # Build proxies
        self.system = _make_proxy("ALSystem", self.ip, self.port)
        if self.system is None:
            self._set_status("TCP OK but ALProxy failed", False)
            self.connected = False
            return

        self.tts     = _make_proxy("ALTextToSpeech",  self.ip, self.port)
        self.leds    = _make_proxy("ALLeds",           self.ip, self.port)
        self.motion  = _make_proxy("ALMotion",         self.ip, self.port)
        self.posture = _make_proxy("ALRobotPosture",   self.ip, self.port)
        self.memory  = _make_proxy("ALMemory",         self.ip, self.port)
        self.life    = _make_proxy("ALAutonomousLife",  self.ip, self.port)
        self.audio   = _make_proxy("ALAudioDevice",    self.ip, self.port)
        self.connected = True

        # Sync sliders with robot state
        try:
            self.vol_var.set(self.audio.getOutputVolume())
        except Exception:
            pass
        try:
            self.lang_var.set(self.tts.getLanguage())
        except Exception:
            pass
        try:
            self.tts.say("Connected")
        except Exception:
            pass

        ver = "?"
        try:
            ver = self.system.systemVersion()
        except Exception:
            pass
        self._set_status("Connected to %s (%s)" % (self.ip, ver), True)

    # =================================================================
    #  Volume
    # =================================================================
    def _on_volume_set(self):
        if not self._require_connection():
            return
        vol = self.vol_var.get()
        try:
            self.audio.setOutputVolume(vol)
            self._set_status("Volume set to %d%%" % vol)
        except Exception as e:
            self._set_status("Volume error: %s" % e, False)

    # =================================================================
    #  Language
    # =================================================================
    def _on_language_set(self):
        if not self._require_connection():
            return
        lang = self.lang_var.get()
        try:
            self.tts.setLanguage(lang)
            self._set_status("Language -> %s" % lang)
        except Exception as e:
            self._set_status("Language error: %s" % e, False)

    def _on_language_get(self):
        if not self._require_connection():
            return
        try:
            lang = self.tts.getLanguage()
            self.lang_var.set(lang)
            self._set_status("Current language: %s" % lang)
        except Exception as e:
            self._set_status("Language error: %s" % e, False)

    # =================================================================
    #  Speech
    # =================================================================
    def _on_speak(self):
        if not self._require_connection():
            return
        text = self.speech_entry.get().strip()
        if not text:
            return
        def _say():
            try:
                self.tts.say(str(text))
            except Exception:
                pass
        threading.Thread(target=_say).start()
        self._set_status("Speaking: \"%s\"" % text)

    # =================================================================
    #  LEDs
    # =================================================================
    def _on_led_colour(self, colour_hex):
        if not self._require_connection():
            return
        group = self.led_group_var.get()
        try:
            self.leds.fadeRGB(group, colour_hex, 0.3)
            self._set_status("LEDs %s -> #%06x" % (group, colour_hex))
        except Exception as e:
            self._set_status("LED error: %s" % e, False)

    # =================================================================
    #  Posture
    # =================================================================
    def _on_posture(self):
        if not self._require_connection():
            return
        name = self.posture_var.get()
        self._set_status("Going to posture: %s ..." % name)
        self.root.update_idletasks()
        def _go():
            try:
                self.posture.goToPosture(name, 0.8)
                self._set_status("Posture: %s" % name)
            except Exception as e:
                self._set_status("Posture error: %s" % e, False)
        threading.Thread(target=_go).start()

    # =================================================================
    #  Autonomous life
    # =================================================================
    def _on_life_on(self):
        if not self._require_connection():
            return
        try:
            self.life.setState("solitary")
            self.life_status.set("ON")
            self._set_status("Autonomous life enabled")
        except Exception as e:
            self._set_status("Life error: %s" % e, False)

    def _on_life_off(self):
        if not self._require_connection():
            return
        try:
            self.life.setState("disabled")
            self.life_status.set("OFF")
            self._set_status("Autonomous life disabled")
        except Exception as e:
            self._set_status("Life error: %s" % e, False)

    # =================================================================
    #  Battery
    # =================================================================
    def _on_battery(self):
        if not self._require_connection():
            return
        try:
            level = self.memory.getData(
                "Device/SubDeviceList/Battery/Charge/Sensor/Value")
            pct = int(level * 100)
            self.battery_var.set("%d%%" % pct)
            self._set_status("Battery: %d%%" % pct)
        except Exception as e:
            self._set_status("Battery error: %s" % e, False)

    # =================================================================
    #  PS5 Controller
    # =================================================================
    def _on_controller_detect(self):
        if not HAS_PYGAME:
            self.ctrl_status.set("pygame not installed")
            self.ctrl_indicator.config(fg=ERROR)
            self._set_status("pygame not available", False)
            return
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.ctrl_status.set("No controller found")
            self.ctrl_indicator.config(fg=ERROR)
            self._set_status("No controller detected", False)
            self._joystick = None
            return
        js = pygame.joystick.Joystick(0)
        js.init()
        self._joystick = js
        name = js.get_name()
        axes = js.get_numaxes()
        self.ctrl_status.set("%s (%d axes)" % (name, axes))
        self.ctrl_indicator.config(fg=SUCCESS)
        self._set_status("Controller: %s" % name)

    def _on_controller_start(self):
        if self._joystick is None:
            self._set_status("Detect a controller first", False)
            return
        if not self._require_connection():
            return
        if self._ctrl_running:
            return
        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(target=self._controller_loop)
        self._ctrl_thread.daemon = True
        self._ctrl_thread.start()
        self._set_status("Controller active")

    def _on_controller_stop(self):
        self._ctrl_running = False
        self._cur_fwd = 0.0
        self._cur_rot = 0.0
        if self.motion:
            try:
                self.motion.moveToward(0, 0, 0)
            except Exception:
                pass
        self.ctrl_axes_var.set("Fwd: 0.00   Rot: 0.00")
        self.gyro_var.set("Tilt X: --   Y: --   Stability: --")
        self.gyro_lbl.config(fg=FG)
        self._set_status("Controller stopped")

    # -----------------------------------------------------------------
    #  Gyro helpers
    # -----------------------------------------------------------------
    def _read_tilt(self):
        """Read body tilt from NAO inertial sensor (radians).
        Returns (angleX, angleY) or (0, 0) on failure.
          angleX = pitch (forward / back tilt)
          angleY = roll  (side-to-side tilt)
        """
        try:
            ax = self.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
            ay = self.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
            return float(ax), float(ay)
        except Exception:
            return 0.0, 0.0

    def _stability_factor(self, tilt_x, tilt_y):
        """Return 0.0-1.0 multiplier based on how tilted NAO is.
        1.0 = upright, 0.0 = dangerously tilted.
        """
        tilt_mag = math.sqrt(tilt_x ** 2 + tilt_y ** 2)
        if tilt_mag >= self._TILT_DANGER:
            return 0.0
        if tilt_mag <= self._TILT_WARN:
            return 1.0
        # Linear ramp between warn and danger
        ratio = (tilt_mag - self._TILT_WARN) / (self._TILT_DANGER - self._TILT_WARN)
        return 1.0 - ratio

    # -----------------------------------------------------------------
    #  Controller loop  (runs in its own thread at ~20 Hz)
    # -----------------------------------------------------------------
    def _is_robot_standing(self):
        """Return True if NAO is in a standing posture (safe to walk)."""
        try:
            p = self.posture.getPostureFamily()
            return p in ("Standing", "Standing", "StandInit", "Stand", "StandZero")
        except Exception:
            return False

    def _controller_loop(self):
        js = self._joystick

        while self._ctrl_running:
            try:
                pygame.event.pump()

                # ---- Button actions (ALWAYS processed, regardless of tilt) ----
                try:
                    if js.get_button(0):       # Cross  -> StandInit
                        if self.posture:
                            self._set_status("StandInit via controller...")
                            self.posture.goToPosture("StandInit", 0.8)
                            self._set_status("StandInit complete")

                    if js.get_button(1):       # Circle -> Sit
                        if self.posture:
                            self._set_status("Sitting via controller...")
                            self.posture.goToPosture("Sit", 0.8)
                            self._set_status("Sit complete")

                    if js.get_button(2):       # Triangle -> Relax (servos off)
                        if self.motion:
                            self._set_status("Relaxing servos...")
                            self.motion.rest()
                            self._set_status("Servos relaxed")
                        self._ctrl_running = False
                        break

                    if js.get_button(3):       # Square -> Stop controller
                        self._ctrl_running = False
                        if self.motion:
                            self.motion.moveToward(0, 0, 0)
                        self._set_status("Controller stopped")
                        break
                except Exception:
                    pass

                # ---- Read sticks ----
                # Left stick Y  = forward / back ONLY  (no strafe)
                raw_fwd = -js.get_axis(1)      # inverted: push up = positive
                # Right stick X = rotation ONLY
                raw_rot =  js.get_axis(2)       # axis 2 = right stick horizontal

                dz    = self.deadzone_var.get()
                speed = self.speed_var.get()

                fwd_in = raw_fwd * speed if abs(raw_fwd) > dz else 0.0
                rot_in = raw_rot * speed if abs(raw_rot) > dz else 0.0

                # ---- Gyro feedback (only active while standing) ----
                tilt_x, tilt_y = self._read_tilt()
                standing = self._is_robot_standing()

                if standing:
                    stab = self._stability_factor(tilt_x, tilt_y)
                else:
                    # Not standing: don't gate anything, just show tilt
                    stab = 1.0

                # Scale targets by stability factor
                tgt_fwd = _clamp(fwd_in * stab, -self._MAX_FORWARD, self._MAX_FORWARD)
                tgt_rot = _clamp(rot_in * stab, -self._MAX_ROTATE,  self._MAX_ROTATE)

                # Emergency stop walk when dangerously tilted AND standing
                if stab == 0.0 and standing:
                    tgt_fwd = 0.0
                    tgt_rot = 0.0

                # ---- Smooth (lerp) ----
                self._cur_fwd = _lerp(self._cur_fwd, tgt_fwd, self._SMOOTHING)
                self._cur_rot = _lerp(self._cur_rot, tgt_rot, self._SMOOTHING)

                # Snap near-zero to zero to avoid micro-drift
                if abs(self._cur_fwd) < 0.005:
                    self._cur_fwd = 0.0
                if abs(self._cur_rot) < 0.005:
                    self._cur_rot = 0.0

                # ---- Update UI labels ----
                self.ctrl_axes_var.set(
                    "Fwd: %.2f   Rot: %.2f" % (self._cur_fwd, self._cur_rot))

                if not standing:
                    stab_txt = "N/A (not standing)"
                    stab_col = FG
                elif stab >= 0.8:
                    stab_txt = "OK"
                    stab_col = SUCCESS
                elif stab > 0.0:
                    stab_txt = "CAUTION (%d%%)" % int(stab * 100)
                    stab_col = WARN
                else:
                    stab_txt = "DANGER - walk stopped"
                    stab_col = ERROR

                self.gyro_var.set(
                    "Tilt X:%+.2f  Y:%+.2f   Stability: %s"
                    % (tilt_x, tilt_y, stab_txt))
                self.gyro_lbl.config(fg=stab_col)

                # ---- Drive NAO  (only send walk commands when standing) ----
                if self.motion and standing:
                    self.motion.moveToward(
                        float(self._cur_fwd),
                        0.0,                   # NO strafe ever
                        float(self._cur_rot))

                time.sleep(0.05)   # ~20 Hz
            except Exception:
                time.sleep(0.1)

        # Clean up readout when loop exits
        self.ctrl_axes_var.set("Fwd: 0.00   Rot: 0.00")

    # =================================================================
    #  Run
    # =================================================================
    def run(self):
        self.root.mainloop()


# =====================================================================
if __name__ == "__main__":
    app = NaoSettingsApp()
    app.run()
