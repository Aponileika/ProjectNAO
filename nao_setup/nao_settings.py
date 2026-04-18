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
        self.video = None
        self.face = None
        self.people = None

        # Camera state
        self._cam_name = None
        self._cam_running = False
        self._cam_after_id = None
        self._cam_photo = None
        self._human_hold_until = 0.0
        self._alarm_active = False

        # Controller panel visibility / PS button toggle
        self._controller_expanded = False
        self._ps_prev_down = False
        self._PS_BUTTON_CANDIDATES = (10, 12, 16)

        self._build_ui()

    # =================================================================
    #  Build the entire UI
    # =================================================================
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("NAO Control Panel")
        self.root.configure(bg=BG)
        self.root.state("zoomed")  # start maximised
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

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
        self._card_camera(right)

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
        self.ctrl_toggle_btn = self._make_btn(
            r2, "Show", self._toggle_controller_details, width=10)
        self.ctrl_toggle_btn.pack(side="left", padx=4)

        # Advanced controller settings (collapsed by default)
        self.ctrl_advanced = tk.Frame(card, bg=CARD_BG)

        # -- Deadzone slider
        r3 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
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
        r4 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r4.pack(fill="x", pady=(0, 4))
        tk.Label(r4, text="Speed:", font=self.font_small,
                 bg=CARD_BG, fg=FG).pack(side="left")
        self.speed_var = tk.DoubleVar(value=0.30)
        tk.Scale(r4, from_=0.10, to=0.45, resolution=0.05,
                 orient="horizontal", variable=self.speed_var,
                 length=180, bg=CARD_BG, fg=FG, troughcolor=BTN_BG,
                 highlightthickness=0,
                 font=self.font_small).pack(side="left", padx=4)

        # -- Live axis readout
        r5 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r5.pack(fill="x", pady=(0, 4))
        self.ctrl_axes_var = tk.StringVar(value="Fwd: 0.00   Rot: 0.00")
        tk.Label(r5, textvariable=self.ctrl_axes_var, font=self.font_guide,
                 bg=CARD_BG, fg=ACCENT).pack(side="left", padx=4)

        # -- Gyro / tilt readout
        r6 = tk.Frame(self.ctrl_advanced, bg=CARD_BG)
        r6.pack(fill="x", pady=(0, 6))
        self.gyro_var = tk.StringVar(value="Tilt X: --   Y: --   Stability: --")
        self.gyro_lbl = tk.Label(r6, textvariable=self.gyro_var,
                                 font=self.font_guide, bg=CARD_BG, fg=FG)
        self.gyro_lbl.pack(side="left", padx=4)

        # -- Button guide  (plain ASCII - no Unicode escapes)
        guide = self._make_card(self.ctrl_advanced, "Button Guide")
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
        self._MAX_FORWARD  = 0.34
        self._MAX_BACKWARD = 0.24
        self._MAX_ROTATE   = 0.16

        # Smoothing factor  (low = smoother, less jerky)
        # 0.10 means 10% of new target blended per tick
        self._SMOOTHING = 0.10

        # Reduce turning authority while moving forward to avoid yaw wobble
        self._ROTATE_WHILE_FORWARD_FACTOR = 0.50

        # Conservative NAO gait config for better straight-line stability
        self._WALK_CONFIG = [
            ["Frequency", 0.90],
            ["MaxStepX", 0.055],
            ["MaxStepY", 0.010],
            ["MaxStepTheta", 0.18],
            ["StepHeight", 0.011],
            ["TorsoWy", 0.01],
        ]

        # Gyro tilt thresholds (radians)
        self._TILT_WARN   = 0.20   # ~11.5 deg -> begin reducing speed
        self._TILT_DANGER = 0.32   # ~18.3 deg -> emergency stop

        # Current smoothed outputs
        self._cur_fwd = 0.0
        self._cur_rot = 0.0
        self._btn_busy = False  # prevents overlapping button actions

        self._set_controller_details(False)

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

    # =================================================================
    #  RIGHT column: Camera card
    # =================================================================
    def _card_camera(self, parent):
        card = self._make_card(parent, "Camera")

        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x", pady=(0, 6))
        self._make_btn(row, "Start", self._on_camera_start, width=10).pack(
            side="left", padx=4)
        self._make_btn(row, "Stop", self._on_camera_stop, width=10).pack(
            side="left", padx=4)
        self.cam_status_var = tk.StringVar(value="Stopped")
        tk.Label(row, textvariable=self.cam_status_var,
                 font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left", padx=6)

        row2 = tk.Frame(card, bg=CARD_BG)
        row2.pack(fill="x", pady=(0, 4))
        self.cam_detect_var = tk.StringVar(value="Human presence: --")
        self.cam_detect_lbl = tk.Label(row2, textvariable=self.cam_detect_var,
                           font=self.font_small, bg=CARD_BG, fg=FG)
        self.cam_detect_lbl.pack(side="left", padx=4)

        self.cam_preview_lbl = tk.Label(
            card,
            text="No feed",
            bg="#000000",
            fg=FG,
            anchor="center")
        self.cam_preview_lbl.pack(fill="x", padx=4, pady=(0, 2))

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
        self.video   = _make_proxy("ALVideoDevice",    self.ip, self.port)
        self.face    = _make_proxy("ALFaceDetection",  self.ip, self.port)
        self.people  = _make_proxy("ALPeoplePerception", self.ip, self.port)
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
    #  Camera
    # =================================================================
    def _rgb_to_photo(self, width, height, payload):
        # Use strict binary PPM (P6) payload expected by Tk PhotoImage.
        header = "P6\n%d %d\n255\n" % (int(width), int(height))
        ppm_data = header + payload
        photo = tk.PhotoImage(data=ppm_data, format="PPM")

        # Scale up small camera resolutions for better visibility.
        target_w = 480
        scale = max(1, int(target_w / max(1, int(width))))
        if scale > 1:
            photo = photo.zoom(scale, scale)
        return photo

    def _on_camera_start(self):
        if not self._require_connection():
            return
        if self._cam_running:
            return

        # Keep camera area clear unless controller details are explicitly needed
        self._set_controller_details(False)

        if self.video is None:
            self.video = _make_proxy("ALVideoDevice", self.ip, self.port)
            if self.video is None:
                self._set_status("Camera proxy unavailable", False)
                self.cam_status_var.set("Unavailable")
                return

        try:
            # kQVGA=1 (320x240), kRGBColorSpace=11
            self._cam_name = self.video.subscribe("nao_settings_cam", 1, 11, 5)
        except Exception as e:
            self._set_status("Camera start error: %s" % e, False)
            self.cam_status_var.set("Start failed")
            return

        if self.face is None:
            self.face = _make_proxy("ALFaceDetection", self.ip, self.port)
        if self.face is not None:
            try:
                self.face.subscribe("nao_settings_face", 500, 0.0)
            except Exception:
                pass
            try:
                self.face.setTrackingEnabled(True)
            except Exception:
                pass

        if self.people is None:
            self.people = _make_proxy("ALPeoplePerception", self.ip, self.port)
        if self.people is not None:
            try:
                # Better distance robustness over raw speed.
                self.people.setFastModeEnabled(False)
            except Exception:
                pass
            try:
                # Extend detection range when supported by robot firmware.
                self.people.setMaximumDetectionRange(5.0)
            except Exception:
                pass
            try:
                self.people.subscribe("nao_settings_people")
            except Exception:
                pass

        # Slightly raise gaze to improve medium/far human visibility.
        if self.motion is not None:
            try:
                self.motion.setAngles("HeadPitch", -0.12, 0.08)
            except Exception:
                pass

        if self.leds is not None:
            try:
                # Default "all clear" blue
                self.leds.fadeRGB("AllLeds", 0x0000FF, 1.0)
            except Exception:
                pass

        self._cam_running = True
        self.cam_status_var.set("Starting...")
        self.cam_detect_var.set("Human presence: scanning...")
        self.cam_detect_lbl.config(fg=FG)
        self._camera_tick()

    def _on_camera_stop(self):
        self._cam_running = False
        if self._cam_after_id is not None:
            try:
                self.root.after_cancel(self._cam_after_id)
            except Exception:
                pass
            self._cam_after_id = None

        if self.video and self._cam_name:
            try:
                self.video.unsubscribe(self._cam_name)
            except Exception:
                pass
        if self.face:
            try:
                self.face.unsubscribe("nao_settings_face")
            except Exception:
                pass
        if self.people:
            try:
                self.people.unsubscribe("nao_settings_people")
            except Exception:
                pass
        self._human_hold_until = 0.0
        
        # Reset alarm state and turn off blue if camera stops
        if self._alarm_active:
            self._alarm_active = False
            if self.tts:
                try:
                    self.tts.stopAll()
                except Exception:
                    pass
        if self.leds:
            try:
                self.leds.fadeRGB("AllLeds", 0x000000, 1.0)
            except Exception:
                pass

        self._cam_name = None
        self._cam_photo = None
        self.cam_preview_lbl.config(image="", text="No feed")
        self.cam_status_var.set("Stopped")
        self.cam_detect_var.set("Human presence: --")
        self.cam_detect_lbl.config(fg=FG)

    def _collect_probabilities(self, value, out_list):
        if isinstance(value, (int, float)):
            v = float(value)
            # Accept both [0..1] and [0..100] confidence representations.
            if 0.0 <= v <= 1.0:
                out_list.append(v)
            elif 1.0 < v <= 100.0:
                out_list.append(v / 100.0)
            return
        if isinstance(value, list):
            for item in value:
                self._collect_probabilities(item, out_list)

    def _face_detection_state(self):
        if not self.memory:
            return False, None
        try:
            data = self.memory.getData("FaceDetected")
        except Exception:
            return False, None
        if not data:
            return False, None

        detected = False

        scores = []

        # Typical structure is [TimeStamp, FaceInfoArray, ...]
        try:
            if isinstance(data, list) and len(data) > 1 and isinstance(data[1], list):
                if len(data[1]) > 0:
                    detected = True
                for face_info in data[1]:
                    if isinstance(face_info, list) and len(face_info) > 1:
                        extra = face_info[1]
                        self._collect_probabilities(extra, scores)
        except Exception:
            pass

        if not scores:
            self._collect_probabilities(data, scores)

        if not scores:
            return detected, None
        return True, max(scores)

    def _people_detected(self):
        if not self.memory:
            return False, 0
        try:
            people = self.memory.getData("PeoplePerception/PeopleList")
        except Exception:
            return False, 0
        if isinstance(people, list) and len(people) > 0:
            return True, len(people)
        return False, 0

    def _alarm_loop(self):
        while self._alarm_active:
            if self.audio:
                try:
                    self.audio.setOutputVolume(100)
                except Exception:
                    pass
            if self.tts:
                try:
                    self.tts.say("destroy")
                except Exception:
                    pass
            # Avoid tight spin loop
            time.sleep(0.1)

    def _start_alarm(self):
        self._alarm_active = True
        if self.leds:
            try:
                self.leds.fadeRGB("AllLeds", 0xFF0000, 0.2) # Red
            except Exception:
                pass
        t = threading.Thread(target=self._alarm_loop)
        t.daemon = True
        t.start()

    def _stop_alarm(self):
        self._alarm_active = False
        if self.tts:
            try:
                self.tts.stopAll()
            except Exception:
                pass
        if self.leds:
            try:
                self.leds.fadeRGB("AllLeds", 0x0000FF, 0.5) # Blue
            except Exception:
                pass
        
        def _diffuse():
            time.sleep(0.5) # Wait for alarm thread to finish
            if self.tts:
                try:
                    self.tts.say("situation diffused")
                except Exception:
                    pass

        t = threading.Thread(target=_diffuse)
        t.daemon = True
        t.start()

    def _update_human_detection_label(self):
        face_detected, conf = self._face_detection_state()
        people_detected, people_count = self._people_detected()
        now = time.time()

        # Update alarm state. Hold applies if detected recently.
        human_present = bool(people_detected or face_detected or now < self._human_hold_until)
        if human_present and not self._alarm_active:
            self._start_alarm()
        elif not human_present and self._alarm_active:
            self._stop_alarm()

        # Priority: body-level human presence first, face confidence second.
        if people_detected:
            self._human_hold_until = now + 1.2
            if people_count > 1:
                msg = "Human presence: yes (%d people)" % people_count
            else:
                msg = "Human presence: yes (1 person)"

            if conf is not None:
                if conf >= 0.80:
                    band = "high"
                elif conf >= 0.60:
                    band = "medium"
                else:
                    band = "low"
                msg += ", face %d%% (%s)" % (int(conf * 100.0), band)

            self.cam_detect_var.set(msg)
            self.cam_detect_lbl.config(fg=SUCCESS)
            return

        # Fallback when body detector misses but face detector triggers
        if face_detected:
            self._human_hold_until = now + 1.2
            if conf is None:
                self.cam_detect_var.set("Human presence: likely (face)")
                self.cam_detect_lbl.config(fg=WARN)
                return

            if conf >= 0.80:
                band = "high"
                col = SUCCESS
            elif conf >= 0.60:
                band = "medium"
                col = WARN
            else:
                band = "low"
                col = WARN

            self.cam_detect_var.set("Human presence: likely, face %d%% (%s)" % (
                int(conf * 100.0), band))
            self.cam_detect_lbl.config(fg=col)
            return

        if now < self._human_hold_until:
            self.cam_detect_var.set("Human presence: yes (recent)")
            self.cam_detect_lbl.config(fg=SUCCESS)
            return

        if not face_detected and not people_detected:
            self.cam_detect_var.set("Human presence: no")
            self.cam_detect_lbl.config(fg=ERROR)
            return

    def _camera_tick(self):
        if not self._cam_running or not self.video or not self._cam_name:
            return

        try:
            img = self.video.getImageRemote(self._cam_name)
            if img and len(img) >= 7:
                width = int(img[0])
                height = int(img[1])
                payload = img[6]
                photo = self._rgb_to_photo(width, height, payload)
                self._cam_photo = photo
                self.cam_preview_lbl.config(image=photo, text="")
                self.cam_status_var.set("Live %dx%d" % (width, height))
            else:
                self.cam_status_var.set("No frame")
        except Exception as e:
            self.cam_status_var.set("Frame error")
            self._set_status("Camera frame error: %s" % e, False)

        self._update_human_detection_label()

        if self._cam_running:
            self._cam_after_id = self.root.after(150, self._camera_tick)

    # =================================================================
    #  PS5 Controller
    # =================================================================
    def _on_controller_detect(self):
        if not HAS_PYGAME:
            self.ctrl_status.set("pygame not installed")
            self.ctrl_indicator.config(fg=ERROR)
            self._set_status("pygame not available", False)
            return
        # Full quit and reinit so pygame rescans for newly connected controllers
        pygame.quit()
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

        try:
            self.motion.wakeUp()
            self.motion.setStiffnesses("Body", 1.0)
            self.motion.setFallManagerEnabled(True)
            self.motion.setMoveArmsEnabled(True, True)
            self.motion.setMotionConfig([
                ["ENABLE_FOOT_CONTACT_PROTECTION", True],
            ])
        except Exception:
            pass

        self._ps_prev_down = False

        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(target=self._controller_loop)
        self._ctrl_thread.daemon = True
        self._ctrl_thread.start()
        self._set_status("Controller active - press Cross to stand")

    def _on_controller_stop(self):
        self._ctrl_running = False
        self._ps_prev_down = False
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

    def _stand_safely(self):
        if not self.motion or not self.posture:
            return False

        try:
            self.motion.wakeUp()
        except Exception:
            pass
        try:
            self.motion.setStiffnesses("Body", 1.0)
        except Exception:
            pass
        try:
            self.motion.setFallManagerEnabled(True)
        except Exception:
            pass
        try:
            self.motion.setMoveArmsEnabled(True, True)
        except Exception:
            pass

        poses = ("StandInit", "Stand")
        for _ in range(2):
            for pose in poses:
                try:
                    ok = self.posture.goToPosture(pose, 1.0)
                except Exception:
                    ok = False
                time.sleep(0.5)
                if ok and self._is_robot_standing():
                    return True
        return False

    def _is_ps_button_down(self, js):
        try:
            n = js.get_numbuttons()
            for idx in self._PS_BUTTON_CANDIDATES:
                if idx < n and js.get_button(idx):
                    return True
        except Exception:
            pass
        return False

    def _controller_loop(self):
        js = self._joystick
        print("[DEBUG] Controller loop started, buttons=%d axes=%d" % (
            js.get_numbuttons(), js.get_numaxes()))

        while self._ctrl_running:
            try:
                pygame.event.pump()

                ps_down = self._is_ps_button_down(js)
                if ps_down and not self._ps_prev_down:
                    self.root.after(0, self._toggle_controller_details)
                self._ps_prev_down = ps_down

                # ---- Show which buttons are pressed (debug) ----
                pressed = []
                for bi in range(js.get_numbuttons()):
                    if js.get_button(bi):
                        pressed.append(str(bi))
                if pressed:
                    print("[DEBUG] Buttons: %s" % ", ".join(pressed))
                    self._set_status("Buttons pressed: %s" % ", ".join(pressed))

                # ---- Button actions (run in threads so they don't block the loop) ----
                try:
                    if js.get_button(0) and not self._btn_busy:  # Cross -> StandInit
                        if self.posture:
                            self._btn_busy = True
                            def _do_stand():
                                try:
                                    self._set_status("Standing up safely...")
                                    if self._stand_safely():
                                        self._set_status("Stand complete")
                                    else:
                                        self._set_status("Stand failed - hold robot and retry", False)
                                except Exception as e:
                                    print("[DEBUG] StandInit error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_stand).start()

                    if js.get_button(1) and not self._btn_busy:  # Circle -> Sit
                        if self.posture:
                            self._btn_busy = True
                            def _do_sit():
                                try:
                                    self._set_status("Sitting via controller...")
                                    self.posture.goToPosture("Sit", 0.8)
                                    self._set_status("Sit complete")
                                except Exception as e:
                                    print("[DEBUG] Sit error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_sit).start()

                    if js.get_button(2) and not self._btn_busy:  # Triangle -> Relax (servos off)
                        if self.motion:
                            self._btn_busy = True
                            self._set_status("Relaxing servos...")
                            def _do_relax():
                                try:
                                    self.motion.rest()
                                    self._set_status("Servos relaxed (press Cross to stand)")
                                except Exception as e:
                                    print("[DEBUG] Relax error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_relax).start()

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
                raw_fwd = js.get_axis(1)
                # Right stick X = rotation ONLY
                raw_rot = js.get_axis(2)

                dz    = self.deadzone_var.get()
                speed = self.speed_var.get()

                fwd_in = raw_fwd * speed if abs(raw_fwd) > dz else 0.0
                rot_in = raw_rot * speed if abs(raw_rot) > dz else 0.0

                # Heavily damp turning while actively moving forward
                if abs(fwd_in) > 0.10:
                    rot_in *= self._ROTATE_WHILE_FORWARD_FACTOR

                # Extra tiny deadband on rotation to reduce oscillation
                if abs(rot_in) < 0.04:
                    rot_in = 0.0

                # ---- Gyro feedback (only active while standing) ----
                tilt_x, tilt_y = self._read_tilt()
                standing = self._is_robot_standing()

                if standing:
                    stab = self._stability_factor(tilt_x, tilt_y)
                else:
                    # Not standing: don't gate anything, just show tilt
                    stab = 1.0

                # Scale targets by stability factor (asymmetric for safer reverse)
                fwd_target = fwd_in * stab
                if fwd_target >= 0.0:
                    tgt_fwd = _clamp(fwd_target, -self._MAX_FORWARD, self._MAX_FORWARD)
                else:
                    tgt_fwd = _clamp(fwd_target, -self._MAX_BACKWARD, self._MAX_BACKWARD)
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
                        float(self._cur_rot),
                        self._WALK_CONFIG)

                time.sleep(0.05)   # ~20 Hz
            except Exception as e:
                print("[DEBUG] Controller loop error: %s" % e)
                time.sleep(0.1)

        print("[DEBUG] Controller loop ended")
        # Clean up readout when loop exits
        self.ctrl_axes_var.set("Fwd: 0.00   Rot: 0.00")

    # =================================================================
    #  Run
    # =================================================================
    def _on_close(self):
        try:
            self._on_camera_stop()
        except Exception:
            pass
        try:
            if self._ctrl_running:
                self._on_controller_stop()
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# =====================================================================
if __name__ == "__main__":
    app = NaoSettingsApp()
    app.run()
