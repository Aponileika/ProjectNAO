# -*- coding: utf-8 -*-
import Tkinter as tk
import tkFont
import threading
import math
from nao_app.ui.widgets import make_card, make_btn, BG, FG, ACCENT, SUCCESS, ERROR, WARN, CARD_BG, BTN_BG, BTN_FG
from nao_app.backend import persona

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

        self.config_ip = ""
        self.config_port = ""
        # Hard motion lock.  Set by Relax / emergency stop; every autonomous
        # loop must check it and issue no further motion commands until an
        # explicit user action clears it.
        self._motion_locked = False
        # Burst-gait timings (config.json can override).  Measured on the bad
        # robot, roll is already ~0.16 by t=2s, so a burst has to stay well
        # under that or it ends at the warning threshold every single time.
        self.config_burst_walk_s   = 1.0
        self.config_burst_settle_s = 1.2
        self.config_resume_roll    = 0.06
        # Arm swing during walking.  Normally on — swinging arms genuinely help
        # NAO balance.  Exposed only so arm motion can be removed as a variable
        # when chasing a gait problem.
        self.config_walk_arms      = True
        # Burst gait walks in short bursts with a settle pause between them.
        # It exists to compensate for one specific robot whose roll diverges
        # until it topples (see BodyId ALDR1312N090344); a healthy robot does
        # not need it and walks better without the pauses.  Off unless
        # config.json opts in.
        self.config_burst_gait = False

        # Character and voice.  The picker keeps spoken lines from repeating;
        # the style wraps every utterance in Acapela markup so the robot has
        # one consistent voice wherever the words came from.
        self.persona = persona.PhrasePicker()
        self.voice   = persona.VoiceStyle()

        self.api_key_autofill = ""   # first key (shown in UI entry)
        self.api_keys_list    = []   # all keys, used for rotation
        try:
            import json, os
            # App window is in NAO/nao_setup/nao_app/ui/app_window.py (4 levels deep)
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            conf_path    = os.path.join(base_dir, "config.json")
            secrets_path = os.path.join(base_dir, "secrets.json")
            self.config_path = conf_path

            if os.path.isfile(conf_path):
                with open(conf_path, "r") as f:
                    cdata = json.load(f)
                    k = cdata.get("gemini_key", "")
                    if k and k not in self.api_keys_list:
                        self.api_keys_list.append(k)
                    ip = cdata.get("ip", "")
                    port = cdata.get("port", "")
                    if ip:
                        self.config_ip = ip
                    if port:
                        try:
                            self.config_port = str(int(port))
                        except Exception:
                            self.config_port = str(port)
                    self.config_burst_gait = bool(cdata.get("burst_gait", False))
                    # Burst timings, tunable without editing code.
                    try:
                        self.config_burst_walk_s = float(
                            cdata.get("burst_walk_s", 1.0))
                        self.config_burst_settle_s = float(
                            cdata.get("burst_settle_s", 1.2))
                        self.config_resume_roll = float(
                            cdata.get("burst_resume_roll", 0.06))
                    except Exception:
                        pass
                    self.config_walk_arms = bool(cdata.get("walk_arms", True))
                    try:
                        self.config_step_m = float(cdata.get("step_m", 0.12))
                        self.config_torso_wy = max(-0.122, min(0.122, float(
                            cdata.get("torso_wy", -0.08))))
                    except Exception:
                        pass
                    # Voice shape.  Absent keys keep the module default.
                    try:
                        self.voice = persona.VoiceStyle(
                            cdata.get("voice_vct"), cdata.get("voice_rspd"))
                    except Exception:
                        pass

            if os.path.isfile(secrets_path):
                with open(secrets_path, "r") as f:
                    sdata = json.load(f)
                    # Support both a single key and a list
                    single = sdata.get("gemini_key", "")
                    if single and single not in self.api_keys_list:
                        self.api_keys_list.append(single)
                    for k in sdata.get("gemini_keys", []):
                        if k and k not in self.api_keys_list:
                            self.api_keys_list.append(k)

            if self.api_keys_list:
                self.api_key_autofill = self.api_keys_list[0]
                print("[NaoAppWindow] Loaded %d API key(s) from config." % len(self.api_keys_list))
        except Exception as e:
            print("[NaoAppWindow] Could not load config/secrets: %s" % e)

        # Vision speaks for itself on human detection, so give it the same
        # character and voice rather than letting it use stock NAO.
        if self.vision is not None:
            try:
                self.vision.persona = self.persona
                self.vision.voice   = self.voice
            except Exception:
                pass

        self._controller_expanded = False
        # Live map state, shared with the wander thread so the map window can
        # draw it. Odometry frame, so it is only meaningful within one walk.
        self._map_blocked = []      # [(x, y, t)]
        self._map_trail   = []      # [(x, y)]
        self._map_pose    = None    # (x, y, theta)
        self._map_win     = None
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

        # Cards that are useful occasionally but mostly just take up room. They
        # stay collapsed until asked for, via the View menu.
        self._build_view_menu(left, right)

        self.root.after(500, self._start_ip_scan)

    # Titles of the cards hidden until the View menu asks for them.
    OPTIONAL_CARDS = ("Volume", "Language", "LEDs", "PS5 Controller", "Camera")

    @staticmethod
    def _collect_cards(parent):
        """(title, widget) for each card in a column, in packing order.

        Read back off the parent rather than threading return values through
        thirteen _card_* methods; the LabelFrame title is already the name."""
        out = []
        for w in parent.winfo_children():
            try:
                title = w.cget("text").strip()
            except Exception:
                title = ""
            out.append((title, w))
        return out

    def _build_view_menu(self, left, right):
        self._card_columns = [self._collect_cards(left), self._collect_cards(right)]
        self._card_vis = {}
        for cards in self._card_columns:
            for title, _w in cards:
                if title in self.OPTIONAL_CARDS:
                    self._card_vis[title] = tk.BooleanVar(value=False)

        menubar = tk.Menu(self.root)
        view = tk.Menu(menubar, tearoff=0)
        for title in self.OPTIONAL_CARDS:
            if title in self._card_vis:
                view.add_checkbutton(
                    label=title, variable=self._card_vis[title],
                    command=self._apply_card_visibility)
        view.add_separator()
        view.add_command(label="Show all", command=lambda: self._set_all_cards(True))
        view.add_command(label="Hide all", command=lambda: self._set_all_cards(False))
        view.add_separator()
        view.add_command(label="Map window", command=self._toggle_map_window)
        menubar.add_cascade(label="View", menu=view)
        self.root.config(menu=menubar)

        self._apply_card_visibility()

    # ---------------------------------------------------------------- map ---
    def _toggle_map_window(self):
        if getattr(self, "_map_win", None) is not None:
            self._close_map_window()
            return
        win = tk.Toplevel(self.root)
        win.title("NAO map")
        win.configure(bg=BG)
        win.geometry("440x480")
        self._map_canvas = tk.Canvas(win, bg="#11111b", highlightthickness=0)
        self._map_canvas.pack(fill="both", expand=True, padx=8, pady=(8, 4))
        self._map_info = tk.StringVar(value="No odometry yet - start a walk.")
        tk.Label(win, textvariable=self._map_info, font=self.font_small,
                 bg=BG, fg=FG).pack(anchor="w", padx=10, pady=(0, 6))
        win.protocol("WM_DELETE_WINDOW", self._close_map_window)
        self._map_win = win
        self._draw_map()

    def _close_map_window(self):
        win = getattr(self, "_map_win", None)
        self._map_win = None
        if win is not None:
            try: win.destroy()
            except Exception: pass

    def _draw_map(self):
        """Redraw the live map: trail, blocked points, and the robot.

        Everything is in the odometry frame, which only means anything within a
        single walk — it drifts over minutes and is reset by a fall."""
        win = getattr(self, "_map_win", None)
        if win is None:
            return
        c = self._map_canvas
        try:
            c.delete("all")
            w = max(c.winfo_width(), 50)
            h = max(c.winfo_height(), 50)
        except Exception:
            self._map_win = None
            return

        trail   = list(self._map_trail)
        blocked = list(self._map_blocked)
        pose    = self._map_pose

        pts = list(trail) + [(b[0], b[1]) for b in blocked]
        if pose:
            pts.append((pose[0], pose[1]))
        if not pts:
            c.create_text(w / 2, h / 2, text="No odometry yet.\nStart a walk.",
                          fill="#585b70", font=self.font_norm, justify="center")
            win.after(400, self._draw_map)
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # Always show at least a 2m window so a stationary robot is not zoomed
        # to absurdity, and keep the aspect ratio square so the map is not skewed.
        span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) * 1.15
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        scale = min(w, h) / span

        def to_px(x, y):
            # NAO odometry: +x forward, +y left. Draw +x up and +y left so the
            # map reads the way the robot was facing when the walk started.
            return (w / 2.0 - (y - cy) * scale,
                    h / 2.0 - (x - cx) * scale)

        # 1m grid
        step_px = scale
        if step_px > 12:
            n = int(span / 2.0) + 1
            for i in range(-n, n + 1):
                gx, gy = to_px(cx + i, cy)
                c.create_line(0, gy, w, gy, fill="#1e1e2e")
                gx, gy = to_px(cx, cy + i)
                c.create_line(gx, 0, gx, h, fill="#1e1e2e")

        # Blocked places, with the radius actually used for avoidance
        for bx, by, _bt in blocked:
            px, py = to_px(bx, by)
            r = 0.45 * scale
            c.create_oval(px - r, py - r, px + r, py + r,
                          outline="#f38ba8", fill="#3a1f2b")
            c.create_oval(px - 2, py - 2, px + 2, py + 2,
                          outline="", fill="#f38ba8")

        # Trail
        if len(trail) > 1:
            flat = []
            for x, y in trail:
                px, py = to_px(x, y)
                flat.extend([px, py])
            c.create_line(*flat, fill="#89b4fa", width=2, smooth=True)

        # Robot, as an arrow pointing where it is facing
        if pose:
            x, y, th = pose
            px, py = to_px(x, y)
            nose  = to_px(x + 0.18 * math.cos(th), y + 0.18 * math.sin(th))
            left  = to_px(x + 0.07 * math.cos(th + 2.5),
                          y + 0.07 * math.sin(th + 2.5))
            right = to_px(x + 0.07 * math.cos(th - 2.5),
                          y + 0.07 * math.sin(th - 2.5))
            c.create_polygon(nose[0], nose[1], left[0], left[1],
                             right[0], right[1],
                             fill="#a6e3a1", outline="#1e1e2e")

        self._map_info.set(
            "%d blocked  |  %.1fm trail  |  %.1fm across  |  odometry frame"
            % (len(blocked),
               sum(math.hypot(trail[i][0] - trail[i - 1][0],
                              trail[i][1] - trail[i - 1][1])
                   for i in range(1, len(trail))) if len(trail) > 1 else 0.0,
               span))
        win.after(400, self._draw_map)

    def _set_all_cards(self, shown):
        for var in self._card_vis.values():
            var.set(bool(shown))
        self._apply_card_visibility()

    def _apply_card_visibility(self):
        """Repack each column so hidden cards leave no gap and the surviving
        cards keep their original order (pack() alone would send a re-shown card
        to the bottom)."""
        for cards in getattr(self, "_card_columns", []):
            for _title, w in cards:
                w.pack_forget()
            for title, w in cards:
                var = self._card_vis.get(title)
                if var is None or var.get():
                    w.pack(fill="x", pady=4)

    def _card_connection(self, parent):
        card = make_card(parent, "Connection", self.font_head)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="IP:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.ip_entry = tk.Entry(row, font=self.font_norm, width=16)
        default_ip = self.config_ip or "192.168.0.123"
        self._ip_default_value = default_ip
        self.ip_entry.insert(0, default_ip)
        self.ip_entry.pack(side="left", padx=4)
        tk.Label(row, text="Port:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.port_entry = tk.Entry(row, font=self.font_norm, width=6)
        self.port_entry.insert(0, self.config_port or "9559")
        self.port_entry.pack(side="left", padx=4)
        make_btn(row, "Connect", self._on_connect, width=10, font_norm=self.font_norm).pack(side="left", padx=6)

    def _start_ip_scan(self):
        if getattr(self, "_ip_scan_running", False):
            return
        self._ip_scan_running = True

        def _scan():
            try:
                local_ip = self._get_local_ip()
                if not local_ip:
                    self.root.after(0, lambda: self._set_status("IP scan failed: local IP not found", False))
                    return
                parts = local_ip.split(".")
                if len(parts) != 4:
                    self.root.after(0, lambda: self._set_status("IP scan failed: bad local IP", False))
                    return
                prefix = ".".join(parts[:3]) + "."
                try:
                    port = int(self.port_entry.get().strip() or "9559")
                except Exception:
                    port = 9559

                for i in range(1, 255):
                    if not getattr(self, "_ip_scan_running", False):
                        break
                    target = prefix + str(i)
                    if target == local_ip:
                        continue
                    if self._is_port_open(target, port, timeout=0.15):
                        def _apply():
                            current = self.ip_entry.get().strip()
                            if current in ("", self._ip_default_value, self.config_ip, "192.168.0.123"):
                                self.ip_entry.delete(0, tk.END)
                                self.ip_entry.insert(0, target)
                                self._set_status("Suggested IP: %s" % target)
                        self.root.after(0, _apply)
                        break
            finally:
                self._ip_scan_running = False

        t = threading.Thread(target=_scan)
        t.daemon = True
        t.start()

    def _get_local_ip(self):
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except Exception:
                return None

    def _is_port_open(self, host, port, timeout=0.15):
        import socket
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            return s.connect_ex((host, port)) == 0
        except Exception:
            return False
        finally:
            if s:
                try:
                    s.close()
                except Exception:
                    pass

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
        card = make_card(parent, "Speech & Voice", self.font_head)

        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        tk.Label(row, text="Say:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.speech_entry = tk.Entry(row, font=self.font_norm, width=22)
        self.speech_entry.insert(0, persona.identity_sentence(spoken=True))
        self.speech_entry.pack(side="left", padx=4)
        make_btn(row, "Speak", self._on_speak, width=8, font_norm=self.font_norm).pack(side="left", padx=6)

        # Voice shape.  The preset menu is what people will actually use; the
        # two sliders are there for dialling in a voice the presets miss.
        row2 = tk.Frame(card, bg=CARD_BG)
        row2.pack(fill="x", pady=(6, 0))
        tk.Label(row2, text="Voice:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.voice_preset_var = tk.StringVar(value=persona.DEFAULT_PRESET)
        names = [p[0] for p in persona.VOICE_PRESETS]
        m = tk.OptionMenu(row2, self.voice_preset_var, *names,
                          command=self._on_voice_preset)
        m.config(font=self.font_norm, bg=BTN_BG, fg=BTN_FG,
                 activebackground=ACCENT, highlightthickness=0, width=10)
        m.pack(side="left", padx=4)
        make_btn(row2, "Test", self._on_voice_test, width=6,
                 font_norm=self.font_norm).pack(side="left", padx=4)
        make_btn(row2, "Save", self._on_voice_save, width=6,
                 font_norm=self.font_norm).pack(side="left")

        self.voice_vct_var  = tk.IntVar(value=self.voice.vct)
        self.voice_rspd_var = tk.IntVar(value=self.voice.rspd)
        for label, var, lo, hi in (
                ("Depth", self.voice_vct_var,
                 persona.VCT_MIN, persona.VCT_MAX),
                ("Speed", self.voice_rspd_var,
                 persona.RSPD_MIN, persona.RSPD_MAX)):
            r = tk.Frame(card, bg=CARD_BG)
            r.pack(fill="x")
            tk.Label(r, text=label, font=self.font_small, bg=CARD_BG, fg=FG,
                     width=6, anchor="w").pack(side="left")
            tk.Scale(r, from_=lo, to=hi, orient="horizontal", variable=var,
                     length=200, bg=CARD_BG, fg=FG, troughcolor=BTN_BG,
                     highlightthickness=0, font=self.font_small,
                     command=self._on_voice_slider).pack(side="left", padx=4)
        tk.Label(card, text="lower Depth = deeper, bigger-sounding voice",
                 font=self.font_small, bg=CARD_BG, fg=FG).pack(anchor="w")

    # -- voice controls -------------------------------------------------

    def _on_voice_preset(self, name=None):
        name = name or self.voice_preset_var.get()
        for label, vct, rspd in persona.VOICE_PRESETS:
            if label == name:
                self.voice.vct, self.voice.rspd = vct, rspd
                self.voice_vct_var.set(vct)
                self.voice_rspd_var.set(rspd)
                self._set_status("Voice preset: %s" % label)
                break

    def _on_voice_slider(self, _value=None):
        """Sliders win over the preset menu, so blank the preset label."""
        self.voice.vct  = self.voice_vct_var.get()
        self.voice.rspd = self.voice_rspd_var.get()
        self.voice.clamp()

    def _on_voice_test(self):
        if not self._require_connection() or not self.conn.tts: return
        self._on_voice_slider()
        line = self.persona.line("greeting")
        threading.Thread(target=lambda: self.say(line, blocking=True)).start()
        self._set_status("Voice test (depth %d, speed %d)"
                         % (self.voice.vct, self.voice.rspd))

    def _on_voice_save(self):
        """Persist the current voice to config.json so it survives a restart."""
        self._on_voice_slider()
        path = getattr(self, "config_path", None)
        if not path:
            self._set_status("No config.json path known", False)
            return
        try:
            import json, os
            data = {}
            if os.path.isfile(path):
                with open(path, "r") as f:
                    data = json.load(f)
            data.update(self.voice.as_dict())
            data.setdefault("_comment_voice",
                            "Acapela voice shape. voice_vct is vocal tract "
                            "length 50-150 (100 = stock NAO, lower = deeper); "
                            "voice_rspd is relative speed 60-140.")
            with open(path, "w") as f:
                json.dump(data, f, indent=4)
            self._set_status("Voice saved (depth %d, speed %d)"
                             % (self.voice.vct, self.voice.rspd))
        except Exception as e:
            self._set_status("Could not save voice: %s" % e, False)

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
        make_btn(row, "Dance", self._on_dance, width=8, font_norm=self.font_norm).pack(side="left", padx=6)

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

    def _all_api_keys(self):
        """Return deduplicated list of API keys: UI entry first, then secrets."""
        keys = []
        ui_key = self.gemini_key_entry.get().strip() if hasattr(self, 'gemini_key_entry') else ""
        if ui_key:
            keys.append(ui_key)
        for k in getattr(self, 'api_keys_list', []):
            if k and k not in keys:
                keys.append(k)
        return keys

    def _make_gemini_client(self):
        """Create a GeminiClient pre-loaded with all available API keys."""
        from nao_app.ai.gemini_client import GeminiClient
        client = GeminiClient()
        client.set_api_keys(self._all_api_keys())
        return client

    def _card_gemini(self, parent):
        card = make_card(parent, "AI Chat (Gemini)", self.font_head)
        
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=(0, 6))
        tk.Label(r1, text="API Key:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left")
        self.gemini_key_entry = tk.Entry(r1, font=self.font_norm, show="*", width=20)
        if self.api_key_autofill:
            self.gemini_key_entry.insert(0, self.api_key_autofill)
        self.gemini_key_entry.pack(side="left", padx=4)

        # Live key-count label — updates whenever the entry changes
        n_keys = len(self.api_keys_list) if self.api_keys_list else (1 if self.api_key_autofill else 0)
        self.gemini_keys_lbl_var = tk.StringVar(
            value=("%d key(s) loaded" % n_keys) if n_keys else "no key")
        self.gemini_keys_lbl = tk.Label(r1, textvariable=self.gemini_keys_lbl_var,
                                        font=self.font_small, bg=CARD_BG,
                                        fg=SUCCESS if n_keys else ERROR)
        self.gemini_keys_lbl.pack(side="left", padx=(6, 0))
        
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
        
        self.include_image = tk.BooleanVar(value=True)
        tk.Checkbutton(r4, text="Include Vision", variable=self.include_image, font=self.font_small, bg=CARD_BG, fg=FG, selectcolor=CARD_BG, activebackground=CARD_BG, activeforeground=FG).pack(side="right")

    def _card_autonomous(self, parent):
        card = make_card(parent, "Autonomous Wander", self.font_head)
        
        info = tk.Label(card, text="NAO walks, avoids walls, and searches for a target using its eyes.",
                        font=self.font_small, bg=CARD_BG, fg=FG, wraplength=300, justify="left")
        info.pack(fill="x", pady=(0, 6))

        # Target object entry
        rt = tk.Frame(card, bg=CARD_BG)
        rt.pack(fill="x", pady=2)
        tk.Label(rt, text="Search for:", font=self.font_norm, bg=CARD_BG, fg=FG).pack(side="left", padx=(0, 4))
        self.wander_target_var = tk.StringVar()
        self.wander_target_entry = tk.Entry(rt, textvariable=self.wander_target_var,
                                            font=self.font_norm, width=20)
        self.wander_target_entry.pack(side="left", padx=2)
        tk.Label(rt, text='e.g. "white shoe"', font=self.font_small, bg=CARD_BG, fg=FG).pack(side="left", padx=(4, 0))

        # Floor boundary checkbox
        rb = tk.Frame(card, bg=CARD_BG)
        rb.pack(fill="x", pady=2)
        self.wander_boundary_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rb, text="Stay on green floor (boundary detection)",
                       variable=self.wander_boundary_var,
                       font=self.font_small, bg=CARD_BG, fg=FG,
                       selectcolor=CARD_BG, activebackground=CARD_BG).pack(side="left")

        # Buttons
        r1 = tk.Frame(card, bg=CARD_BG)
        r1.pack(fill="x", pady=(4, 2))
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
                if self.conn.tts:
                    # Default the robot to English on connect rather than just
                    # reporting whatever it was left in.  The Language card is
                    # hidden by default now, so nobody would notice it had come
                    # up in another language until it started talking.
                    try:
                        self.conn.tts.setLanguage("English")
                        self.lang_var.set("English")
                    except Exception as e:
                        print("[Connect] could not set English: %s" % e)
                        try: self.lang_var.set(self.conn.tts.getLanguage())
                        except Exception: pass
                    self.say_line("greeting")
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

    # ------------------------------------------------------------------
    # Speech
    # ------------------------------------------------------------------

    def say(self, text, blocking=False):
        """Speak `text` in the robot's configured voice.

        Every spoken line in the app goes through here, so the personality
        stays consistent and one voice setting covers phrase banks, Gemini
        replies and the Speech Test box alike.  Never raises: speech failing
        must not take down a control loop."""
        if not text or not self.conn or not self.conn.tts:
            return
        try:
            styled = self.voice.apply(text)
        except Exception:
            styled = str(text)
        try:
            if blocking:
                self.conn.tts.say(styled)
            else:
                self.conn.tts.post.say(styled)
        except Exception:
            pass

    @staticmethod
    def _in_character(prompt):
        """Prepend the character brief to a Gemini prompt.

        Anything the robot will read aloud goes through here so a generated
        reply sounds like the same robot as the phrase banks - and so it
        always knows it is NAO, from FIA Robotics at Linkoping University.
        Deliberately NOT used for the yes/no vision classifier in the wander
        loop: personality there would only muddy the answer parsing."""
        return persona.system_prompt() + "\n\n" + str(prompt)

    def say_line(self, category, blocking=False, **fmt):
        """Speak a random, non-repeating line from a persona phrase bank."""
        try:
            text = self.persona.line(category, **fmt)
        except Exception:
            return ""
        if text:
            self.say(text, blocking=blocking)
        return text

    def _on_speak(self):
        if not self._require_connection() or not self.conn.tts: return
        text = self.speech_entry.get().strip()
        if not text: return
        threading.Thread(target=lambda: self.say(text, blocking=True)).start()
        self._set_status("Speaking: \"%s\"" % text)

    def _on_led_colour(self, colour_hex):
        if not self._require_connection() or not self.conn.leds: return
        group = self.led_group_var.get()
        try:
            self.conn.leds.fadeRGB(group, colour_hex, 0.3)
            self._set_status("LEDs %s -> #%06x" % (group, colour_hex))
        except Exception as e:
            self._set_status("LED error: %s" % e, False)

    def _emergency_relax(self):
        """Hard stop. Halt every autonomous loop, kill in-flight motion tasks,
        then release the motors.

        Relax has to win outright.  It used to just call rest(), while the
        wander thread carried on issuing moveToward/setAngles — so the robot
        went limp for an instant and was immediately driven again, which is
        precisely when someone is trying to make it safe.  Order matters here:
        set the lock first so no loop can queue new commands, and only then
        release the joints."""
        import time                     # module-scope 'time' is not imported here
        self._motion_locked = True      # loops must stop issuing commands
        self._seeking = False           # ends the wander/seek thread
        self._auto_task_done = True

        try:
            self.auto_status.set("EMERGENCY RELAX")
        except Exception:
            pass
        print("[Relax] emergency stop: locking motion and releasing joints")

        motion = self.conn.motion
        if motion:
            # stopMove ends the walk; killAll cancels every queued/running
            # motion task (posture transitions, animations, setAngles ramps)
            # that would otherwise keep executing after rest().
            for step in ("stopMove", "killAll", "rest"):
                try:
                    getattr(motion, step)()
                except Exception as e:
                    print("[Relax] %s failed: %s" % (step, e))
            try:
                motion.setStiffnesses("Body", 0.0)
            except Exception:
                pass
            # Re-assert once more. A command that was already in flight when we
            # locked can land just after rest() and re-stiffen a joint, which is
            # what made Relax look unreliable.
            time.sleep(0.35)
            for step in ("stopMove", "killAll", "rest"):
                try:
                    getattr(motion, step)()
                except Exception:
                    pass
            try:
                motion.setStiffnesses("Body", 0.0)
            except Exception:
                pass

        # Silence anything mid-sentence too — it is confusing to hear the robot
        # narrate a task it has just been stopped from doing.
        if self.conn.tts:
            try:
                self.conn.tts.stopAll()
            except Exception:
                pass
        self._set_status("Relaxed. Motion locked until the next command.")

    def _on_posture(self):
        if not self._require_connection() or not self.conn.posture: return
        name = self.posture_var.get()
        self._set_status("Going to posture: %s ..." % name)
        self.root.update_idletasks()
        def _go():
            try:
                if name == "Relax":
                    self._emergency_relax()
                else:
                    # Any deliberate posture command clears the lock.
                    self._motion_locked = False
                    # Wake first.  A resting robot has zero stiffness, so
                    # goToPosture() has no motors to work with and the button
                    # appears to do nothing at all — which is exactly what it
                    # did before this.  (_run_dance already did this, so Dance
                    # worked from rest while Posture did not.)
                    if self.conn.motion:
                        try:
                            if not self.conn.motion.robotIsWakeUp():
                                self._set_status("Waking up for posture...")
                            self.conn.motion.wakeUp()
                            self.conn.motion.setStiffnesses("Body", 1.0)
                        except Exception as e:
                            print("[Posture] wake-up warning: %s" % e)
                    if name in ("Sit", "SitRelax", "Crouch"):
                        self.conn.posture.goToPosture("StandInit", 0.5)
                        self.conn.posture.goToPosture(name, 0.5)
                    else:
                        self.conn.posture.goToPosture(name, 0.6)
                self._set_status("Posture: %s" % name)
            except Exception as e:
                self._set_status("Posture error: %s" % e, False)
        threading.Thread(target=_go).start()

    def _on_dance(self):
        if not self._require_connection(): return
        self._set_status("Waking up for dance...")
        self.root.update_idletasks()
        t = threading.Thread(target=self._run_dance)
        t.daemon = True
        t.start()

    def _run_dance(self):
        """Execute the full dance sequence (blocking). Safe to call from any thread."""
        import time
        if not self._motion_allowed():
            print("[Dance] skipped: motion locked (relax).")
            return
        try:
            if self.conn.motion:
                self.conn.motion.wakeUp()
            if self.conn.posture:
                self.conn.posture.goToPosture("StandInit", 0.5)

            dance_played = False

            # 1. Try built-in ALAnimationPlayer (Standard Choregraphe animations)
            anim_proxy = self.conn.get_proxy("ALAnimationPlayer")
            if anim_proxy:
                dances = [
                    "animations/Stand/Gestures/Excited_1",
                    "animations/Stand/Gestures/Joy_1",
                    "animations/Stand/Emotions/Positive/Excited_1",
                    "animations/Stand/Emotions/Positive/Happy_4",
                    "animations/Stand/Gestures/Enthusiastic_5",
                    "animations/Stand/Gestures/Taichi_1",
                    "animations/Stand/Gestures/ComeOn_1",
                ]
                for d in dances:
                    try:
                        self._set_status("Dancing: %s..." % d.split('/')[-1])
                        anim_proxy.run(d)
                        dance_played = True
                        break
                    except Exception:
                        pass

            # 2. Try ALBehaviorManager if animation player failed
            if not dance_played:
                behavior = self.conn.get_proxy("ALBehaviorManager")
                if behavior:
                    for b in behavior.getInstalledBehaviors():
                        if "taichi" in b.lower() or "dance" in b.lower() or "macarena" in b.lower():
                            self._set_status("Executing behavior: %s" % b)
                            if not behavior.isBehaviorRunning(b):
                                behavior.runBehavior(b)
                            dance_played = True
                            break

            # 3. Fallback: hardcoded Python dance
            if not dance_played and self.conn.motion:
                self._set_status("Doing a custom Python dance!")
                m = self.conn.motion
                names = ["LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll"]
                m.setAngles(names, [-1.0, -1.0, 0.5, -0.5], 0.2)
                time.sleep(1.0)
                m.moveToward(0.0, 0.0, 0.5)
                time.sleep(1.5)
                m.moveToward(0.0, 0.0, -0.5)
                time.sleep(1.5)
                m.stopMove()
                for _ in range(2):
                    m.setAngles(["LShoulderPitch", "RShoulderPitch"], [0.0, -1.5], 0.3)
                    time.sleep(0.5)
                    m.setAngles(["LShoulderPitch", "RShoulderPitch"], [-1.5, 0.0], 0.3)
                    time.sleep(0.5)
                m.setAngles(names, [1.5, 1.5, 0.1, -0.1], 0.2)
                time.sleep(0.5)
                if self.conn.posture:
                    self.conn.posture.goToPosture("StandInit", 0.5)
                dance_played = True

            if dance_played:
                self._set_status("Finished dance.")
            else:
                self._set_status("Could not dance. No modules available.", False)
        except Exception as e:
            self._set_status("Dance error: %s" % e, False)

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

    def _capture_image_bytes(self, resolution=2):
        """Capture one frame from NAO's camera and return it as PNG bytes,
        or None if capture fails.  Works entirely within Python 2.7 stdlib —
        no Pillow, no subprocess, no temp files.

        resolution: NAOqi resolution code (0=QQVGA 160x120, 1=QVGA 320x240,
                    2=VGA 640x480).  Use 1 for fast/cheap search snaps,
                    2 (default) for full-quality AI photo requests."""
        video = self.conn.video
        if not video:
            print("[Vision] No video proxy (conn.video is None - is NAO connected?)")
            return None

        w, h, payload = None, None, None

        # Re-use the live camera frame only for full-quality requests; for
        # low-resolution search snaps always do a fresh subscribe so we
        # get the right resolution.
        if (resolution == 2
                and getattr(self, 'vision', None)
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
                cam_name = video.subscribe("gemini_snap", resolution, 11, 5)
                video.getImageRemote(cam_name)      # discard first (dark) frame
                time.sleep(0.4)                     # let auto-exposure settle
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
        prompt  = self.gemini_prompt_entry.get().strip()

        if not self._all_api_keys():
            self.gemini_status.set("Error: Need API Key!")
            return
        if not prompt:
            self.gemini_status.set("Enter a prompt!")
            return

        self.gemini_status.set("Thinking...")
        self.root.update_idletasks()

        def _fetch_and_say():
            try:
                client = self._make_gemini_client()
                
                image_bytes = None
                if getattr(self, 'include_image', None) and self.include_image.get():
                    image_bytes = self._capture_image_bytes()
                    if image_bytes is None:
                        self._set_status("No image captured - sending text prompt only.", False)

                response_text = client.generate_text(
                    self._in_character(prompt), image_bytes=image_bytes)

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

        if not self._all_api_keys():
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
                
                client = self._make_gemini_client()
                
                # We let the user type a custom extra prompt, or use a default one
                prompt = self.gemini_prompt_entry.get().strip()
                if not prompt:
                    prompt = "Please listen to the attached audio recording of my voice. Answer what I say naturally."
                
                # Take a picture from the robot's eyes to send to Gemini
                image_bytes = None
                if getattr(self, 'include_image', None) and self.include_image.get():
                    image_bytes = self._capture_image_bytes()
                    if image_bytes is None:
                        self._set_status("No image captured - sending audio only.", False)
                
                response_text = client.generate_text(
                    self._in_character(prompt),
                    audio_bytes=wav_bytes, image_bytes=image_bytes)
                
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
            
        self.say(safe_text, blocking=True)
        
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
                    self.say(cmd.split("say ", 1)[-1])
                elif "speak " in cmd:
                    self.say(cmd.split("speak ", 1)[-1])
                
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

        target       = getattr(self, 'wander_target_var',   None)
        target       = target.get().strip() if target else ""
        # Blank target means "just go for a walk" — no search, no win condition.
        target_label = target
        use_boundary = getattr(self, 'wander_boundary_var', None)
        use_boundary = bool(use_boundary.get()) if use_boundary else False
        api_keys     = self._all_api_keys()

        # Starting a wander is a deliberate user action, so it clears any
        # standing emergency-relax lock.
        self._motion_locked = False
        self._seeking = True
        if target:
            self.auto_status.set("Searching for: %s" % target)
        else:
            self.auto_status.set("Going for a walk...")
        self._set_status("Autonomous wander started. Use 'Stop Auto' to abort.")
        if target:
            self.say_line("search_start", target=target_label)
        else:
            self.say_line("walk_start")
        threading.Thread(target=self._wander_seek_thread,
                         args=(target, use_boundary, api_keys)).start()

    def _stop_wander_seek(self):
        if getattr(self, "_seeking", False):
            self._seeking = False
            self.auto_status.set("Stopping...")
            self._set_status("Stopping autonomous mode...")

    def _wander_seek_thread(self, target="", use_boundary=False, api_keys=None):
        import time
        import math
        import random

        if api_keys is None:
            api_keys = []

        # Friendly display name used in speech and status messages.  Empty when
        # no target was typed, which is the "just go for a walk" mode.
        target_label = target.strip()

        sonar          = None
        floor_cam_name = None
        # Must be bound before any early return: the finally block unsubscribes
        # it, and an abort (e.g. "failed to stand") used to raise
        # UnboundLocalError there, masking the real reason for the abort.
        novelty_cam_name = None
        wander_video   = self.conn.video

        # One persistent Gemini client for the whole wander session so that
        # exhausted-key state is preserved across multiple vision checks.
        from nao_app.ai.gemini_client import GeminiClient
        search_client = GeminiClient()
        search_client.set_api_keys(api_keys)
        has_api = bool(search_client.api_key)

        # Words that indicate the search target is a human being.
        # When matched, the faster NAO SDK face detection runs alongside Gemini
        # and whichever fires first wins.
        _HUMAN_KEYWORDS = (
            "human", "person", "people", "man", "woman",
            "boy", "girl", "face", "someone", "anybody",
        )
        # Only a target that actually names a person counts as a human search.
        # A blank target used to mean "hunt for a face", which made a plain walk
        # impossible; now blank means walk, with no win condition at all.
        target_is_human = bool(target) and any(
            k in target.lower() for k in _HUMAN_KEYWORDS)
        _sdk_found  = [False]
        _task_done  = [False]   # set True the moment target found; aborts in-flight Gemini checks

        try:
            self._set_status("Standing up safely...")
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

            # 'robotHasFallen' is a latched flag: it stays True after a fall until
            # something clears it.  Left set, the very next tilt spike trips the
            # fall branch instantly, so clear it before we start walking.
            if self.conn.memory:
                try: self.conn.memory.insertData("robotHasFallen", False)
                except Exception: pass

            if self.conn.motion:
                try:
                    self.conn.motion.setFallManagerEnabled(True)
                    _arms = bool(getattr(self, "config_walk_arms", True))
                    self.conn.motion.setMoveArmsEnabled(_arms, _arms)
                    print("[Gait] arm swing during walk: %s"
                          % ("on" if _arms else "OFF (experiment)"))
                    self.conn.motion.setMotionConfig([
                        ["ENABLE_FOOT_CONTACT_PROTECTION", True],
                    ])
                    # Legs must actually be at full stiffness before walking — a
                    # partially relaxed leg is the classic cause of a collapse a
                    # few steps in.
                    try: self.conn.motion.setStiffnesses("Body", 1.0)
                    except Exception: pass
                    # NOTE: setSmartStiffnessEnabled() is deliberately NOT used
                    # here.  It lets NAOqi lower stiffness on joints it thinks
                    # are idle, and a leg joint going soft mid-stride is exactly
                    # what a forward collapse looks like.
                    #
                    # "Move" is excluded from collision protection on purpose too:
                    # it halts the walk abruptly when it predicts contact, and an
                    # abrupt stop mid-stride is itself a fall risk.  Obstacles are
                    # handled by the sonar/bumper logic in the loop instead.
                    try: self.conn.motion.setExternalCollisionProtectionEnabled("Arms", True)
                    except Exception: pass
                except Exception:
                    pass

            # --- Sonar ---
            # ALSonar must be subscribed or the US/*/Sensor/Value keys simply
            # stop refreshing — and a frozen "nothing ahead" reading looks
            # exactly like an empty room.  The old code swallowed the failure
            # silently, so verify it actually took and say so if it did not.
            sonar = self.conn.get_proxy("ALSonar")
            if sonar:
                try:
                    sonar.subscribe("WanderSeeker")
                except Exception as e:
                    print("[Sense] ALSonar.subscribe failed: %s" % e)
                try:
                    subs = sonar.getSubscribersInfo()
                    if not subs:
                        print("[Sense] WARNING: ALSonar reports no subscribers — "
                              "sonar values will be stale. Relying on bumpers.")
                    else:
                        print("[Sense] sonar active (%d subscriber(s))." % len(subs))
                except Exception:
                    pass
            else:
                print("[Sense] WARNING: no ALSonar proxy — bumpers only.")

            # --- Face detection (used in face-seek mode only) ---
            if self.conn.face:
                try: self.conn.face.subscribe("WanderSeekerFace")
                except Exception: pass

            tracker = self.conn.get_proxy("ALTracker")

            # --- Floor camera (QQVGA 160x120 for fast color analysis) ---
            floor_threshold   = 0.25   # will be replaced by calibration if boundary mode
            _floor_bottom_cam = False  # True when using NAO's chin cam (no head tilt needed)
            if use_boundary and wander_video:
                try: wander_video.unsubscribe("wander_floor_cam")
                except Exception: pass
                try:
                    # Prefer the bottom (chin) camera — it always points at the floor
                    # so the top-camera head is completely free for active scanning.
                    try:
                        floor_cam_name = wander_video.subscribeCamera(
                            "wander_floor_cam", 1, 0, 11, 10)   # 1 = bottom camera
                        _floor_bottom_cam = True
                        print("[Floor] Using bottom camera — head free for active scanning")
                    except Exception:
                        floor_cam_name = wander_video.subscribe("wander_floor_cam", 0, 11, 10)
                        print("[Floor] Bottom camera unavailable, will tilt head for calibration")

                    # Only tilt head down for calibration when we must use the top camera
                    if not _floor_bottom_cam and self.conn.motion:
                        self.conn.motion.setAngles("HeadPitch", 0.45, 0.30)
                    self.auto_status.set("Calibrating floor sensor... stay still")
                    self._set_status("Calibrating floor boundary (2 seconds)...")

                    time.sleep(2.0)
                    baseline_samples = []
                    for _ in range(6):
                        _, ratio = self._check_floor_boundary(floor_cam_name, wander_video)
                        if ratio is not None:
                            baseline_samples.append(ratio)
                        time.sleep(0.15)

                    if baseline_samples:
                        baseline = sum(baseline_samples) / float(len(baseline_samples))
                        floor_threshold = max(0.10, baseline * 0.50)
                        self._set_status(
                            "Floor calibrated: baseline=%.2f, trigger below %.2f" % (
                                baseline, floor_threshold))
                        print("[Floor] Calibrated baseline=%.2f threshold=%.2f" % (
                            baseline, floor_threshold))
                    else:
                        self._set_status("Floor calibration failed – using default threshold.", False)
                        print("[Floor] Calibration failed, no valid samples.")

                    # Return head to forward level after top-camera calibration tilt
                    if not _floor_bottom_cam and self.conn.motion:
                        try: self.conn.motion.setAngles("HeadPitch", 0.0, 0.20)
                        except Exception: pass
                except Exception as e:
                    print("[Wander] Floor camera setup failed: %s" % e)

            # --- Head tilt for object search on the floor ---
            # Static tilt removed: the active scanning loop below cycles through
            # all angles, including a floor-facing slot used for boundary checks.

            # --- Local novelty detector (no API, runs every second) ---
            # Divides the QQVGA frame into a grid of colour zones.  When enough
            # zones change colour significantly vs. the calibrated background,
            # a Gemini check is triggered immediately instead of waiting for
            # the 15-second timer.  This lets NAO react the moment an object
            # enters its field of view rather than relying on lucky timing.
            novelty_cam_name = None
            _novelty_baseline    = [None]   # list of (r,g,b) per zone
            _novelty_triggered   = [False]
            _novelty_last_t      = [0.0]
            _NOVELTY_INTERVAL    = 1.0      # seconds between local checks
            _NOVELTY_THRESHOLD   = 0.35     # fraction of zones that must change
            _NOVELTY_MIN_CALL_GAP = 5.0     # shortest Gemini interval on novelty trigger

            def _zone_means(raw, w, h, n_cols=4, n_rows=3):
                """Return per-zone mean (R,G,B) for the lower 2/3 of a frame."""
                data     = bytearray(raw) if not isinstance(raw, bytearray) else raw
                row_size = w * 3
                start_y  = h // 3          # top third is usually ceiling/sky
                zone_h   = max(1, (h - start_y) // n_rows)
                zone_w   = max(1, w // n_cols)
                zones = []
                for rz in range(n_rows):
                    for cz in range(n_cols):
                        y0 = start_y + rz * zone_h
                        x0 = cz * zone_w
                        r_s = g_s = b_s = cnt = 0
                        for y in range(y0, min(y0 + zone_h, h), 2):
                            base = y * row_size
                            for x in range(x0, min(x0 + zone_w, w), 2):
                                idx = base + x * 3
                                if idx + 2 < len(data):
                                    r_s += data[idx]; g_s += data[idx+1]; b_s += data[idx+2]
                                    cnt += 1
                        zones.append((r_s // max(1, cnt),
                                      g_s // max(1, cnt),
                                      b_s // max(1, cnt)))
                return zones

            def _novelty_score(baseline, current, colour_thresh=30):
                """Fraction of zones whose colour shifted by more than thresh."""
                if not baseline or len(baseline) != len(current):
                    return 0.0
                changed = sum(
                    1 for (br, bg, bb), (cr, cg, cb) in zip(baseline, current)
                    if abs(br-cr) + abs(bg-cg) + abs(bb-cb) > colour_thresh
                )
                return float(changed) / len(baseline)

            def _novelty_blend(baseline, current, alpha=0.08):
                """Slowly absorb the current scene into the baseline (prevents
                false-positives as the robot gradually moves through a room)."""
                if baseline is None:
                    return current
                return [
                    (int(br + alpha * (cr - br)),
                     int(bg + alpha * (cg - bg)),
                     int(bb + alpha * (cb - bb)))
                    for (br, bg, bb), (cr, cg, cb) in zip(baseline, current)
                ]

            if target and wander_video:
                try:
                    try: wander_video.unsubscribe("wander_novelty_cam")
                    except Exception: pass
                    novelty_cam_name = wander_video.subscribe(
                        "wander_novelty_cam", 0, 11, 5)   # QQVGA 160x120
                    # Build initial background baseline from 3 frames
                    baseline_zones = []
                    for _ in range(3):
                        nimg = wander_video.getImageRemote(novelty_cam_name)
                        if nimg and len(nimg) >= 7:
                            baseline_zones.append(
                                _zone_means(nimg[6], int(nimg[0]), int(nimg[1])))
                        time.sleep(0.25)
                    if baseline_zones:
                        # Average the three baseline samples zone-by-zone
                        n_zones = len(baseline_zones[0])
                        avg = []
                        for zi in range(n_zones):
                            avg.append((
                                sum(b[zi][0] for b in baseline_zones) // len(baseline_zones),
                                sum(b[zi][1] for b in baseline_zones) // len(baseline_zones),
                                sum(b[zi][2] for b in baseline_zones) // len(baseline_zones),
                            ))
                        _novelty_baseline[0] = avg
                        print("[Novelty] Baseline built from %d zones." % n_zones)
                    else:
                        print("[Novelty] Could not build baseline — novelty detection disabled.")
                except Exception as e:
                    print("[Novelty] Setup failed: %s" % e)

            # --- Gemini target-search state ---
            _vision_checking    = [False]
            _target_found       = [False]
            _last_vision_t      = [time.time() - 10.0]  # first check after 5 s
            _VISION_INTERVAL    = 15.0   # seconds between Gemini calls (fallback timer)
            _vision_backoff_until = [0.0]    # absolute time: skip checks until here
            _vision_backoff_secs  = [30.0]   # starts at 30 s, doubles on each 429

            def _do_gemini_check(img_bytes):
                # Bail immediately if the target was already found
                if _task_done[0]:
                    _vision_checking[0] = False
                    return
                try:
                    # Use a strict YES/NO prompt that overrides the mobster personality
                    # so the model reliably returns one of the two expected tokens.
                    prompt = (
                        "VISION CHECK — answer with ONE word only, either YES or NO. "
                        "No other text, no punctuation, no mobster slang. "
                        "Look at this image carefully. "
                        "Is a '{}' clearly visible anywhere in the image? "
                        "Reply: YES or NO"
                    ).format(target)
                    result = search_client.generate_text(prompt, image_bytes=img_bytes)
                    # Bail if the target was found by SDK while we awaited the API
                    if _task_done[0]:
                        return
                    key_lbl = (search_client.active_key_label()
                               if hasattr(search_client, "active_key_label") else "?")
                    print("[VisionSearch] Gemini (%s): %s" % (key_lbl, result.strip()))
                    if "429" in result:
                        wait = _vision_backoff_secs[0]
                        _vision_backoff_until[0] = time.time() + wait
                        _vision_backoff_secs[0]  = min(wait * 2.0, 120.0)
                        print("[VisionSearch] All keys exhausted. Pausing vision for %.0f s." % wait)
                    else:
                        r_upper = result.upper().strip()
                        # Explicit YES/NO from the model
                        explicit_yes = "YES" in r_upper and not r_upper.startswith("NO")
                        # Implicit confirmation: the model described the target instead
                        # of answering YES (happens when the mobster personality overrides
                        # the YES/NO instruction).  Accept if the response mentions the
                        # target and doesn't open with a clear negative.
                        _neg_opens = ("NO ", "NOPE", "NOTHING", "NONE", "NOT ", "I DON",
                                      "CAN'T", "CANNOT", "DON'T SEE", "DO NOT SEE")
                        is_negative = any(r_upper.startswith(n) for n in _neg_opens)
                        target_words = [w for w in target_label.lower().split() if len(w) > 2]
                        implicit_yes = (
                            not is_negative
                            and bool(target_words)
                            and any(w in r_upper.lower() for w in target_words)
                        )
                        if explicit_yes or implicit_yes:
                            _target_found[0] = True
                            _task_done[0] = True
                            _vision_backoff_secs[0] = 30.0
                            how_det = "explicit" if explicit_yes else "implicit"
                            print("[VisionSearch] Target confirmed (%s)" % how_det)
                            self.auto_status.set("FOUND: %s!" % target_label)
                except Exception as e:
                    print("[VisionSearch] Error: %s" % e)
                finally:
                    _vision_checking[0] = False

            def _capture_and_check():
                """Grab the frame *and* run the check on this worker thread.

                _capture_image_bytes() costs a camera re-subscribe, a hard 0.4 s
                exposure settle and two full frame transfers, then a pure-Python
                PNG encode — on the order of a second.  It used to run on the
                wander loop itself, and because moveToward() is a continuous
                velocity command the robot kept walking that entire time with no
                sonar checks at all.  That was the single biggest source of
                bumping whenever a search target was set."""
                img = None
                try:
                    img = self._capture_image_bytes(resolution=1)   # QVGA 320x240
                except Exception as e:
                    print("[VisionSearch] capture error: %s" % e)
                if not img:
                    _vision_checking[0] = False
                    return
                _do_gemini_check(img)

            # --- Walk config & tuning ---
            walk_config       = getattr(self.controller, "_WALK_CONFIG", [])

            def _tuned_walk_config(base, overrides):
                if not base:
                    return base
                out = []
                seen = {}
                for key, val in base:
                    if key in overrides:
                        val = overrides[key]
                    out.append([key, val])
                    seen[key] = True
                for key, val in overrides.items():
                    if key not in seen:
                        out.append([key, val])
                return out

            # All values must stay inside the robot's getMoveConfig Min/Max
            # limits (MaxStepY min is 0.101!) or the gait is left undefined.
            # See GamepadController._WALK_CONFIG for the full reasoning.
            # Final consolidated gait for this robot, from the measured runs.
            # The failure is always the same shape: roll alternates sign and
            # roughly DOUBLES every half second during a step, and once it
            # passes ~0.2 the robot goes over whichever way it happens to lean.
            # So everything here aims at one thing — minimise the time the robot
            # spends on a single foot, which is the only window in which roll can
            # grow.
            walk_config = _tuned_walk_config(walk_config, {
                # Small steps: less disturbance injected per stride.
                "MaxStepX":         0.020,
                # Widest legal stance (max 0.160) for maximum lateral margin.
                "MaxStepY":         0.160,
                "MaxStepTheta":     0.100,
                # Back to the robot's own default of 1.0.  Higher frequency means
                # a SHORTER single-support phase, so there is less time per step
                # for roll to diverge.  (It was 0.7 on a resonance theory that
                # turned out to be built on an undersampled 2 Hz trace — with
                # 20 Hz sampling that reading did not hold up.)
                "MaxStepFrequency": 1.000,
                # Lower than default: the foot stays nearer the floor, cutting
                # airborne time.  Safe here — the office floor is flat with
                # nothing to trip on.
                "StepHeight":       0.015,
                "TorsoWx":          0.000,
                # Trim the measured forward lean.  Standing in StandInit this
                # robot sits at tiltY = +0.164 rad (~9 deg forward, confirmed
                # against raw gravity), which eats most of its forward margin —
                # and it has now face-planted from exactly that stance.  A
                # negative TorsoWy pitches the torso back and hands some of that
                # margin back.  Limit is +-0.122; this is deliberately not maxed.
                "TorsoWy":          float(getattr(self, "config_torso_wy", -0.08)),
            })

            # moveToward() takes velocities NORMALISED to [-1, 1] — a fraction of
            # MaxStepX, not m/s.  The old 0.05 meant ~5% of max, i.e. ~1.5mm of
            # travel per step: the robot rocked through a full weight shift for
            # almost no forward motion, which is the *least* stable regime for a
            # ZMP walker.  A real step is both faster and far steadier.
            max_fwd           = 0.45
            wander_turn_bias  = 0.0
            last_bias_t       = time.time()
            next_speech_t     = time.time() + random.uniform(3.0, 20.0)
            walk_start_t      = time.time()
            floor_tick        = 0
            fall_flag_since   = None
            tilt_high_since   = None
            fall_debounce_s   = 0.6

            # --- Stability governor -------------------------------------------
            # The loop used to react only at _tilt_too_high() = 0.50 rad (~29°),
            # which is long past recoverable.  These thresholds brake the walk
            # while the balancer can still plant a corrective step.  Normal
            # walking oscillates well under _TILT_WARN, so this does not nag.
            _TILT_WARN   = 0.15   # rad — start scaling speed down
            # 0.38, not 0.28.  With discrete moveTo steps the combined tilt
            # magnitude routinely peaks around 0.22-0.29 during a perfectly
            # normal stride, and the old threshold was firing mid-step — which
            # calls stopMove() while the robot is on one foot, making a fall
            # more likely rather than less.
            _TILT_DANGER = 0.38   # rad — stop dead and let it settle
            _braking     = [False]

            # Command smoothing: feed the walk engine a continuous velocity
            # rather than step changes, so it never has to re-plan abruptly.
            _cur_fwd  = [0.0]
            _cur_turn = [0.0]
            _CMD_SMOOTH = 0.25

            # --- Proximity sensing --------------------------------------------
            # Sonar and both foot bumpers, batched into a single ALMemory read.
            # NAO's ultrasound is chest-mounted with a ~60° cone and a ~0.25 m
            # blind spot, and soft or thin things (fabric, chair legs) barely
            # echo — so sonar alone cannot see everything.  The bumpers are the
            # backstop: they cannot miss, they just report contact after it has
            # happened.
            _SONAR_KEYS = [
                "Device/SubDeviceList/US/Left/Sensor/Value",
                "Device/SubDeviceList/US/Right/Sensor/Value",
            ]
            _BUMPER_KEYS = [
                "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
                "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
                "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
                "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
            ]
            # Balance state rides along in the same batch: the loop used to spend
            # ~9 separate round trips per tick on these, which capped the reaction
            # rate and therefore the stopping distance.
            _BALANCE_KEYS = [
                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
                "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
                "robotHasFallen",
            ]

            # Validate every key once, up front.  A key this firmware does not
            # expose would make the batched read throw on every tick, and the
            # fail-safe below would then hold the robot permanently "blocked" —
            # so drop unknown keys instead of walking into that.
            def _key_works(k):
                try:
                    self.conn.memory.getData(k)
                    return True
                except Exception:
                    print("[Sense] key unavailable, ignoring: %s" % k)
                    return False

            if self.conn.memory:
                _SONAR_KEYS   = [k for k in _SONAR_KEYS   if _key_works(k)]
                _BUMPER_KEYS  = [k for k in _BUMPER_KEYS  if _key_works(k)]
                _BALANCE_KEYS = [k for k in _BALANCE_KEYS if _key_works(k)]
            if not _SONAR_KEYS:
                print("[Sense] WARNING: no sonar keys available — "
                      "relying on bumpers and the floor check only.")

            _N_SONAR    = len(_SONAR_KEYS)
            _N_BUMPER   = len(_BUMPER_KEYS)
            _SENSE_KEYS = _SONAR_KEYS + _BUMPER_KEYS + _BALANCE_KEYS
            # Measured on this robot: "no echo" reads as 5.0, real detections at
            # ~0.5 m come through intermittently, and the sensor refreshes ~5 Hz.
            _SONAR_CLEAR    = 5.0    # value the sensor reports when nothing is seen
            _STOP_DIST      = 0.45   # m — hard stop and turn away (blind spot is ~0.25)
            _SLOW_DIST      = 0.90   # m — start creeping below this
            _SONAR_WINDOW_S = 0.6    # s — act on the closest reading in this window
            _sonar_hist     = []     # list of (timestamp, distance)
            _sense_fail     = [0]
            _last_trace     = [0.0]  # periodic diagnostic trace

            # Below this normalised forward speed the robot performs a complete
            # weight-shift cycle for a millimetre of travel — the "shuffle band",
            # and measurably the least stable thing it can do.  Speed is either
            # at least this much or exactly zero, never in between.
            _MIN_WALK = 0.22

            # Divergent lateral oscillation is what actually topples this robot:
            # roll builds up cycle over cycle until it goes past recovery, and by
            # the time |tilt| crosses _TILT_DANGER it is already too late to stop.
            # Track the recent roll peak and back off while it is still growing.
            _ROLL_WARN   = 0.15   # rad — normal walking roll peaks around 0.09
            _ROLL_MAX    = 0.24   # rad — treat as a developing topple
            _ROLL_WINDOW = 2.0    # s
            _roll_hist   = []     # list of (timestamp, |roll|)

            # --- Burst gait ----------------------------------------------------
            # This robot cannot sustain a continuous walk.  Measured on NAOqi's
            # OWN default gait, straight line, head still, one command and none
            # of this app's logic, roll grows monotonically
            #     t=0s 0.02 -> t=2s 0.16 -> t=3s 0.22 -> t=4s 0.35 (over)
            # so it is an accumulation, not an instant failure — and standing
            # still is rock steady (sd 0.0003 rad) and visibly damps the
            # oscillation back down.  So: walk in bursts shorter than the
            # build-up, and stand still between them until it has settled.
            # Slower over the ground, but it stays upright.
            _BURST_S      = float(getattr(self, "config_burst_walk_s", 1.0))
            _SETTLE_MIN_S = float(getattr(self, "config_burst_settle_s", 1.2))
            _SETTLE_MAX_S = 4.0    # s — resume anyway rather than freezing
            _RESUME_ROLL  = float(getattr(self, "config_resume_roll", 0.06))
            # One discrete moveTo per burst.  Distance is deliberately short:
            # far enough that the engine actually plans real steps rather than
            # spending the whole time in its weight-shift preamble, but short
            # enough to finish before roll can build.
            _STEP_M       = float(getattr(self, "config_step_m", 0.12))
            _STEP_TURN    = 0.30   # rad of yaw at full wander bias
            _STEP_TIMEOUT = 8.0    # s — moveTo should be long done by now
            _step_issued  = [False]
            _phase        = ["settle"]   # start by settling after standing up
            _phase_t      = [time.time()]
            # Stuck detection.  The sonar is chest-mounted and forward-facing, so
            # a wall met at an angle — or contact at the shoulder — reads as
            # clear space.  The robot then marches on the spot indefinitely.
            # Odometry is the ground truth: if a commanded step produced almost
            # no displacement, it is jammed regardless of what sonar thinks.
            _step_pos0    = [None]
            _no_progress  = [0]
            _escape_dir   = [1.0]        # alternate, so a failed escape is not repeated
            _NO_PROGRESS_FRAC = 0.35     # of the commanded step distance
            _NO_PROGRESS_HITS = 2        # consecutive stalled steps before escaping

            # --- Spatial memory of blocked places -----------------------------
            # Odometry (getRobotPosition) gives x/y/theta in a frame that lasts
            # as long as the session, which is enough for a lightweight local
            # map.  Storage is not the constraint — this runs on the PC, so
            # thousands of points would be fine.  DRIFT is the constraint: leg
            # odometry accumulates error over minutes, and a fall destroys the
            # reference completely.  So points expire, and the map is cleared on
            # a fall rather than trusted afterwards.
            # Alias the shared list so the map window sees the same object.
            self._map_blocked = []
            self._map_trail   = []
            _blocked      = self._map_blocked   # [(x, y, t)] places that stopped us
            _BLOCK_TTL    = 180.0   # s — forget before drift makes it a lie
            _BLOCK_MAX    = 200     # cap, purely defensive
            _BLOCK_RADIUS = 0.45    # m — how wide a blocked point counts as
            _PROBE_AHEAD  = 0.60    # m — how far ahead a candidate heading looks

            def _robot_pose():
                try:
                    p = self.conn.motion.getRobotPosition(True)
                    pose = (p[0], p[1], p[2])
                except Exception:
                    return None
                self._map_pose = pose
                # Breadcrumb trail, thinned so it does not grow without bound.
                tr = self._map_trail
                if not tr or math.hypot(tr[-1][0] - pose[0],
                                        tr[-1][1] - pose[1]) > 0.04:
                    tr.append((pose[0], pose[1]))
                    if len(tr) > 1500:
                        del tr[0]
                return pose

            def _robot_xy():
                p = _robot_pose()
                return (p[0], p[1]) if p else None

            def _note_blocked(reason):
                """Remember here as a place that stopped us."""
                p = _robot_xy()
                if not p:
                    return
                now_t = time.time()
                # Merge into a nearby existing point instead of piling up
                # duplicates every time we bump the same wall.
                for i, (bx, by, _bt) in enumerate(_blocked):
                    if math.hypot(bx - p[0], by - p[1]) < _BLOCK_RADIUS * 0.5:
                        _blocked[i] = (bx, by, now_t)
                        return
                _blocked.append((p[0], p[1], now_t))
                if len(_blocked) > _BLOCK_MAX:
                    del _blocked[0]
                print("[Map] blocked point #%d at (%.2f, %.2f) - %s"
                      % (len(_blocked), p[0], p[1], reason))

            def _prune_blocked():
                cutoff = time.time() - _BLOCK_TTL
                _blocked[:] = [b for b in _blocked if b[2] >= cutoff]

            def _heading_cost(x, y, th):
                """How blocked does it look this way? Lower is better."""
                px = x + _PROBE_AHEAD * math.cos(th)
                py = y + _PROBE_AHEAD * math.sin(th)
                cost = 0.0
                for bx, by, _bt in _blocked:
                    d = math.hypot(bx - px, by - py)
                    if d < _BLOCK_RADIUS:
                        # Closer to a known blockage = worse.
                        cost += (_BLOCK_RADIUS - d) / _BLOCK_RADIUS
                return cost

            def _pick_escape_turn():
                """Choose a turn that heads somewhere we have not already been
                stopped, instead of turning a random amount and wandering back
                into the same corner."""
                _prune_blocked()
                pose = _robot_pose()
                if not pose or not _blocked:
                    # Nothing remembered: fall back to alternating turns.
                    t = _escape_dir[0] * random.uniform(1.2, 1.8)
                    _escape_dir[0] *= -1.0
                    return t
                x, y, th = pose
                best, best_cost = None, None
                # Sample turns both ways, never straight ahead (that is where we
                # just got stuck).
                for delta in (-2.6, -2.0, -1.5, -1.0, 1.0, 1.5, 2.0, 2.6):
                    c = _heading_cost(x, y, th + delta)
                    # Mild preference for a smaller turn among equally clear ones.
                    c += abs(delta) * 0.05
                    if best_cost is None or c < best_cost:
                        best, best_cost = delta, c
                print("[Map] %d blocked points known; choosing turn %+.2f rad "
                      "(cost %.2f)" % (len(_blocked), best, best_cost))
                return best

            def _planned_move(x, y, th, label, timeout=12.0):
                """Run one planned move, watching balance throughout.

                Polling only moveIsActive() would park the wander loop here for
                seconds with the stability governor not running — the robot was
                unmonitored during exactly the manoeuvre that most often put it
                over."""
                try:
                    self.conn.motion.post.moveTo(x, y, th, walk_config)
                except Exception as e:
                    print("[Move] %s failed: %s" % (label, e))
                    return
                deadline = time.time() + timeout
                while time.time() < deadline:
                    if not getattr(self, "_seeking", False): break
                    if getattr(self, "_motion_locked", False): break
                    try:
                        if not self.conn.motion.moveIsActive(): break
                    except Exception:
                        break
                    try:
                        vals = self.conn.memory.getListData(_BALANCE_KEYS)
                        tx, ty = float(vals[0]), float(vals[1])
                        mag = math.sqrt(tx * tx + ty * ty)
                        if mag >= _TILT_DANGER:
                            print("[Move] %s aborted: tilt %.3f (tiltY=%+.3f)"
                                  % (label, mag, ty))
                            break
                    except Exception:
                        pass
                    time.sleep(0.05)
                try: self.conn.motion.stopMove()
                except Exception: pass

            def _escape_stuck():
                """Jammed against something the sonar cannot see.

                Reversing is this robot's least stable direction, so it is not
                done casually — but when it is physically wedged there is no
                alternative: turning on the spot just grinds against whatever it
                is touching.  Back off a real distance, then turn hard, and
                alternate the turn direction so a failed escape is not repeated."""
                print("[Stuck] no progress - backing off and trying another way")
                self.auto_status.set("Stuck - backing out...")
                _note_blocked("stuck")
                self.say_line("stuck")
                _planned_move(-0.15, 0.0, 0.0, "escape reverse")
                time.sleep(0.4)          # let it settle before turning
                turn = _pick_escape_turn()
                print("[Stuck] turning %+.2f rad to find another route" % turn)
                _planned_move(0.0, 0.0, turn, "escape turn")
                _no_progress[0] = 0
                _phase[0]   = "settle"
                _phase_t[0] = time.time()
            _burst_gait   = bool(getattr(self, "config_burst_gait", False))
            if _burst_gait:
                print("[Gait] burst gait ENABLED: discrete moveTo steps of "
                      "%.2fm / settle >=%.1fs / resume below roll %.2f"
                      % (_STEP_M, _SETTLE_MIN_S, _RESUME_ROLL))
            else:
                print("[Gait] burst gait disabled (continuous walk)")

            # --- Active head-scanning state machine ---
            # The head cycles through 7 positions so the camera covers the whole
            # environment: left/right at three elevations, plus one floor-facing
            # slot used for boundary checks (or always active with bottom cam).
            # Yaw kept to ±0.40 rad: the head is ~0.5 kg at the top of the mass
            # column and the walk engine does not model it moving, so wide fast
            # sweeps shove the CoM sideways mid-stride.  With NAO's ~60° camera
            # FOV this still covers the room.
            _HEAD_SCAN_POSITIONS = [
                # (yaw, pitch)  — pitch: +ve = look down, −ve = look up
                ( 0.0,  0.10),   # forward, slight down
                ( 0.40, 0.05),   # left, level
                ( 0.40,-0.12),   # left, slightly up
                ( 0.0, -0.12),   # forward up (human face height)
                (-0.40, 0.05),   # right, level
                (-0.40,-0.12),   # right, slightly up
                ( 0.0,  0.42),   # forward, floor-facing (boundary check slot)
            ]
            _FLOOR_SLOT       = len(_HEAD_SCAN_POSITIONS) - 1
            _HEAD_SCAN_DWELL  = 2.5   # seconds to hold each position
            _head_scan_idx    = [0]
            _head_scan_next_t = [time.time() + 1.5]   # first move after settle
            _head_moved_t     = [time.time()]          # last time head was commanded
            _head_at_floor    = [False]                # currently in floor slot
            _head_scan_paused = [False]                # suppressed during avoidance

            # Idle chatter comes from the persona banks (backend/persona.py),
            # which shuffle rather than pick at random so the same sentence
            # does not come round twice in a row.

            # ================================================================
            while getattr(self, "_seeking", False):

                # An emergency relax must win immediately, even mid-burst.
                if getattr(self, "_motion_locked", False):
                    print("[Wander] motion locked - aborting loop.")
                    break

                # 1. SDK FACE DETECTION — runs every loop tick for any human-like
                #    target (including empty target / pure wander mode).  Much
                #    faster than Gemini; whichever fires first wins.
                if target_is_human and self.conn.memory and not _sdk_found[0]:
                    try:
                        val = self.conn.memory.getData("FaceDetected")
                        if (val and isinstance(val, list) and len(val) >= 2
                                and isinstance(val[1], list) and len(val[1]) > 0):
                            _sdk_found[0] = True
                            _task_done[0] = True   # abort any in-flight Gemini image checks
                            msg = "SDK found %s — task complete." % target_label
                            print("[WanderSeek] " + msg)
                            self.root.after(0, lambda m=msg: self.gemini_status.set(m))
                    except Exception:
                        pass

                # 2. UNIFIED FOUND: SDK spotted a human OR Gemini confirmed the target
                _any_found = _sdk_found[0] or (target and _target_found[0])
                if _any_found:
                    how          = "SDK" if _sdk_found[0] else "Gemini"
                    found_human  = _sdk_found[0] or target_is_human
                    label        = "human" if found_human else target
                    self._seeking = False
                    self.conn.motion.stopMove()
                    self.auto_status.set("Found %s! (%s)" % (label, how))

                    # --- Immediate acknowledgment: shout NOW, don't block on Gemini ---
                    # Non-blocking: plays while the robot is still moving.
                    if found_human:
                        self.say_line("found_human")
                    else:
                        self.say_line("found_target", target=target_label)

                    if found_human:
                        # Lock eyes on the face while celebrating
                        if tracker:
                            try:
                                tracker.registerTarget("Face", 0.15)
                                tracker.setMode("Head")
                                tracker.track("Face")
                            except Exception:
                                pass
                        # Celebrate immediately — dance runs alongside/after the speech above
                        self._celebrate_found_human()
                        if tracker:
                            try:
                                tracker.stopTracker()
                                tracker.unregisterAllTargets()
                            except Exception:
                                pass
                    else:
                        self._celebrate_found_object(target)

                    # --- Follow-up: ask for next instructions (Gemini or fallback) ---
                    follow_up = "What would you like me to do next?"
                    if has_api:
                        try:
                            fu_prompt = self._in_character(
                                "You just completed a search mission and found "
                                "'%s'. Ask in one short excited sentence what "
                                "you should do next." % target_label)
                            fu = search_client.generate_text(fu_prompt)
                            if fu and "Error" not in fu:
                                follow_up = fu
                        except Exception:
                            pass
                    self.say(follow_up, blocking=True)
                    break

                # 3. BACKGROUND GEMINI VISION CHECK (target mode)
                now = time.time()

                # 3a. Local novelty check (runs every second, no API)
                if (target and novelty_cam_name and wander_video
                        and _novelty_baseline[0] is not None
                        and now - _novelty_last_t[0] >= _NOVELTY_INTERVAL
                        and now - _head_moved_t[0] > 1.2):   # wait for head to settle
                    _novelty_last_t[0] = now
                    try:
                        nimg = wander_video.getImageRemote(novelty_cam_name)
                        if nimg and len(nimg) >= 7:
                            cur_zones = _zone_means(nimg[6], int(nimg[0]), int(nimg[1]))
                            score = _novelty_score(_novelty_baseline[0], cur_zones)
                            print("[Novelty] score=%.2f" % score)
                            if score >= _NOVELTY_THRESHOLD:
                                _novelty_triggered[0] = True
                                print("[Novelty] Scene change detected (%.0f%%) — triggering Gemini." % (score*100))
                            else:
                                # Slowly blend current scene into baseline
                                _novelty_baseline[0] = _novelty_blend(
                                    _novelty_baseline[0], cur_zones)
                    except Exception as e:
                        print("[Novelty] Check error: %s" % e)

                # 3b. Trigger Gemini when novelty fires or timer expires
                if target and has_api and not _vision_checking[0] and not _task_done[0]:
                    if now < _vision_backoff_until[0]:
                        remaining = int(_vision_backoff_until[0] - now)
                        self.auto_status.set("Rate limited — resuming in %ds..." % remaining)
                    else:
                        novelty_ready = (_novelty_triggered[0]
                                         and now - _last_vision_t[0] >= _NOVELTY_MIN_CALL_GAP)
                        timer_ready   = (now - _last_vision_t[0] >= _VISION_INTERVAL)
                        if novelty_ready or timer_ready:
                            _novelty_triggered[0] = False
                            _vision_checking[0] = True
                            _last_vision_t[0]   = now
                            reason = "novelty" if novelty_ready else "timer"
                            self.auto_status.set("Scanning for %s... (%s)" % (target, reason))
                            # Capture happens on the worker thread now — see
                            # _capture_and_check.  Keeping it here blindfolded
                            # the obstacle checks for ~1-2 s per vision check.
                            t = threading.Thread(target=_capture_and_check)
                            t.daemon = True
                            t.start()

                # 3.5. ACTIVE HEAD SCANNING — advance to next position every dwell period
                now = time.time()
                if (self.conn.motion and not _head_scan_paused[0]
                        and now >= _head_scan_next_t[0]):
                    slot = _head_scan_idx[0] % len(_HEAD_SCAN_POSITIONS)
                    yaw, pitch        = _HEAD_SCAN_POSITIONS[slot]
                    _head_at_floor[0]    = (slot == _FLOOR_SLOT)
                    _head_moved_t[0]     = now
                    _head_scan_next_t[0] = now + _HEAD_SCAN_DWELL
                    _head_scan_idx[0]   += 1
                    _novelty_baseline[0] = None   # reset baseline for new viewpoint
                    try:
                        # Slow (0.08) so the head glides rather than snaps —
                        # a snap is an impulse the balancer has to absorb.
                        self.conn.motion.setAngles(
                            ["HeadYaw", "HeadPitch"], [yaw, pitch], 0.08)
                    except Exception:
                        pass

                # 4. PROXIMITY SENSING — sonar + foot bumpers in ONE batched read.
                # Each separate getData is a network round trip; batching raises
                # the obstacle-check rate, and that rate is exactly what decides
                # how far the robot travels before it can react.
                l_dist = r_dist = _SONAR_CLEAR
                bumped = False
                tilt_x = tilt_y = 0.0
                fall_flag = False
                if self.conn.memory:
                    vals = None
                    try:
                        vals = self.conn.memory.getListData(_SENSE_KEYS)
                    except Exception:
                        # Older firmware may not expose getListData — fall back
                        # to individual reads rather than assuming "blocked".
                        try:
                            vals = [self.conn.memory.getData(k)
                                    for k in _SENSE_KEYS]
                        except Exception:
                            vals = None
                    if vals and len(vals) == len(_SENSE_KEYS):
                        try:
                            sonar_v = [float(v) for v in vals[:_N_SONAR]]
                            if sonar_v:
                                l_dist = sonar_v[0]
                                r_dist = sonar_v[-1]
                            # Bumpers read 1.0 when pressed.  These are the only
                            # sensors that cannot miss an obstacle — sonar is
                            # blind to anything low, thin, or soft.
                            bumped = any(
                                float(v) > 0.5
                                for v in vals[_N_SONAR:_N_SONAR + _N_BUMPER])
                            bal = vals[_N_SONAR + _N_BUMPER:]
                            if len(bal) >= 3:
                                tilt_x    = float(bal[0])
                                tilt_y    = float(bal[1])
                                fall_flag = bool(bal[2])
                            _sense_fail[0] = 0
                        except Exception:
                            vals = None
                    if not vals:
                        # FAIL SAFE, not fail open.  The old code left the
                        # distances at 1.0 ("all clear") whenever the read threw,
                        # so a flaky link looked identical to an empty room.
                        _sense_fail[0] += 1
                        if _sense_fail[0] >= 3:
                            l_dist = r_dist = 0.0      # treat as blocked
                            if _sense_fail[0] % 20 == 3:
                                print("[Sense] proximity read failing — assuming blocked.")

                # Measured on this robot: the sonar refreshes at only ~5 Hz, and
                # with an object genuinely present at 0.5 m the readings still
                # flip between the real distance and 5.0 ("no echo") from one
                # sample to the next.  So a single reading is not trustworthy —
                # take the closest value seen in a fixed time window, which
                # spans several sensor refreshes regardless of loop rate.
                _sonar_hist.append((now, min(l_dist, r_dist)))
                _cutoff = now - _SONAR_WINDOW_S
                while _sonar_hist and _sonar_hist[0][0] < _cutoff:
                    _sonar_hist.pop(0)
                near = min([d for _, d in _sonar_hist]) if _sonar_hist else _SONAR_CLEAR

                # Proximity governor: scale speed down with distance so the robot
                # is never travelling fast toward something close.  Previously the
                # check was binary at 0.60 m — full speed right up to the trigger.
                if near <= _STOP_DIST:
                    prox = 0.0
                elif near >= _SLOW_DIST:
                    prox = 1.0
                else:
                    prox = ((near - _STOP_DIST) / (_SLOW_DIST - _STOP_DIST))

                # 5. FLOOR BOUNDARY check
                # Bottom camera: runs every 5 ticks (~0.5 s) regardless of head angle.
                # Top camera:    only runs when head is in the floor-facing slot and settled.
                boundary_ok = True
                floor_tick += 1
                _do_floor_check = (use_boundary and floor_tick >= 5 and (
                    _floor_bottom_cam or
                    (_head_at_floor[0] and now - _head_moved_t[0] > 0.8)))
                if _do_floor_check:
                    floor_tick = 0
                    boundary_ok, _ = self._check_floor_boundary(
                        floor_cam_name, wander_video, green_threshold=floor_threshold)

                if not getattr(self, "_seeking", False):
                    break

                # 6. MOVEMENT DECISION — every sensor value below came from the
                # single batched read above, so this costs no extra round trips.
                now = time.time()
                tilt_mag  = math.sqrt(tilt_x * tilt_x + tilt_y * tilt_y)
                tilt_high = tilt_mag >= 0.50
                if fall_flag:
                    if fall_flag_since is None:
                        fall_flag_since = now
                else:
                    fall_flag_since = None

                if tilt_high:
                    if tilt_high_since is None:
                        tilt_high_since = now
                else:
                    tilt_high_since = None

                if fall_flag and tilt_high and tilt_high_since is not None and (now - tilt_high_since) >= fall_debounce_s:
                    try:
                        self.conn.motion.stopMove()
                    except Exception:
                        pass
                    self.auto_status.set("Fallen - motors released.")
                    self._set_status(
                        "Fall detected. Motors released - please help it up.", False)
                    # Odometry does not survive a fall (and the robot gets picked
                    # up and put down somewhere else), so every remembered
                    # coordinate is now meaningless. Better no map than a wrong one.
                    if _blocked:
                        print("[Map] discarding %d blocked points - odometry "
                              "reference lost in the fall" % len(_blocked))
                        _blocked[:] = []
                    # Do NOT try to sit or recover.  This used to run
                    # goToPosture("Sit") straight after a fall, which is a fresh
                    # full-body motion on a robot that has just hit the floor —
                    # it looked exactly like the robot ignoring Relax.  A robot
                    # that cannot get up unaided should go limp and wait for a
                    # human, not thrash.
                    self._emergency_relax()
                    self.say_line("fell")
                    self._seeking = False
                    break

                # 6a-i. ROLL DIVERGENCE — the measured failure mode is lateral
                # oscillation growing cycle over cycle until the robot goes over
                # sideways.  Watching the peak roll across a window catches that
                # build-up while it can still be damped by slowing down.
                _roll_hist.append((now, abs(tilt_x)))
                _rcut = now - _ROLL_WINDOW
                while _roll_hist and _roll_hist[0][0] < _rcut:
                    _roll_hist.pop(0)
                roll_peak = max([r for _, r in _roll_hist]) if _roll_hist else 0.0
                if roll_peak >= _ROLL_MAX:
                    roll_ok = 0.0
                elif roll_peak <= _ROLL_WARN:
                    roll_ok = 1.0
                else:
                    roll_ok = 1.0 - ((roll_peak - _ROLL_WARN) /
                                     (_ROLL_MAX - _ROLL_WARN))

                # Periodic trace so a fall can be reconstructed afterwards.
                # tiltX is roll (topple sideways), tiltY is pitch (nose-over).
                if now - _last_trace[0] >= 0.5:
                    _last_trace[0] = now
                    _robot_pose()      # keeps the map trail and marker current
                    # tiltmag is what the stability governor actually acts on,
                    # and phase/stepping matters more than the old fwd/turn
                    # columns, which are always zero under discrete stepping.
                    print("[Trace] tiltX=%+.3f tiltY=%+.3f tiltmag=%.3f "
                          "rollpk=%.3f near=%.2f %s" % (
                              tilt_x, tilt_y,
                              math.sqrt(tilt_x * tilt_x + tilt_y * tilt_y),
                              roll_peak, near,
                              ("stepping" if (_burst_gait and _phase[0] == "walk"
                                              and _step_issued[0])
                               else ("settling" if _burst_gait
                                     and _phase[0] == "settle" else "walking"))))

                # 6a. STABILITY GOVERNOR — scale the walk down as the torso tilts,
                # and stop outright before the tilt becomes unrecoverable.
                if tilt_mag >= _TILT_DANGER:
                    stab = 0.0
                elif tilt_mag <= _TILT_WARN:
                    stab = 1.0
                else:
                    stab = 1.0 - ((tilt_mag - _TILT_WARN) /
                                  (_TILT_DANGER - _TILT_WARN))

                if stab <= 0.0:
                    # Off balance: stop, hold still, let the balancer catch up.
                    if not _braking[0]:
                        _braking[0] = True
                        print("[Stability] tilt=%.3f rad — braking to recover." % tilt_mag)
                        self.auto_status.set("Steadying...")
                    try: self.conn.motion.stopMove()
                    except Exception: pass
                    _cur_fwd[0] = _cur_turn[0] = 0.0
                    walk_start_t = time.time()      # re-ramp gently on resume
                    time.sleep(0.1)
                    continue
                elif _braking[0] and tilt_mag <= _TILT_WARN:
                    _braking[0] = False
                    self.auto_status.set("Recovered — resuming.")

                # 6b. BURST GAIT — discrete planned steps instead of chopping up
                # a continuous walk.
                #
                # moveToward() has a startup phase: the engine shifts weight and
                # sets up the ZMP for roughly a second before a foot ever leaves
                # the ground.  Cutting a burst at ~1s therefore stopped it mid
                # preparation every cycle — the robot rocked its hips, got reset,
                # and prepared again, managing about two actual steps in thirty
                # seconds.  Lengthening the burst is not an option either, since
                # roll is already ~0.16 by t=2s on this robot.
                #
                # moveTo() avoids the whole problem: the engine plans a complete
                # footstep sequence for a fixed distance and ends it in stable
                # double support.  One call is one self-contained little walk.
                if _burst_gait:
                    if _phase[0] == "settle":
                        settled_for = now - _phase_t[0]
                        ready = (settled_for >= _SETTLE_MIN_S
                                 and roll_peak <= _RESUME_ROLL)
                        if ready or settled_for >= _SETTLE_MAX_S:
                            _phase[0]      = "walk"
                            _phase_t[0]    = now
                            _step_issued[0] = False
                        else:
                            # Hold still and let it damp.  Also the safest moment
                            # to look around, so the head scan runs here rather
                            # than mid-stride.
                            try: self.conn.motion.stopMove()
                            except Exception: pass
                            _head_scan_paused[0] = False
                            self.auto_status.set(
                                "Steadying (roll %.2f)..." % roll_peak)
                            time.sleep(0.05)
                            continue

                    # --- walk phase: one discrete moveTo per burst ---
                    _head_scan_paused[0] = True
                    if not _step_issued[0]:
                        if bumped or near <= _STOP_DIST or not boundary_ok:
                            # Blocked. Settle, and deliberately do NOT continue —
                            # fall through to the obstacle handling below so the
                            # back-up-and-pivot manoeuvre still runs.
                            _phase[0]   = "settle"
                            _phase_t[0] = now
                        else:
                            theta = wander_turn_bias * _STEP_TURN
                            try:
                                # post.* so the loop keeps polling sensors while
                                # the step executes.
                                _step_pos0[0] = _robot_xy()
                                self.conn.motion.post.moveTo(
                                    _STEP_M, 0.0, theta, walk_config)
                                _step_issued[0] = True
                                _phase_t[0]     = now
                                # Log the attitude each step launches from.
                                # Measured runs start anywhere from tiltY=-0.03
                                # to +0.19 for the same posture command, and
                                # that starting posture appears to matter more
                                # than anything tunable in here.
                                print("[Step] moveTo(%.2fm, theta=%+.2f) from "
                                      "tiltX=%+.3f tiltY=%+.3f rollpk=%.3f"
                                      % (_STEP_M, theta, tilt_x, tilt_y,
                                         roll_peak))
                            except Exception as e:
                                print("[Step] moveTo failed: %s" % e)
                                _phase[0]   = "settle"
                                _phase_t[0] = now
                            time.sleep(0.05)
                            continue
                    else:
                        elapsed = now - _phase_t[0]
                        try:
                            still_moving = self.conn.motion.moveIsActive()
                        except Exception:
                            still_moving = False
                        # Do NOT interrupt a planned step just because roll rose.
                        # moveTo ends in stable double support; stopMove() in the
                        # middle of a stride leaves the robot on one foot, which
                        # is the worst moment to freeze it.  The measured runs
                        # show roll peaking ~0.12-0.16 during a normal step and
                        # settling straight back afterwards, so let the step
                        # finish and take it out in the settle phase.  Only a
                        # genuine divergence (_ROLL_MAX) or a physical obstacle
                        # is worth the risk of stopping mid-stride.
                        abort = (roll_peak >= _ROLL_MAX or bumped
                                 or near <= _STOP_DIST)
                        if abort:
                            print("[Step] aborting step (rollpk=%.3f near=%.2f "
                                  "bumped=%s)" % (roll_peak, near, bumped))
                            try: self.conn.motion.stopMove()
                            except Exception: pass
                            _phase[0]   = "settle"
                            _phase_t[0] = now
                        elif (not still_moving) or elapsed > _STEP_TIMEOUT:
                            if elapsed > _STEP_TIMEOUT:
                                print("[Step] timed out after %.1fs" % elapsed)
                                try: self.conn.motion.stopMove()
                                except Exception: pass
                            # Did it actually go anywhere?  Odometry is the only
                            # honest answer: wedged against a wall the engine
                            # happily marches on the spot and sonar sees nothing.
                            p1 = _robot_xy()
                            if p1 and _step_pos0[0]:
                                dx = p1[0] - _step_pos0[0][0]
                                dy = p1[1] - _step_pos0[0][1]
                                moved = math.sqrt(dx * dx + dy * dy)
                                if moved < _STEP_M * _NO_PROGRESS_FRAC:
                                    _no_progress[0] += 1
                                    print("[Step] moved only %.3fm of %.2fm "
                                          "- stalled (%d/%d)"
                                          % (moved, _STEP_M, _no_progress[0],
                                             _NO_PROGRESS_HITS))
                                else:
                                    _no_progress[0] = 0
                            _phase[0]   = "settle"
                            _phase_t[0] = now
                            if _no_progress[0] >= _NO_PROGRESS_HITS:
                                _escape_stuck()
                        self.auto_status.set("Stepping (roll %.2f)..." % roll_peak)
                        time.sleep(0.05)
                        continue

                # With burst gait on, the head only sweeps during the settle
                # phase, so it never moves mid-stride — it is ~0.5 kg at the top
                # of the mass column and the walk engine does not model it
                # moving.  Without burst gait there is no settle phase, so it
                # scans while walking as it always did.
                if _burst_gait:
                    _head_scan_paused[0] = True

                obstacle = bumped or (near <= _STOP_DIST) or not boundary_ok

                if obstacle:
                    _head_scan_paused[0] = True
                    # Center head forward so sonar/camera see the obstacle clearly
                    try:
                        if self.conn.motion:
                            self.conn.motion.setAngles(
                                ["HeadYaw", "HeadPitch"], [0.0, 0.1], 0.25)
                    except Exception:
                        pass
                    self.conn.motion.stopMove()
                    if bumped:
                        reason_txt = "bumper"
                    elif not boundary_ok:
                        reason_txt = "boundary"
                    else:
                        reason_txt = "sonar %.2fm" % near
                    print("[Avoid] %s — backing off." % reason_txt)
                    self.auto_status.set("Avoiding (%s)..." % reason_txt)
                    if bumped:
                        self.say_line("avoid_bumper")
                    elif not boundary_ok:
                        self.say_line("avoid_boundary")
                    else:
                        self.say_line("avoid_sonar")

                    # Avoidance uses discrete planned moves for the same reason
                    # the stepping does: moveToward spends ~1s shifting weight
                    # before it steps, so driving it in short timed bursts just
                    # rocks the hips and never completes a stride.  That is what
                    # made the avoidance look panicky and jerky.  moveTo plans
                    # the whole manoeuvre and finishes in stable double support.
                    # Only reverse when something was physically touched.
                    #
                    # Backwards is NAO's least stable direction, and this robot
                    # already stands with a ~9 degree forward lean, so reversing
                    # is the worst thing to ask of it — a measured run pitched it
                    # backward (tiltY -0.37 -> -0.94) and over, while roll stayed
                    # at a healthy 0.03.  Sonar detections have metres of room, so
                    # just turn away instead: turning on the spot keeps both feet
                    # under the body and needs no backward weight transfer.
                    if bumped:
                        print("[Avoid] bumper contact - reversing 6cm")
                        _planned_move(-0.06, 0.0, 0.0, "back up")
                    else:
                        print("[Avoid] turning away without reversing "
                              "(sonar %.2fm)" % near)

                    # Remember this spot, then turn away by a real angle (radians
                    # for moveTo, not a normalised fraction) so it actually
                    # clears instead of nudging a few degrees.
                    _note_blocked(reason_txt)
                    _prune_blocked()
                    pose = _robot_pose()
                    if pose and len(_blocked) > 1:
                        # Prefer a heading with no remembered blockage in it,
                        # while still respecting which side the sonar saw.
                        x, y, th = pose
                        prefer = -1.0 if l_dist < r_dist else 1.0
                        best, best_cost = None, None
                        for mag in (0.7, 1.0, 1.4, 2.0):
                            for sign in (prefer, -prefer):
                                d = sign * mag
                                c = _heading_cost(x, y, th + d) + abs(d) * 0.05
                                if sign != prefer:
                                    c += 0.15      # mild tiebreak toward the open side
                                if best_cost is None or c < best_cost:
                                    best, best_cost = d, c
                        turn_rad = best
                    else:
                        turn_rad = random.uniform(0.6, 1.0)      # ~35-57 degrees
                        if not boundary_ok:
                            turn_rad *= random.choice([-1, 1])
                        elif l_dist < r_dist:
                            turn_rad = -abs(turn_rad)        # obstacle left -> turn right
                        else:
                            turn_rad = abs(turn_rad)
                        if random.random() < 0.20:
                            turn_rad *= -1.0                 # occasional other way
                    print("[Avoid] turning %+.2f rad" % turn_rad)
                    _planned_move(0.0, 0.0, turn_rad, "pivot")

                    # Resume from a standstill so the ramp/smoothing start clean,
                    # and settle before walking again — the back-up and pivot are
                    # themselves walking, so they build up the same wobble.
                    _cur_fwd[0] = _cur_turn[0] = 0.0
                    walk_start_t = time.time()
                    _phase[0]    = "settle"
                    _phase_t[0]  = time.time()

                    # Resume scanning from current position slot
                    _head_scan_paused[0] = False
                    _head_scan_next_t[0] = time.time() + 0.5   # short settle before next move

                else:
                    # Normal wander movement
                    # Occasional speech
                    if time.time() > next_speech_t:
                        if not target:
                            bank = "walking"
                        elif target_is_human:
                            bank = "searching_human"
                        else:
                            bank = "searching"
                        self.say_line(bank, target=target_label)
                        next_speech_t = time.time() + random.uniform(8.0, 25.0)

                    # Gradual ramp to full speed to avoid sudden jolts
                    ramp = min(1.0, max(0.0, (time.time() - walk_start_t) / 2.0))

                    # Drift change every 8-14 s.  Every turn reversal perturbs the
                    # robot laterally, and the measured roll build-up tracked the
                    # turn oscillation — so hold a heading for longer, and drop
                    # the sinusoid that used to layer on top of the random bias.
                    if time.time() - last_bias_t > random.uniform(8.0, 14.0):
                        wander_turn_bias = random.uniform(-0.10, 0.10)
                        last_bias_t = time.time()

                    # Forward speed: a real walk or a full stop, never the shuffle
                    # band in between.
                    scale   = ramp * stab * prox * roll_ok
                    tgt_fwd = max_fwd * scale
                    if tgt_fwd < _MIN_WALK:
                        tgt_fwd = 0.0 if scale < 0.45 else _MIN_WALK

                    if tgt_fwd <= 0.0:
                        # Too tight ahead to walk properly: rotate on the spot to
                        # look for a clear heading rather than creeping forward.
                        # A pure rotation is a clean, stable gait — what toppled
                        # the robot was mixing a turn into near-zero translation.
                        tgt_turn = (0.45 if wander_turn_bias >= 0 else -0.45) * roll_ok
                    else:
                        tgt_turn = wander_turn_bias * stab * roll_ok

                    # Ease toward the target so the walk engine sees a smooth
                    # velocity ramp instead of a new setpoint every 100 ms.
                    _cur_fwd[0]  += (tgt_fwd  - _cur_fwd[0])  * _CMD_SMOOTH
                    _cur_turn[0] += (tgt_turn - _cur_turn[0]) * _CMD_SMOOTH

                    self.conn.motion.moveToward(
                        float(_cur_fwd[0]), 0.0, float(_cur_turn[0]), walk_config)

                # 50 ms, not 100.  Batching the sensor reads cut the per-tick
                # cost from ~38 ms to ~7 ms (measured), so there is headroom —
                # and the bumpers are a hardware switch, so polling twice as
                # often halves how far the robot pushes before it reacts.
                time.sleep(0.05)

        except Exception as e:
            self._set_status("Wander error: " + str(e)[:60], False)
            self.auto_status.set("Error: see console")
            print("[Wander] Error: " + str(e))

        finally:
            self._seeking = False
            locked = getattr(self, "_motion_locked", False)
            self.auto_status.set("Relaxed." if locked else "Idle.")
            # If the user hit Relax, do NOT touch the motors on the way out —
            # a parting stopMove/setAngles would re-stiffen the neck on a robot
            # that was just deliberately released.
            if not locked:
                try: self.conn.motion.stopMove()
                except Exception: pass
                try:
                    if self.conn.motion:
                        self.conn.motion.setAngles("HeadPitch", 0.0, 0.1)
                except Exception: pass
            if sonar:
                try: sonar.unsubscribe("WanderSeeker")
                except Exception: pass
            if self.conn.face:
                try: self.conn.face.unsubscribe("WanderSeekerFace")
                except Exception: pass
            if floor_cam_name and wander_video:
                try: wander_video.unsubscribe(floor_cam_name)
                except Exception: pass
            if novelty_cam_name and wander_video:
                try: wander_video.unsubscribe(novelty_cam_name)
                except Exception: pass

    def _celebrate_found_human(self):
        """High-five gesture, then sit."""
        import time
        if not self._motion_allowed():
            return
        motion = self.conn.motion
        leds   = self.conn.leds
        if leds:
            try: leds.fadeRGB("AllLeds", 0x00FF8C00, 0.3)
            except Exception: pass
        if motion:
            try:
                # Ensure stiffness is on before moving (stopMove may have relaxed walk engine)
                motion.setStiffnesses(["Body"], [1.0])
                time.sleep(0.2)
                # Raise right arm high for high-five
                motion.setAngles(
                    ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw",
                     "RWristYaw"],
                    [-1.25, -0.20, 0.03, 1.20, 0.0],
                    0.25)
                time.sleep(2.5)
                # Lower arm back down
                motion.setAngles(
                    ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw",
                     "RWristYaw"],
                    [1.50, -0.15, 0.85, 1.20, 0.0],
                    0.20)
                time.sleep(0.5)
            except Exception as e:
                print("[Celebrate] high-five error: %s" % e)
        self._sit_after_task()

    def _celebrate_found_object(self, target_name):
        """Victory arm-wave, then sit."""
        import time
        if not self._motion_allowed():
            return
        motion = self.conn.motion
        leds   = self.conn.leds
        if leds:
            try: leds.fadeRGB("AllLeds", 0x0000FF00, 0.3)
            except Exception: pass
        if motion:
            try:
                # Ensure stiffness is on before moving
                motion.setStiffnesses(["Body"], [1.0])
                time.sleep(0.2)
                # Victory wave: alternate arm raises
                for _ in range(2):
                    motion.setAngles(
                        ["LShoulderPitch", "RShoulderPitch",
                         "LShoulderRoll",  "RShoulderRoll"],
                        [0.0, -1.40, 0.40, -0.40],
                        0.30)
                    time.sleep(0.55)
                    motion.setAngles(
                        ["LShoulderPitch", "RShoulderPitch",
                         "LShoulderRoll",  "RShoulderRoll"],
                        [-1.40, 0.0, 0.40, -0.40],
                        0.30)
                    time.sleep(0.55)
                motion.setAngles(
                    ["LShoulderPitch", "RShoulderPitch",
                     "LShoulderRoll",  "RShoulderRoll"],
                    [1.50, 1.50, 0.15, -0.15],
                    0.20)
                time.sleep(0.5)
            except Exception as e:
                print("[Celebrate] object-dance error: %s" % e)
        self._sit_after_task()

    def _motion_allowed(self):
        """False once an emergency relax has locked motion.

        Anything that starts a new motion must check this, or it will fight a
        deliberate Relax — which is exactly what a fallen robot must not do."""
        return not getattr(self, "_motion_locked", False)

    def _sit_after_task(self):
        """Sit down once a wander/seek task is complete. The full dance
        sequence is riskier (fall-prone) and is now opt-in only via the
        'Dance' button in the Posture card, rather than running automatically."""
        if not self._motion_allowed():
            print("[Sit] skipped: motion locked (relax).")
            return
        if self.conn.posture:
            try:
                self.conn.posture.goToPosture("Sit", 0.5)
            except Exception:
                try:
                    self.conn.posture.goToPosture("SitRelax", 0.5)
                except Exception as e:
                    print("[Celebrate] sit error: %s" % e)

    def _has_fallen(self):
        if not self.conn.memory:
            return False
        try:
            return bool(self.conn.memory.getData("robotHasFallen"))
        except Exception:
            return False

    def _tilt_too_high(self, threshold=0.50):
        if not self.conn.memory:
            return False
        try:
            ax = float(self.conn.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value"))
            ay = float(self.conn.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"))
            return math.sqrt(ax * ax + ay * ay) >= threshold
        except Exception:
            return False

    @staticmethod
    def _check_floor_boundary(floor_cam_name, video_proxy,
                               green_threshold=0.25, bottom_fraction=0.50):
        """Get one frame from the floor camera and check if the bottom half of
        the image still looks like it did during calibration.

        Returns (is_ok: bool, ratio: float|None)
          is_ok  – True = floor looks normal, False = boundary detected.
          ratio  – raw fraction of pixels that match the floor heuristic,
                   or None if no frame was available.
        """
        if not floor_cam_name or not video_proxy:
            return True, None
        try:
            img_data = video_proxy.getImageRemote(floor_cam_name)
            if not img_data or len(img_data) < 7:
                return True, None
            w   = int(img_data[0])
            h   = int(img_data[1])
            raw = img_data[6]
            data     = bytearray(raw) if not isinstance(raw, bytearray) else raw
            row_size = w * 3
            start_row = int(h * (1.0 - bottom_fraction))

            green_hits = 0
            total      = 0
            step       = 4
            for y in range(start_row, h):
                base = y * row_size
                for x in range(0, w, step):
                    idx = base + x * 3
                    if idx + 2 >= len(data):
                        break
                    r, g, b = data[idx], data[idx + 1], data[idx + 2]
                    # Broad green heuristic: green channel clearly dominant.
                    # Intentionally loose so it works under varied lighting.
                    if g > 50 and (g - r) > 15 and (g - b) > 10:
                        green_hits += 1
                    total += 1
            if total == 0:
                return True, None
            ratio = float(green_hits) / total
            print("[Floor] ratio=%.2f threshold=%.2f" % (ratio, green_threshold))
            return ratio >= green_threshold, ratio
        except Exception as e:
            print("[Floor] check error: %s" % e)
            return True, None


    def _on_camera_start(self):
        if not self._require_connection(): return
        self._set_controller_details(False)
        # When the camera sees a person, trigger the celebration dance
        self.vision.ui["on_human_detected"] = self._on_camera_person_detected
        self.vision.start_camera(self.conn.ip, self.conn.port, self.root.after)
        self._poll_camera()

    def _on_camera_person_detected(self):
        """Called by VisionManager the first time a human is spotted via live camera."""
        if not self._require_connection():
            return
        t = threading.Thread(target=self._celebrate_found_human)
        t.daemon = True
        t.start()
        
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

