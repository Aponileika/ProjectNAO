# -*- coding: utf-8 -*-
import os
import sys
import subprocess

if sys.version_info[0] != 2:
    _base = os.path.dirname(os.path.abspath(__file__))
    _root = os.path.dirname(os.path.dirname(_base))
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
            [_bundled_py, __file__] + sys.argv[1:],
            env=_env)
        sys.exit(_rc)

from nao_app.backend.nao_connection import NaoConnectionManager
from nao_app.backend.vision import VisionManager
from nao_app.backend.controller import GamepadController
from nao_app.ui.app_window import NaoAppWindow
import Tkinter as tk
from nao_app.ui.widgets import SUCCESS, ERROR, WARN, FG

def main():
    conn = NaoConnectionManager()
    vision = VisionManager(conn)
    controller = GamepadController(conn)
    
    app = NaoAppWindow(conn, vision, controller)
    app.vision_after_id = None
    
    # wire ui callbacks
    vision.ui = {
        "set_status": app._set_status,
        "on_start": lambda: app.cam_status_var.set("Starting..."),
        "on_stop": lambda: app.cam_status_var.set("Stopped"),
        "update_frame": lambda photo, status: _update_frame(app, photo, status),
        "update_detection": lambda msg, level: _update_detection(app, msg, level),
        "store_after_id": lambda after_id: _store_after_id(app, after_id)
    }
    
    controller.ui = {
        "set_status": app._set_status,
        "on_detect": app._on_controller_detect_cb,
        "on_stop": lambda: app.ctrl_axes_var.set("Fwd: 0.00   Rot: 0.00")
    }
    
    app.run()

def _update_frame(app, photo, status):
    if photo:
        app.cam_preview_lbl.config(image=photo)
    else:
        app.cam_preview_lbl.config(text="No feed", image="")
    app.cam_status_var.set(status)

def _update_detection(app, msg, level_str):
    app.cam_detect_var.set(msg)
    color = FG
    if level_str == "success": color = SUCCESS
    elif level_str == "error": color = ERROR
    elif level_str == "warn": color = WARN
    app.cam_detect_lbl.config(fg=color)

def _store_after_id(app, after_id):
    app.vision_after_id = after_id

if __name__ == "__main__":
    main()
