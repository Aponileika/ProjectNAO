# -*- coding: utf-8 -*-
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
