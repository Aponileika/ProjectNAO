# -*- coding: utf-8 -*-
import Tkinter as tk
import tkFont

BG       = "#1e1e2e"
FG       = "#cdd6f4"
ACCENT   = "#89b4fa"
SUCCESS  = "#a6e3a1"
ERROR    = "#f38ba8"
WARN     = "#fab387"
CARD_BG  = "#313244"
BTN_BG   = "#45475a"
BTN_FG   = "#cdd6f4"

def make_card(parent, title, font_head=None):
    if font_head is None:
        font_head = tkFont.Font(family="Segoe UI", size=11, weight="bold")
    card = tk.LabelFrame(
        parent, text="  " + title + "  ",
        font=font_head, bg=CARD_BG, fg=ACCENT,
        bd=1, relief="groove", padx=10, pady=8)
    card.pack(fill="x", pady=4)
    return card

def make_btn(parent, text, command, width=14, font_norm=None):
    if font_norm is None:
        font_norm = tkFont.Font(family="Segoe UI", size=10)
    return tk.Button(
        parent, text=text, command=command, width=width,
        font=font_norm, bg=BTN_BG, fg=BTN_FG,
        activebackground=ACCENT, activeforeground=BG,
        relief="flat", cursor="hand2")
