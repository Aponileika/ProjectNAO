#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Entry point for the NAO Control Panel.

Run with any Python version - this script will automatically relaunch itself
under the bundled Python 2.7 if needed (required for pynaoqi / NAOqi SDK).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import py27
py27.relaunch_if_needed(__file__)

from nao_app.main import main

if __name__ == "__main__":
    main()
