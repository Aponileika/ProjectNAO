#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import subprocess

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
            [_bundled_py, __file__] + sys.argv[1:],
            env=_env)
        sys.exit(_rc)

from nao_app.main import main

if __name__ == "__main__":
    main()
