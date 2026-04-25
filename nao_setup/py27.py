"""
py27.py - shared Python 2.7 auto-relaunch helper for NAOqi scripts.

Usage at the top of any entry-point script (before NAOqi imports):

    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import py27; py27.relaunch_if_needed(__file__)

If the current interpreter is Python 3, this module re-executes the calling
script under the bundled Python 2.7 (../Python/python.exe) with the pynaoqi
SDK on PYTHONPATH, then calls sys.exit() so the Python 3 process terminates.
If the interpreter is already Python 2 it does nothing.
"""
import os
import sys
import subprocess

_BUNDLED_PY_REL  = os.path.join('Python', 'python.exe')
_BUNDLED_LIB_REL = 'pynaoqi-python2.7-2.8.6.23-win64-vs2015-20191127_152649'


def _find_root(start_dir):
    """Walk upwards from start_dir until we find the directory that contains
    Python/ (the bundled runtime root), or give up after 6 levels."""
    current = os.path.abspath(start_dir)
    for _ in range(6):
        if os.path.isdir(os.path.join(current, 'Python')):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return None


def relaunch_if_needed(script_file):
    """Re-execute *script_file* under the bundled Python 2.7 if we are
    currently running under Python 3.  Exits the current process when done."""
    if sys.version_info[0] == 2:
        return

    root = _find_root(os.path.dirname(os.path.abspath(script_file)))
    if root is None:
        return

    bundled_py  = os.path.join(root, _BUNDLED_PY_REL)
    bundled_lib = os.path.join(root, _BUNDLED_LIB_REL, 'lib')

    if not os.path.isfile(bundled_py):
        return

    env = os.environ.copy()
    env['PYTHONPATH'] = bundled_lib
    env['PATH'] = bundled_lib + os.pathsep + env.get('PATH', '')
    env['NAO_HOST_PYTHON'] = sys.executable  # let Python 2.7 find us for Pillow subprocesses

    rc = subprocess.call(
        [bundled_py, os.path.abspath(script_file)] + sys.argv[1:],
        env=env)
    sys.exit(rc)
