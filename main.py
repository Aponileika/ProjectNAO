#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Start the NAO Control Panel with its output where you can see it.

Run this and nothing else:

    python main.py

Works under any Python.  The app itself must run under the bundled Python 2.7
because pynaoqi requires it, so this script checks the pieces are in place,
launches the app under that interpreter, and mirrors everything it prints to
both the terminal and a timestamped file in logs/.

The logging is the point.  This robot's falls are diagnosed from the console
trace - which gait was active, the per-step moveTo calls, the roll values just
before it went over - and that trace used to scroll away into a terminal that
nobody kept.  Now every run leaves logs/nao_<date>_<time>.log behind, with a
wall-clock stamp on every line so a fall can be matched to what was printed.

It also catches the failure mode that has no traceback: a Tk call from a worker
thread kills python27.dll with an access violation, and the only symptom is
output stopping mid-line.  If the app exits without a Python error, this says
so explicitly instead of leaving a silent terminal.
"""

import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))

BUNDLED_PY  = os.path.join(ROOT, "Python", "python.exe")
PYNAOQI_DIR = os.path.join(ROOT, "pynaoqi-python2.7-2.8.6.23-win64-vs2015-20191127_152649")
ENTRY       = os.path.join(ROOT, "nao_setup", "nao_settings.py")
LOG_DIR     = os.path.join(ROOT, "logs")

OLLAMA_HOST = "127.0.0.1:11434"
LOCAL_MODEL = "qwen2.5:3b-instruct"


def _say(msg):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


# ----------------------------------------------------------------------
# Pre-flight
# ----------------------------------------------------------------------

def _check_ollama():
    """Report the local model server without requiring it.

    Wander speech falls back to the phrase banks when the model is missing, so
    this is information, never a reason to refuse to start."""
    try:
        try:
            from urllib.request import urlopen          # Python 3
        except ImportError:
            from urllib2 import urlopen                 # Python 2
        import json
        raw = urlopen("http://%s/api/tags" % OLLAMA_HOST, timeout=3).read()
        if not isinstance(raw, str):
            raw = raw.decode("utf-8", "ignore")
        names = [m.get("name", "") for m in json.loads(raw).get("models", [])]
        if not names:
            return "running, but no models pulled"
        if any(n == LOCAL_MODEL or n.split(":")[0] == LOCAL_MODEL.split(":")[0]
               for n in names):
            return "ready (%s)" % LOCAL_MODEL
        return "running, but %s is not pulled (have: %s)" % (
            LOCAL_MODEL, ", ".join(names)[:50])
    except Exception:
        return "not running - wander speech will use phrase banks"


def preflight():
    """Check what the app needs.  Returns False only for fatal problems."""
    ok = True
    _say("NAO Control Panel")
    _say("-" * 60)

    if os.path.isfile(BUNDLED_PY):
        _say("  bundled Python 2.7 : found")
    else:
        _say("  bundled Python 2.7 : MISSING at %s" % BUNDLED_PY)
        ok = False

    if os.path.isdir(PYNAOQI_DIR):
        _say("  pynaoqi SDK        : found")
    else:
        _say("  pynaoqi SDK        : MISSING at %s" % PYNAOQI_DIR)
        ok = False

    if os.path.isfile(ENTRY):
        _say("  entry point        : found")
    else:
        _say("  entry point        : MISSING at %s" % ENTRY)
        ok = False

    conf = os.path.join(ROOT, "config.json")
    if os.path.isfile(conf):
        try:
            import json
            with open(conf) as fh:
                data = json.load(fh)
            _say("  config.json        : ok (ip %s, burst_gait %s)" % (
                data.get("ip", "?"), data.get("burst_gait", False)))
        except Exception:
            _say("  config.json        : PRESENT BUT UNREADABLE")
    else:
        _say("  config.json        : missing (app will use defaults)")

    _say("  local model        : %s" % _check_ollama())
    _say("-" * 60)
    return ok


# ----------------------------------------------------------------------
# Launch
# ----------------------------------------------------------------------

def run():
    if not preflight():
        _say("\nCannot start: something above is missing.")
        return 1

    if not os.path.isdir(LOG_DIR):
        os.makedirs(LOG_DIR)
    log_path = os.path.join(
        LOG_DIR, "nao_%s.log" % time.strftime("%Y%m%d_%H%M%S"))

    env = os.environ.copy()
    lib = os.path.join(PYNAOQI_DIR, "lib")
    env["PYTHONPATH"]       = lib
    env["PATH"]             = lib + os.pathsep + env.get("PATH", "")
    env["NAO_HOST_PYTHON"]  = sys.executable
    env["PYTHONUNBUFFERED"] = "1"

    _say("logging to %s" % log_path)
    _say("press Ctrl+C to stop\n")

    # -u so the child does not sit on a block buffer: a log that arrives in
    # 8KB chunks is useless for watching a robot walk, and worse than useless
    # if the process dies with a partial buffer unwritten.
    # universal_newlines puts the pipe in text mode, which is what makes
    # bufsize=1 mean line buffering; in binary mode Python 3 ignores it and
    # warns, and the log arrives in blocks instead of lines.
    proc = subprocess.Popen(
        [BUNDLED_PY, "-u", ENTRY],
        cwd=ROOT, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        universal_newlines=True, bufsize=1)

    saw_output = False
    started = time.time()
    log = open(log_path, "w")
    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                break
            if not isinstance(line, str):
                line = line.decode("utf-8", "ignore")
            line = line.rstrip("\r\n")
            saw_output = True
            stamped = "%s  %s" % (time.strftime("%H:%M:%S"), line)
            _say(stamped)
            log.write(stamped + "\n")
            log.flush()          # a crash must not cost us the last lines
        proc.wait()
    except KeyboardInterrupt:
        _say("\nstopping...")
        try:
            proc.terminate()
        except Exception:
            pass
        proc.wait()
    finally:
        log.close()

    _report_exit(proc.returncode, saw_output, time.time() - started, log_path)
    return proc.returncode or 0


def _report_exit(code, saw_output, elapsed, log_path):
    _say("")
    _say("-" * 60)
    _say("app exited after %.0f seconds (code %s)" % (elapsed, code))

    # Windows reports an access violation as 0xC0000005, which Python surfaces
    # as a large unsigned or negative return code.  That is the signature of
    # Tk being touched from a worker thread - no traceback, output just stops.
    if code not in (0, 1, None) and abs(code) > 1000:
        _say("")
        _say("  That exit code is a native crash, not a Python error.")
        _say("  If the log stops mid-line with no traceback, it is almost")
        _say("  certainly a Tk call from a worker thread.  Check:")
        _say("    Get-WinEvent -FilterHashtable @{LogName='Application';"
             "ProviderName='Application Error'} -MaxEvents 5")
    elif not saw_output:
        _say("  The app produced no output at all - it failed before start.")

    _say("  log: %s" % log_path)


if __name__ == "__main__":
    sys.exit(run())
