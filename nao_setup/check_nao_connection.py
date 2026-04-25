#!/usr/bin/env python
"""Simple NAO connection checker.

Usage:
  python check_nao_connection.py <NAO_IP> [port] [--no-speech]

Checks TCP reachability to port 9559 (default) and attempts to create
an ALProxy to verify NAOqi connectivity.
"""
import os
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import py27
py27.relaunch_if_needed(__file__)

import traceback

IMPORT_EXCEPTION = None
try:
    from naoqi import ALProxy
except Exception:
    IMPORT_EXCEPTION = traceback.format_exc()
    ALProxy = None


def tcp_check(ip, port=9559, timeout=3):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        s.close()
        return True
    except Exception:
        return False


def naoqi_check(ip, port=9559):
    if ALProxy is None:
        msg = "pynaoqi (naoqi) not installed or import failed"
        if IMPORT_EXCEPTION:
            first_line = IMPORT_EXCEPTION.splitlines()[-1]
            msg = msg + ": " + first_line
        return False, msg
    try:
        proxy = ALProxy("ALSystem", ip, port)
        return True, "ALProxy created"
    except Exception as e:
        return False, str(e)


def main():
    if len(sys.argv) < 2:
        print("Usage: python check_nao_connection.py <NAO_IP> [port] [--no-speech]")
        sys.exit(1)

    speak = True
    args = [a for a in sys.argv[1:]]
    if '--no-speech' in args:
        speak = False
        args.remove('--no-speech')

    ip = args[0] if len(args) >= 1 else None
    port = int(args[1]) if len(args) >= 2 else 9559
    if not ip:
        print("Usage: python check_nao_connection.py <NAO_IP> [port] [--no-speech]")
        sys.exit(1)

    print("Checking TCP port {} on {}...".format(port, ip))
    tcp_ok = tcp_check(ip, port)
    print("  TCP reachable: {}".format(tcp_ok))
    if not tcp_ok:
        print("  -> NAOqi probably not reachable (port closed or network issue).")

    print("Checking NAOqi via ALProxy...")
    naoqi_ok, msg = naoqi_check(ip, port)
    print("  ALProxy check: {} - {}".format(naoqi_ok, msg))

    if naoqi_ok:
        print("Connection to NAO successful.")
        try:
            sys_proxy = ALProxy("ALSystem", ip, port)
            version = sys_proxy.systemVersion()
            print("  NAOqi version: {}".format(version))
        except Exception:
            pass

        if speak:
            try:
                tts = ALProxy('ALTextToSpeech', ip, port)
                tts.say('Connected')
            except Exception:
                pass
    else:
        print("ALProxy connection failed.")


if __name__ == "__main__":
    main()
