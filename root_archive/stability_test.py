# -*- coding: utf-8 -*-
"""
NAO Stability Limit Testing Script
===================================
Systematically tests movement at increasing intensities to find
the safe operating envelope. Run this WITH A SPOTTER next to the robot.

CONTROLS:
  At any time press R then Enter in the terminal to RELAX servos.
  At prompts: Enter=go, r=relax, s=skip group, q=quit
  Ctrl+C = emergency relax

Usage:
    .\Python\python.exe stability_test.py [--ip 192.168.1.113] [--start-level 1]
"""

import sys
import os
import json
import time
import argparse
import threading

# Python 2/3 compatibility
try:
    input = raw_input
except NameError:
    pass

# Windows keypress detection (non-blocking)
try:
    import msvcrt
    _HAS_MSVCRT = True
except ImportError:
    _HAS_MSVCRT = False

# Add SDK to path from config
_cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.json')
with open(_cfg_path, 'r') as f:
    config = json.load(f)
sys.path.append(config["filepath"])

from naoqi import ALProxy


# -- Configuration -----------------------------------------------------------
STEP_DURATION = 3.0   # seconds per move
PAUSE_BETWEEN = 2.0   # seconds rest between moves
INTENSITY_LEVELS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

# Globals set in main()
motion  = None
posture = None
memory  = None
_relaxed = False          # True after relax() is called
_kill_listener = False    # Signal to stop the background key listener


# -- Background key listener -------------------------------------------------
# Runs in a daemon thread. On Windows, polls msvcrt.kbhit() so pressing 'r'
# at ANY time (even during a blocking goToPosture) instantly relaxes servos.

def _key_listener():
    """Background thread: press 'r' at any time to relax servos."""
    global _kill_listener
    while not _kill_listener:
        try:
            if _HAS_MSVCRT and msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'r', b'R'):
                    relax()
            time.sleep(0.1)
        except Exception:
            pass


def start_key_listener():
    global _kill_listener
    _kill_listener = False
    t = threading.Thread(target=_key_listener)
    t.daemon = True
    t.start()
    print("  [i] Background key listener active -- press 'r' at ANY time to relax servos")


# -- Safety helpers ----------------------------------------------------------

def relax():
    """Instantly kill all servo stiffness. Robot goes completely limp."""
    global _relaxed
    _relaxed = True
    print("\n  [RELAX] Turning off all motors NOW...")
    try:
        motion.stopMove()
    except Exception:
        pass
    try:
        motion.setStiffnesses("Body", 0.0)
    except Exception as e:
        print("  Error relaxing:", e)
    print("  Motors OFF. Robot is limp -- support it physically!\n")


def safe_stop():
    """Try to crouch if stiffness is still on, otherwise just rest."""
    global _kill_listener
    _kill_listener = True  # stop the key listener
    print("\n  [SAFE STOP]")

    if _relaxed:
        # Stiffness is already zero -- don't try to move
        print("  Servos already relaxed. Nothing to do.")
        try:
            motion.rest()
        except Exception:
            pass
        return

    # Stiffness is on -- try a gentle crouch, but don't hang forever
    try:
        motion.stopMove()
    except Exception:
        pass

    print("  Attempting crouch (5s timeout)...")
    try:
        # Run goToPosture in a thread so we can time it out
        done = threading.Event()
        def _crouch():
            try:
                posture.goToPosture("Crouch", 0.5)
            except Exception:
                pass
            done.set()
        ct = threading.Thread(target=_crouch)
        ct.daemon = True
        ct.start()
        finished = done.wait(5.0)  # wait max 5 seconds
        if not finished:
            print("  Crouch timed out -- relaxing servos instead.")
    except Exception:
        pass

    # Always relax at the end
    relax()
    print("  Robot is safe.\n")


def check_fallen():
    """Return True if the robot has fallen."""
    try:
        return memory.getData("robotHasFallen")
    except Exception:
        return False


def careful_stand():
    """Bring the robot to StandInit. Returns True on success."""
    global _relaxed

    if _relaxed:
        print("  Re-enabling stiffness...")
        motion.setStiffnesses("Body", 1.0)
        time.sleep(0.3)
        _relaxed = False

    print("  Standing up...")
    try:
        motion.wakeUp()
        time.sleep(0.5)

        # Enable safety features
        motion.setFallManagerEnabled(True)
        motion.setMoveArmsEnabled(True, True)
        motion.wbEnable(True)

        # Go straight to StandInit at normal speed (0.5)
        # Speed 0.3 was too slow and caused instability
        result = posture.goToPosture("StandInit", 0.5)
        time.sleep(1.0)

        if not result:
            print("  ** goToPosture returned failure.")
            return False

        print("  Standing OK.")
        return True

    except Exception as e:
        print("  Error standing: {}".format(e))
        return False


def prompt(text):
    """Show a prompt. Returns 'continue', 'skip', or 'quit'."""
    if _relaxed:
        answer = input(text + "  [Enter=re-stand+go / r=stay relaxed / q=quit]: ").strip().lower()
        if answer == 'q':
            return 'quit'
        if answer == 'r':
            print("  Staying relaxed.")
            answer2 = input("  Type 'q' to quit or Enter to re-stand: ").strip().lower()
            if answer2 == 'q':
                return 'quit'
        # Try to stand
        if not careful_stand():
            print("  Could not stand. Relaxing.")
            relax()
            return 'quit'
        return 'continue'

    answer = input(text + "  [Enter=go / r=relax / s=skip / q=quit]: ").strip().lower()
    if answer == 'r':
        relax()
        answer2 = input("  Relaxed. 'q' to quit, Enter to re-stand: ").strip().lower()
        if answer2 == 'q':
            return 'quit'
        if not careful_stand():
            relax()
            return 'quit'
        return 'continue'
    if answer == 'q':
        return 'quit'
    if answer == 's':
        return 'skip'
    return 'continue'


def ask_stable():
    """Ask the operator if the robot was stable. Sensor is advisory only."""
    fell = check_fallen()
    if fell:
        print("  (i) Fall sensor triggered -- but YOU decide if it actually fell.")

    answer = input("  Was the robot stable? (y=yes / n=no / f=fell over / r=relax): ").strip().lower()
    if answer == 'r':
        relax()
        return False
    if answer == 'f':
        print("  Recorded as FELL.")
        return False
    return answer == 'y'


# -- Test routines -----------------------------------------------------------

def run_move(x, y, theta, duration, label):
    """Execute a moveToward for `duration` seconds."""
    print("  -> {} | x={:.2f} y={:.2f} theta={:.2f} for {:.1f}s".format(
        label, x, y, theta, duration))

    motion.moveToward(x, y, theta)
    time.sleep(duration)
    motion.stopMove()
    time.sleep(0.5)


def test_single_axis(axis_name, levels):
    """Test one axis at increasing intensity."""
    print("\n" + "=" * 60)
    print("  TEST: {} at increasing intensity".format(axis_name))
    print("=" * 60)

    last_safe = 0.0
    for level in levels:
        if axis_name == "forward":
            x, y, theta = level, 0, 0
        elif axis_name == "backward":
            x, y, theta = -level, 0, 0
        elif axis_name == "lateral_left":
            x, y, theta = 0, level, 0
        elif axis_name == "lateral_right":
            x, y, theta = 0, -level, 0
        elif axis_name == "rotate_left":
            x, y, theta = 0, 0, level
        elif axis_name == "rotate_right":
            x, y, theta = 0, 0, -level
        else:
            continue

        action = prompt("\n  Next: {} at {:.0f}%.".format(axis_name, level * 100))
        if action == 'quit':
            return last_safe, 'quit'
        if action == 'skip':
            return last_safe, 'skip'

        # Stand up safely before each sub-test
        if not careful_stand():
            print("  Cannot stand -- skipping rest of this test.")
            relax()
            return last_safe, 'quit'

        run_move(x, y, theta, STEP_DURATION,
                 "{} {:.0f}%".format(axis_name, level * 100))

        time.sleep(PAUSE_BETWEEN)

        if ask_stable():
            last_safe = level
        else:
            print("  >> Instability at {:.0f}% -- stopping this axis.".format(level * 100))
            return last_safe, 'unstable'

    return last_safe, 'done'


def test_combined(levels):
    """Test diagonal movement: forward + lateral."""
    print("\n" + "=" * 60)
    print("  TEST: Combined forward + lateral (diagonal)")
    print("=" * 60)

    last_safe = 0.0
    for level in levels:
        x = level * 0.7
        y = level * 0.7

        action = prompt("\n  Next: diagonal at {:.0f}% (x={:.2f}, y={:.2f}).".format(
            level * 100, x, y))
        if action == 'quit':
            return last_safe
        if action == 'skip':
            return last_safe

        if not careful_stand():
            relax()
            return last_safe

        run_move(x, y, 0, STEP_DURATION,
                 "diagonal {:.0f}%".format(level * 100))
        time.sleep(PAUSE_BETWEEN)

        if ask_stable():
            last_safe = level
        else:
            return last_safe

    return last_safe


def test_impulse():
    """Test sudden start/stop at moderate speed."""
    print("\n" + "=" * 60)
    print("  TEST: Sudden start/stop (impulse)")
    print("=" * 60)

    results = {}
    for direction, (x, y, th) in [
        ("sudden_forward",       (0.4, 0, 0)),
        ("sudden_lateral_left",  (0, 0.3, 0)),
        ("sudden_lateral_right", (0, -0.3, 0)),
    ]:
        action = prompt("\n  Next: {}".format(direction))
        if action == 'quit':
            return results
        if action == 'skip':
            return results

        if not careful_stand():
            relax()
            return results

        print("  -> Sudden START")
        run_move(x, y, th, 2.0, direction)

        print("  -> Sudden STOP")
        motion.stopMove()
        time.sleep(2.0)

        results[direction] = ask_stable()

    return results


def test_reversal():
    """Test sudden direction reversal."""
    print("\n" + "=" * 60)
    print("  TEST: Sudden direction reversal")
    print("=" * 60)

    results = {}
    for name, cmd_a, cmd_b in [
        ("forward_to_backward", (0.3, 0, 0), (-0.3, 0, 0)),
        ("left_to_right",       (0, 0.25, 0), (0, -0.25, 0)),
    ]:
        action = prompt("\n  Next: {} reversal.".format(name))
        if action == 'quit':
            return results
        if action == 'skip':
            return results

        if not careful_stand():
            relax()
            return results

        print("  -> Phase A")
        run_move(cmd_a[0], cmd_a[1], cmd_a[2], 2.0, name + " A")

        print("  -> REVERSAL to Phase B")
        run_move(cmd_b[0], cmd_b[1], cmd_b[2], 2.0, name + " B")

        motion.stopMove()
        time.sleep(1.5)

        results[name] = ask_stable()

    return results


# -- Main --------------------------------------------------------------------

def main():
    global motion, posture, memory

    parser = argparse.ArgumentParser(description="NAO stability limit tester")
    parser.add_argument("--ip", default=None,
                        help="Robot IP (default from config.json)")
    parser.add_argument("--start-level", type=int, default=1,
                        help="Test group to start from (1-7)")
    args = parser.parse_args()

    with open(_cfg_path, 'r') as f:
        cfg = json.load(f)
    ip = args.ip or str(cfg['ip'])
    port = int(cfg['port'])

    print("Connecting to NAO at {}:{}".format(ip, port))
    motion  = ALProxy("ALMotion", ip, port)
    posture = ALProxy("ALRobotPosture", ip, port)
    memory  = ALProxy("ALMemory", ip, port)

    results = {}

    try:
        print("\n" + "#" * 60)
        print("  NAO STABILITY LIMIT TEST")
        print("")
        print("  CONTROLS:")
        print("    Press 'r' at ANY TIME to instantly relax servos")
        print("    At prompts: Enter=go, s=skip, q=quit")
        print("    Ctrl+C = emergency relax")
        print("")
        print("  Have a spotter ready to catch the robot!")
        print("#" * 60)

        start_key_listener()

        action = prompt("\n  Ready to wake up the robot?")
        if action == 'quit':
            relax()
            return results

        motion.wakeUp()
        time.sleep(1.0)

        if not careful_stand():
            print("\n  Robot could not stand up safely.")
            print("  Check that it is on a flat surface and try again.")
            relax()
            return results

        print("\n  Robot is standing. Starting tests...\n")

        # -- 1-4: Single axis tests --
        axes = ["forward", "backward", "lateral_left",
                "lateral_right", "rotate_left", "rotate_right"]

        if args.start_level <= 1:
            for axis in axes:
                safe, status = test_single_axis(axis, INTENSITY_LEVELS)
                results[axis] = safe
                print("\n  >> {} safe limit: {:.0f}%".format(axis, safe * 100))
                if status == 'quit':
                    raise KeyboardInterrupt
                if status == 'unstable':
                    action = prompt("  Move to next axis?")
                    if action == 'quit':
                        raise KeyboardInterrupt

        # -- 5: Combined diagonal --
        if args.start_level <= 5:
            safe = test_combined(INTENSITY_LEVELS)
            results["diagonal"] = safe

        # -- 6: Impulse --
        if args.start_level <= 6:
            results["impulse"] = test_impulse()

        # -- 7: Reversal --
        if args.start_level <= 7:
            results["reversal"] = test_reversal()

        # -- Summary --
        print("\n" + "=" * 60)
        print("  RESULTS SUMMARY")
        print("=" * 60)
        for k, v in sorted(results.items()):
            if isinstance(v, float):
                print("  {:25s}  safe up to {:.0f}%".format(k, v * 100))
            else:
                print("  {:25s}  {}".format(k, v))

        print("")
        print("  Recommendation: use the LOWEST value as your MAX_SPEED")
        print("  clamp, then apply 20-30%% extra safety margin.")
        print("")

    except KeyboardInterrupt:
        print("\n\n  Ctrl+C -- emergency relax!")
        relax()
    finally:
        safe_stop()

    return results


if __name__ == "__main__":
    main()
