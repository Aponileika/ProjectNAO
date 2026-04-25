# -*- coding: utf-8 -*-
import threading
import time
import math

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

from nao_app.config import _clamp, _lerp

class GamepadController(object):
    def __init__(self, proxies, ui_callbacks=None):
        """
        :param proxies: An object/dictionary that provides access to ALProxies
                        such as motion, posture, memory.
        :param ui_callbacks: Optional dictionary of callback functions to update the GUI.
        """
        self.proxies = proxies
        self.ui = ui_callbacks or {}
        
        self._joystick = None
        self._ctrl_running = False
        self._ctrl_thread = None
        self._ps_prev_down = False
        self._btn_busy = False

        self._MAX_FORWARD  = 0.34
        self._MAX_BACKWARD = 0.24
        self._MAX_ROTATE   = 0.16
        self._SMOOTHING = 0.10
        self._ROTATE_WHILE_FORWARD_FACTOR = 0.50
        
        self._WALK_CONFIG = [
            ["Frequency", 0.90],
            ["MaxStepX", 0.055],
            ["MaxStepY", 0.010],
            ["MaxStepTheta", 0.18],
            ["StepHeight", 0.011],
            ["TorsoWy", 0.01],
        ]

        self._TILT_WARN = 0.20
        self._TILT_DANGER = 0.32

        self._cur_fwd = 0.0
        self._cur_rot = 0.0
        self._PS_BUTTON_CANDIDATES = (10, 12, 16)
        
        self.deadzone = 0.25
        self.speed = 0.50

    def get_proxy(self, name):
        """Helper to get proxy from dict or object."""
        if isinstance(self.proxies, dict):
            return self.proxies.get(name)
        return getattr(self.proxies, name, None)

    def is_pygame_available(self):
        return HAS_PYGAME

    def update_settings(self, deadzone, speed):
        self.deadzone = deadzone
        self.speed = speed

    def detect_controller(self):
        if not HAS_PYGAME:
            if "on_detect" in self.ui:
                self.ui["on_detect"](False, "pygame not installed")
            return
            
        pygame.quit()
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self._joystick = None
            if "on_detect" in self.ui:
                self.ui["on_detect"](False, "No controller found")
            return
            
        js = pygame.joystick.Joystick(0)
        js.init()
        self._joystick = js
        
        name = js.get_name()
        axes = js.get_numaxes()
        if "on_detect" in self.ui:
            self.ui["on_detect"](True, "%s (%d axes)" % (name, axes))

    def start_controller(self):
        if self._joystick is None:
            if "set_status" in self.ui:
                self.ui["set_status"]("Detect a controller first", False)
            return
        if self._ctrl_running:
            return

        motion = self.get_proxy("motion")
        if motion is not None:
            try:
                motion.wakeUp()
                motion.setStiffnesses("Body", 1.0)
                motion.setFallManagerEnabled(True)
                motion.setMoveArmsEnabled(True, True)
                motion.setMotionConfig([
                    ["ENABLE_FOOT_CONTACT_PROTECTION", True],
                ])
            except Exception as e:
                print("[GamepadController] Motion setup warning: %s" % e)

        self._ps_prev_down = False
        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(target=self._controller_loop)
        self._ctrl_thread.daemon = True
        self._ctrl_thread.start()
        
        if "set_status" in self.ui:
            self.ui["set_status"]("Controller active - press Cross to stand")

    def stop_controller(self):
        self._ctrl_running = False
        self._ps_prev_down = False
        self._cur_fwd = 0.0
        self._cur_rot = 0.0
        
        motion = self.get_proxy("motion")
        if motion:
            try:
                motion.moveToward(0, 0, 0)
            except Exception:
                pass
                
        if "on_stop" in self.ui:
            self.ui["on_stop"]()

    def _read_tilt(self):
        memory = self.get_proxy("memory")
        if not memory:
            return 0.0, 0.0
        try:
            ax = memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
            ay = memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
            return float(ax), float(ay)
        except Exception:
            return 0.0, 0.0

    def _stability_factor(self, tilt_x, tilt_y):
        tilt_mag = math.sqrt(tilt_x ** 2 + tilt_y ** 2)
        if tilt_mag >= self._TILT_DANGER:
            return 0.0
        if tilt_mag <= self._TILT_WARN:
            return 1.0
        ratio = (tilt_mag - self._TILT_WARN) / (self._TILT_DANGER - self._TILT_WARN)
        return 1.0 - ratio

    def _is_robot_standing(self):
        posture = self.get_proxy("posture")
        if not posture:
            return False
        try:
            p = posture.getPostureFamily()
            return p in ("Standing", "StandInit", "Stand", "StandZero")
        except Exception:
            return False

    def _stand_safely(self):
        motion = self.get_proxy("motion")
        posture = self.get_proxy("posture")
        if not motion or not posture:
            return False

        try:
            motion.wakeUp()
        except Exception:
            pass
        try:
            motion.setStiffnesses("Body", 1.0)
        except Exception:
            pass
        try:
            motion.setFallManagerEnabled(True)
        except Exception:
            pass
        try:
            motion.setMoveArmsEnabled(True, True)
        except Exception:
            pass

        poses = ("StandInit", "Stand")
        for _ in range(2):
            for pose in poses:
                try:
                    ok = posture.goToPosture(pose, 1.0)
                except Exception:
                    ok = False
                time.sleep(0.5)
                if ok and self._is_robot_standing():
                    return True
        return False

    def _is_ps_button_down(self, js):
        try:
            n = js.get_numbuttons()
            for idx in self._PS_BUTTON_CANDIDATES:
                if idx < n and js.get_button(idx):
                    return True
        except Exception:
            pass
        return False

    def _controller_loop(self):
        js = self._joystick
        
        motion = self.get_proxy("motion")
        posture = self.get_proxy("posture")

        while self._ctrl_running:
            try:
                pygame.event.pump()

                ps_down = self._is_ps_button_down(js)
                if ps_down and not self._ps_prev_down:
                    if "toggle_details" in self.ui:
                        self.ui["toggle_details"]()
                self._ps_prev_down = ps_down

                pressed = []
                for bi in range(js.get_numbuttons()):
                    if js.get_button(bi):
                        pressed.append(str(bi))
                
                try:
                    if js.get_button(0) and not self._btn_busy:
                        if posture:
                            self._btn_busy = True
                            def _do_stand():
                                try:
                                    if "set_status" in self.ui:
                                        self.ui["set_status"]("Standing up safely...")
                                    if self._stand_safely():
                                        if "set_status" in self.ui:
                                            self.ui["set_status"]("Stand complete")
                                    else:
                                        if "set_status" in self.ui:
                                            self.ui["set_status"]("Stand failed - hold robot and retry", False)
                                except Exception as e:
                                    print("[GamepadController] Stand thread error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_stand).start()

                    if js.get_button(1) and not self._btn_busy:
                        if posture:
                            self._btn_busy = True
                            def _do_sit():
                                try:
                                    if "set_status" in self.ui:
                                        self.ui["set_status"]("Sitting via controller...")
                                    posture.goToPosture("StandInit", 0.5)
                                    posture.goToPosture("Sit", 0.5)
                                    if "set_status" in self.ui:
                                        self.ui["set_status"]("Sit complete")
                                except Exception as e:
                                    print("[GamepadController] Sit thread error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_sit).start()

                    if js.get_button(2) and not self._btn_busy:
                        if motion:
                            self._btn_busy = True
                            if "set_status" in self.ui:
                                self.ui["set_status"]("Relaxing servos...")
                            def _do_relax():
                                try:
                                    motion.rest()
                                    if "set_status" in self.ui:
                                        self.ui["set_status"]("Servos relaxed (press Cross to stand)")
                                except Exception as e:
                                    print("[GamepadController] Relax thread error: %s" % e)
                                finally:
                                    self._btn_busy = False
                            threading.Thread(target=_do_relax).start()

                    if js.get_button(3):
                        self.stop_controller()
                        break
                except Exception:
                    pass

                raw_fwd = -js.get_axis(1)
                raw_rot = -js.get_axis(2)

                dz = self.deadzone
                speed = self.speed

                fwd_in = raw_fwd * speed if abs(raw_fwd) > dz else 0.0
                rot_in = raw_rot * speed if abs(raw_rot) > dz else 0.0

                if abs(fwd_in) > 0.10:
                    rot_in *= self._ROTATE_WHILE_FORWARD_FACTOR

                if abs(rot_in) < 0.04:
                    rot_in = 0.0

                tilt_x, tilt_y = self._read_tilt()
                standing = self._is_robot_standing()

                if standing:
                    stab = self._stability_factor(tilt_x, tilt_y)
                else:
                    stab = 1.0

                fwd_target = fwd_in * stab
                if fwd_target >= 0.0:
                    tgt_fwd = _clamp(fwd_target, -self._MAX_FORWARD, self._MAX_FORWARD)
                else:
                    tgt_fwd = _clamp(fwd_target, -self._MAX_BACKWARD, self._MAX_BACKWARD)
                tgt_rot = _clamp(rot_in * stab, -self._MAX_ROTATE,  self._MAX_ROTATE)

                if stab == 0.0 and standing:
                    tgt_fwd = 0.0
                    tgt_rot = 0.0

                self._cur_fwd = _lerp(self._cur_fwd, tgt_fwd, self._SMOOTHING)
                self._cur_rot = _lerp(self._cur_rot, tgt_rot, self._SMOOTHING)

                if abs(self._cur_fwd) < 0.005:
                    self._cur_fwd = 0.0
                if abs(self._cur_rot) < 0.005:
                    self._cur_rot = 0.0

                if "update_axes" in self.ui:
                    self.ui["update_axes"](self._cur_fwd, self._cur_rot)

                if not standing:
                    stab_txt = "N/A (not standing)"
                    stab_col = "fg"
                elif stab >= 0.8:
                    stab_txt = "OK"
                    stab_col = "success"
                elif stab > 0.0:
                    stab_txt = "CAUTION (%d%%)" % int(stab * 100)
                    stab_col = "warn"
                else:
                    stab_txt = "DANGER - walk stopped"
                    stab_col = "error"

                if "update_gyro" in self.ui:
                    self.ui["update_gyro"](tilt_x, tilt_y, stab_txt, stab_col)

                if motion and standing:
                    motion.moveToward(
                        float(self._cur_fwd),
                        0.0,
                        float(self._cur_rot),
                        self._WALK_CONFIG)

                time.sleep(0.05)
            except Exception as e:
                print("[GamepadController] Loop error: %s" % e)
                time.sleep(0.1)
