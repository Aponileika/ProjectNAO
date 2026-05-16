# -*- coding: utf-8 -*-
import threading
import time

try:
    import Tkinter as tk
except ImportError:
    tk = None

class VisionManager(object):
    def __init__(self, proxies, ui_callbacks=None):
        """
        :param proxies: An object/dictionary that provides access to ALProxies
                        such as video, face, people, motion, leds, memory, audio, tts.
        :param ui_callbacks: Optional dictionary of callback functions to update the GUI.
        """
        self.proxies = proxies
        self.ui = ui_callbacks or {}
        
        self._cam_name = None
        self._cam_running = False
        self._cam_photo = None
        self._human_hold_until = 0.0
        self._alarm_active = False

    def get_proxy(self, name):
        """Helper to get proxy from dict or object."""
        if isinstance(self.proxies, dict):
            return self.proxies.get(name)
        return getattr(self.proxies, name, None)

    def _rgb_to_photo(self, width, height, payload):
        if tk is None:
            raise RuntimeError("Tkinter is not available; cannot create PhotoImage")
        header = "P6\n%d %d\n255\n" % (int(width), int(height))
        ppm_data = header + payload
        photo = tk.PhotoImage(data=ppm_data, format="PPM")

        target_w = 480
        scale = max(1, int(target_w / max(1, int(width))))
        if scale > 1:
            photo = photo.zoom(scale, scale)
        return photo

    def start_camera(self, ip, port, after_func):
        """
        We assume video, face, people proxies are already available or can be created.
        after_func is the Tkinter after() method to schedule the tick.
        """
        if self._cam_running:
            return

        video = self.get_proxy("video")
        if video is None:
            if "set_status" in self.ui:
                self.ui["set_status"]("Camera proxy unavailable", False)
            return

        try:
            self._cam_name = video.subscribe("nao_settings_cam", 1, 11, 5)
        except Exception as e:
            if "set_status" in self.ui:
                self.ui["set_status"]("Camera start error: %s" % e, False)
            return

        face = self.get_proxy("face")
        if face is not None:
            try:
                face.subscribe("nao_settings_face", 500, 0.0)
                face.setTrackingEnabled(True)
            except Exception as e:
                print("[VisionManager] Face detection subscribe failed: %s" % e)

        people = self.get_proxy("people")
        if people is not None:
            try:
                people.setFastModeEnabled(False)
                people.setMaximumDetectionRange(5.0)
                people.subscribe("nao_settings_people")
            except Exception as e:
                print("[VisionManager] People perception subscribe failed: %s" % e)

        motion = self.get_proxy("motion")
        if motion is not None:
            try:
                motion.setAngles("HeadPitch", -0.12, 0.08)
            except Exception:
                pass

        leds = self.get_proxy("leds")
        if leds is not None:
            try:
                leds.fadeRGB("AllLeds", 0x0000FF, 1.0)
            except Exception:
                pass

        self._cam_running = True
        
        if "on_start" in self.ui:
            self.ui["on_start"]()
            
        self.camera_tick(after_func)

    def stop_camera(self, after_cancel_func, after_id):
        self._cam_running = False
        if after_id is not None and after_cancel_func:
            try:
                after_cancel_func(after_id)
            except Exception:
                pass

        video = self.get_proxy("video")
        if video and self._cam_name:
            try:
                video.unsubscribe(self._cam_name)
            except Exception:
                pass
                
        face = self.get_proxy("face")
        if face:
            try:
                face.unsubscribe("nao_settings_face")
            except Exception:
                pass
                
        people = self.get_proxy("people")
        if people:
            try:
                people.unsubscribe("nao_settings_people")
            except Exception:
                pass
                
        self._human_hold_until = 0.0
        
        if self._alarm_active:
            self._alarm_active = False
            tts = self.get_proxy("tts")
            if tts:
                try:
                    tts.stopAll()
                except Exception:
                    pass
                    
        leds = self.get_proxy("leds")
        if leds:
            try:
                leds.fadeRGB("AllLeds", 0x000000, 1.0)
            except Exception:
                pass

        self._cam_name = None
        self._cam_photo = None
        
        if "on_stop" in self.ui:
            self.ui["on_stop"]()

    def _collect_probabilities(self, value, out_list):
        if isinstance(value, (int, float)):
            v = float(value)
            if 0.0 <= v <= 1.0:
                out_list.append(v)
            elif 1.0 < v <= 100.0:
                out_list.append(v / 100.0)
            return
        if isinstance(value, list):
            for item in value:
                self._collect_probabilities(item, out_list)

    def _face_detection_state(self):
        memory = self.get_proxy("memory")
        if not memory:
            return False, None
        try:
            data = memory.getData("FaceDetected")
        except Exception:
            return False, None
        if not data:
            return False, None

        detected = False
        scores = []

        try:
            if isinstance(data, list) and len(data) > 1 and isinstance(data[1], list):
                if len(data[1]) > 0:
                    detected = True
                for face_info in data[1]:
                    if isinstance(face_info, list) and len(face_info) > 1:
                        extra = face_info[1]
                        self._collect_probabilities(extra, scores)
        except Exception:
            pass

        if not scores:
            self._collect_probabilities(data, scores)

        if not scores:
            return detected, None
        return True, max(scores)

    def _people_detected(self):
        memory = self.get_proxy("memory")
        if not memory:
            return False, 0
        try:
            people = memory.getData("PeoplePerception/PeopleList")
        except Exception:
            return False, 0
        if isinstance(people, list) and len(people) > 0:
            return True, len(people)
        return False, 0

    def _alarm_loop(self):
        """Hold the alarm-active state; speech is fired once in _start_alarm."""
        while self._alarm_active:
            time.sleep(0.5)

    def _start_alarm(self):
        self._alarm_active = True
        leds = self.get_proxy("leds")
        if leds:
            try:
                leds.fadeRGB("AllLeds", 0xFF0000, 0.2)
            except Exception:
                pass
        # Say the detection phrase ONCE (not in a tight loop)
        tts = self.get_proxy("tts")
        if tts:
            try:
                tts.say("Human detected.")
            except Exception:
                pass
        # Fire the external on_human_detected callback (e.g. to trigger dance)
        if "on_human_detected" in self.ui:
            try:
                self.ui["on_human_detected"]()
            except Exception as e:
                print("[VisionManager] on_human_detected callback error: %s" % e)
        t = threading.Thread(target=self._alarm_loop)
        t.daemon = True
        t.start()

    def _stop_alarm(self):
        self._alarm_active = False
        tts = self.get_proxy("tts")
        if tts:
            try:
                tts.stopAll()
            except Exception:
                pass
        leds = self.get_proxy("leds")
        if leds:
            try:
                leds.fadeRGB("AllLeds", 0x0000FF, 0.5)
            except Exception:
                pass
        
        def _diffuse():
            time.sleep(0.5)
            if tts:
                try:
                    tts.say("situation diffused")
                except Exception:
                    pass

        t = threading.Thread(target=_diffuse)
        t.daemon = True
        t.start()

    def _update_human_detection_label(self):
        face_detected, conf = self._face_detection_state()
        people_detected, people_count = self._people_detected()
        now = time.time()

        human_present = bool(people_detected or face_detected or now < self._human_hold_until)
        if human_present and not self._alarm_active:
            self._start_alarm()
        elif not human_present and self._alarm_active:
            self._stop_alarm()

        msg = ""
        level = "normal"
        if people_detected:
            self._human_hold_until = now + 1.2
            if people_count > 1:
                msg = "Human presence: yes (%d people)" % people_count
            else:
                msg = "Human presence: yes (1 person)"

            if conf is not None:
                if conf >= 0.80:
                    band = "high"
                elif conf >= 0.60:
                    band = "medium"
                else:
                    band = "low"
                msg += ", face %d%% (%s)" % (int(conf * 100.0), band)

            level = "success"

        elif face_detected:
            self._human_hold_until = now + 1.2
            if conf is None:
                msg = "Human presence: likely (face)"
                level = "warn"
            else:
                if conf >= 0.80:
                    band = "high"
                    level = "success"
                elif conf >= 0.60:
                    band = "medium"
                    level = "warn"
                else:
                    band = "low"
                    level = "warn"

                msg = "Human presence: likely, face %d%% (%s)" % (int(conf * 100.0), band)

        elif now < self._human_hold_until:
            msg = "Human presence: yes (recent)"
            level = "success"
        else:
            msg = "Human presence: no"
            level = "error"
            
        if "update_detection" in self.ui:
            self.ui["update_detection"](msg, level)

    def camera_tick(self, after_func):
        video = self.get_proxy("video")
        if not self._cam_running or not video or not self._cam_name:
            return

        try:
            img = video.getImageRemote(self._cam_name)
            if img and len(img) >= 7:
                width = int(img[0])
                height = int(img[1])
                payload = img[6]                  
                self._last_raw_img = (width, height, payload) # Cache for Gemini to steal independently                 
                photo = self._rgb_to_photo(width, height, payload)
                self._cam_photo = photo
                if "update_frame" in self.ui:
                    self.ui["update_frame"](photo, "Live %dx%d" % (width, height))
            else:
                if "update_frame" in self.ui:
                    self.ui["update_frame"](None, "No frame")
        except Exception as e:
            if "update_frame" in self.ui:
                self.ui["update_frame"](None, "Frame error")
            if "set_status" in self.ui:
                self.ui["set_status"]("Camera frame error: %s" % e, False)

        self._update_human_detection_label()

        if self._cam_running and after_func:
            after_id = after_func(150, lambda: self.camera_tick(after_func))
            if "store_after_id" in self.ui:
                self.ui["store_after_id"](after_id)
