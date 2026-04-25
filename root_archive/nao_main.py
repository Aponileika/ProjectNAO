import sys
import json
import socket
import time
import threading
import math
from custom_moves import *

#Add SDK
with open('config.json', 'r') as f:
    config = json.load(f)

sys.path.append(config["filepath"])
from naoqi import ALProxy


# ── Safety constants ──────────────────────────────────────────
MAX_FORWARD  = 0.28  # forward / backward cap
MAX_LATERAL  = 0.10  # sideways cap  (most dangerous axis)
MAX_ROTATION = 0.20  # rotation cap
SMOOTH_FACTOR = 0.10 # ramping speed (lower = smoother)
DEADZONE = 0.20      # joystick dead zone
TILT_DANGER = 0.30   # radians; emergency stop threshold (~17 deg)

def _clamp(v, limit):
    return max(-limit, min(limit, v))

def _deadzone(v, thresh=DEADZONE):
    return 0.0 if abs(v) < thresh else v

def _smooth(cur, tgt, factor=SMOOTH_FACTOR):
    return cur + factor * (tgt - cur)
# ──────────────────────────────────────────────────────────────


class Nao(object):
    def __init__(self):
        with open('config.json', 'r') as f:
            config = json.load(f)
        IP = str(config['ip'])
        PORT = int(config['port'])

        #MOTION SETUP
        self.motion  = ALProxy("ALMotion", IP, PORT)
        self.posture = ALProxy("ALRobotPosture", IP, PORT) 
        self.memory  = ALProxy("ALMemory", IP, PORT)

        self._configure_motion_safety()
        self._stand_with_retries()

        # Conservative gait config for improved straight-line stability
        self.walk_config = [
            ["Frequency", 0.65],
            ["MaxStepX", 0.03],
            ["MaxStepY", 0.010],
            ["MaxStepTheta", 0.20],
            ["StepHeight", 0.012],
            ["TorsoWy", 0.02],
        ]

        # Smoothed velocity state
        self._smooth_x = 0.0
        self._smooth_y = 0.0
        self._smooth_theta = 0.0
        
        #COMMUNICATIONS SETUP
        self.conn = None
        self.addr = None
        self.message = {"command": None}

        #THREADS SETUP
        self.running_threads = True
        self.balance_event = threading.Event()
        self.movement_event = threading.Event()
        self.balance_event.set()
        self.movement_event.set()

        self.balance_t = threading.Thread(target=self.balance)
        self.movement_t = threading.Thread(target=self.movement)
        self.connection_t = threading.Thread(target=self.remote_connection)
        
        self.balance_t.daemon = True
        self.movement_t.daemon = True
        self.connection_t.daemon = True

        self.balance_t.start()
        self.movement_t.start()
        self.connection_t.start()

        self.balance_t.join()
        self.movement_t.join()
        self.connection_t.join()


    def _configure_motion_safety(self):
        try:
            self.motion.wakeUp()
        except Exception:
            pass
        try:
            self.motion.setStiffnesses("Body", 1.0)
        except Exception:
            pass
        try:
            self.motion.setFallManagerEnabled(True)
        except Exception:
            pass
        try:
            self.motion.setMoveArmsEnabled(True, True)
        except Exception:
            pass
        try:
            self.motion.wbEnable(True)
        except Exception:
            pass
        try:
            self.motion.setMotionConfig([
                ["ENABLE_FOOT_CONTACT_PROTECTION", True],
            ])
        except Exception:
            pass


    def _stop_motion_now(self):
        try:
            self.motion.stopMove()
        except Exception:
            pass
        self._smooth_x = 0.0
        self._smooth_y = 0.0
        self._smooth_theta = 0.0


    def _is_standing(self):
        try:
            fam = self.posture.getPostureFamily()
            return fam == "Standing"
        except Exception:
            return False


    def _has_fallen(self):
        try:
            return bool(self.memory.getData("robotHasFallen"))
        except Exception:
            return False


    def _tilt_too_high(self):
        try:
            ax = float(self.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value"))
            ay = float(self.memory.getData(
                "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"))
            return math.sqrt(ax * ax + ay * ay) >= TILT_DANGER
        except Exception:
            return False


    def _stand_with_retries(self):
        self._configure_motion_safety()
        poses = ["StandInit", "Stand"]
        for _ in range(2):
            for pose in poses:
                try:
                    ok = self.posture.goToPosture(pose, 1.0)
                except Exception:
                    ok = False
                time.sleep(0.6)
                if ok and self._is_standing() and not self._has_fallen():
                    return True
        return False


    def __del__(self):
        #For when the object disappears
        try:
            self.conn.close()
        except Exception:
            pass
        self.running_threads = False


    def shutdown(self):
        #Safely shuts down robot
        print("\nShutting down\n")

        #Close connection, try since connection might not exist
        try:
            self.conn.close()
        except AttributeError:
            pass

        self.running_threads = False
        self._stop_motion_now()

        #Set safe resting posture
        self.posture.goToPosture("Crouch", 0.5)
        time.sleep(2)
        self.motion.rest()
         

    def comm_init(self):
        #Initiate connections

        HOST = "127.0.0.1" #HARDKODAD
        HOST_PORT = 5000 #EVENTUELLT EN PORT PER PROCESS
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, HOST_PORT))
        s.listen(1)

        self.conn, self.addr = s.accept()


    def balance(self):
        #Checks if robot has fallen and needs to stand up, blocks other processes if that is the case,
        #currently disabled, needs more work
        #TO DO: MAKE ROBOT STAND UP BY ITSELF
        #TO DO: FALL MANAGER; PREVENTS FALLS

        print("Balance thread started")

        while self.running_threads:
            try:
                fallen = self._has_fallen()
                tilted = self._tilt_too_high()
            except Exception:
                fallen = False
                tilted = False

            if fallen or tilted:
                print("UNSAFE POSTURE DETECTED - stopping and recovering")
                self.balance_event.clear()
                self._stop_motion_now()
                recovered = self._stand_with_retries()

                if recovered:
                    print("Recovery successful")
                    self.balance_event.set()
                else:
                    print("Recovery failed - keeping movement blocked")
                    try:
                        self.motion.rest()
                    except Exception:
                        pass

            else:
                self.balance_event.set()

            time.sleep(0.1)


        print("balance_return")
        return


    def movement(self):
        """
        Decodes messages and triggers corresponding functions

        Longer movements / animations should be written in the "custom_moves" file
        Messages must have "command" key
        """
        
        print("Movement thread started")

        while self.running_threads:
            print("waiting for command")
            self.movement_event.wait() #Only pass if there is a new instruction

            print("command being handled")
            print(self.message)

            if self.message["command"] == "walk":
                if not self._is_standing():
                    if not self._stand_with_retries():
                        print("Stand recovery failed - ignoring walk command")
                        self._stop_motion_now()
                        self.movement_event.clear()
                        continue

                if self._has_fallen() or self._tilt_too_high():
                    print("Unsafe to walk - ignoring command")
                    self._stop_motion_now()
                    self.movement_event.clear()
                    continue

                x1 = float(self.message["x1"])
                y1 = float(self.message["y1"])
                x2 = float(self.message["x2"])
                y2 = float(self.message["y2"])

                # Deadzone
                x1 = _deadzone(x1)
                y1 = _deadzone(y1)
                x2 = _deadzone(x2)

                # Clamp to safe limits
                fwd = _clamp(y1,  MAX_FORWARD)
                lat = _clamp(-x1, MAX_LATERAL)
                rot = _clamp(x2,  MAX_ROTATION)

                # Suppress lateral and turning authority while moving forward
                # to reduce side-to-side sway and yaw oscillation.
                if abs(fwd) > 0.12:
                    lat *= 0.40
                    rot *= 0.50

                # Smooth to prevent sudden jolts
                self._smooth_x     = _smooth(self._smooth_x,     fwd)
                self._smooth_y     = _smooth(self._smooth_y,     lat)
                self._smooth_theta = _smooth(self._smooth_theta, rot)

                if abs(self._smooth_y) < 0.01:
                    self._smooth_y = 0.0
                if abs(self._smooth_theta) < 0.01:
                    self._smooth_theta = 0.0

                self.motion.moveToward(
                    self._smooth_x, self._smooth_y, self._smooth_theta, self.walk_config)

            elif self.message["command"] == "sit":
                self._stop_motion_now()
                self.posture.goToPosture("StandInit", 0.5)
                self.posture.goToPosture("Sit", 0.5)

            elif self.message["command"] == "stand":
                self._stand_with_retries()

            #EXAMPLE, NOT REAL
            elif self.message["command"] == "kick":
                kick(self)

            elif self.message["command"] == "quit":
                self.shutdown()

            self.movement_event.clear()

        print("movement_return")
        return


    def remote_connection(self):
        """
        Listen to connections

        Recieves remote messages and triggers an event for another thread to process the message
        """
        print("Remote connection thread started")
        
        self.comm_init()

        buffer = ''
        while self.running_threads:
            self.balance_event.wait()  #Only pass if the robot has not fallen over

            
            try:
                print("WAITING FOR REMOTE MESSAGE")
                data = self.conn.recv(1024)
                self.message = json.loads(data.decode())

                print(self.message)
                if self.message != "":
                    self.movement_event.set()
                    print("COMMAND RECIEVED")

                time.sleep(0.1)


            except Exception as e:
                #RESTART CONNECTION 
                print("Error:", e)

                self.conn.close()
                time.sleep(1)
                self.comm_init()
                time.sleep(1)
            

        print("remote_connection_return") 
        return
    

if __name__ == "__main__":
    nao_obj = Nao()
        
