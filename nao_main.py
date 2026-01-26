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

        self.motion.wbEnable(False)
        self.motion.wakeUp()
        self.posture.goToPosture("StandInit", 0.75)
        
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


    def __del__(self):
        #For when the object disappears
        self.conn.close()
        self.running_threads = False


    def shutdown(self):
        #Safely shuts down robot
        print("\nShutting down\n")

        #Set safe resting posture
        self.posture.goToPosture("Crouch", 0.5)
        time.sleep(2)
        self.motion.rest()

        
        #Close connection, try since connection might not exist
        try:
            self.conn.close()
        except AttributeError:
            pass

        self.running_threads = False
         

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
            pitch = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
            roll = self.memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")

            pitch_vel = self.memory.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")
            roll_vel = self.memory.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value")

            lean_threshold = 45 * 3.1415 / 180 # How much the robot is allowed to lean, at max.
            angular_velocity_threshold = 2.5 # Rotation speed, rad/s

            # The "robotHasFallen" event triggers when the C.O.M. is outside the support surface, meaning it can
            # trigger during some movements, even if not falling. Therefore, the robot lean is taken into account
            if (math.sqrt(pitch**2 + roll**2) > lean_threshold or 
                math.sqrt(pitch_vel**2 + roll_vel**2) > angular_velocity_threshold):
                
                print("ROBOT HAS FALLEN")
                self.balance_event.clear()
                self.posture.goToPosture("StandInit", 1)
                time.sleep(2)

                self.balance_event.set()
            
            time.sleep(0.05)

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
                x1 = float(self.message["x1"])
                y1 = float(self.message["y1"])
                x2 = float(self.message["x2"])
                y2 = float(self.message["y2"])

                if x1**2 < 0.2:
                    x1 = 0
                if y1**2 < 0.2:
                    y1 = 0
                if x2**2 < 0.2:
                    x2 = 0

                self.motion.moveToward(y1, -x1, x2)

            elif self.message["command"] == "sit":
                self.posture.goToPosture("Sit", 0.75)

            elif self.message["command"] == "stand":
                self.posture.goToPosture("StandInit", 0.75)

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
        
