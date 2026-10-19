from math import *
import matplotlib.pyplot as plt
from time import sleep
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cascar_msgs.msg import CarCommand, CarMeasurement, Path
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class PurePursuit():
    def __init__(self, vmax: float, delta_max:float, kx: float, kix: float, ky: float, ktheta:float):


        #Constants
        self.vmax = vmax
        self.delta_max = delta_max      

        self.D = 0.08
        self.L = 0.285

        #Control system data
        self.v = 0
        self.delta = 0
        self.kx=kx,
        self.kix = kix,
        self.ky=ky,
        self.ktheta=ktheta

        self.path = []
        self.curr_node = 0

        self.distanceTraveled = 0


    def control_signal(self, x, y, theta, dt, debug = False):
        if self.path:
            self.get_next_node(x, y, theta, self.path[self.curr_node][3])

            dir = self.path[self.curr_node][3]
            ed, ey, etheta = self.get_errors(x, y, theta)

            # Stop Integral Winding (?)
            if abs((self.ied + ed*dt)*self.kix) < 0.95 * self.vmax:
                self.ied += ed * dt

            # Velocity (allow reverse)
            if ed < 0.01:
                ed = 0
                self.ied = 0
            
            v = self.path[0][3] * self.vmax * min(0.9*cos(etheta) + 0.1, self.kx * ed + self.kix * self.ied)
            self.v = self.value_limit(v, -self.vmax, self.vmax)

            # # Adjust heading error for reverse
            # if self.v < 0:
            #     etheta += pi
            # etheta = atan2(sin(etheta), cos(etheta))

            # Steering (NO velocity division!)
            dtheta = (self.path[self.curr_node+1][2] - self.path[self.curr_node][2])
            cs = ((atan2(sin(dtheta), cos(dtheta))) / 
                  (self.path[self.curr_node+1][4] - self.path[self.curr_node][4]))
            u0 = 0
            u = (u0 - 
                 atan(self.ky * ey) - 
                 self.ktheta * etheta)
            delta = atan(u * self.L)
            self.delta = self.value_limit(delta, -self.delta_max, self.delta_max)


            if debug:
                if self.infoCounter >= 1/self.Ts:
                    print(f"Driven={self.distanceTraveled:.3f}, ed={ed:.3f}, ey={ey:.3f}, et={etheta:.3f}, v={self.v:.3f}")
                    self.infoCounter = 0
                else:
                    self.infoCounter += 1

            return (self.v, self.delta)

        else:
            return (0, 0)
   

    def get_errors(self, x, y, theta):
        dx  = self.path[self.curr_node][0] - x 
        dy  = self.path[self.curr_node][1] - y

        ed =  self.path[-1][4] - self.distanceTraveled
        #ex =  dx*cos(self.theta) + dy*sin(self.theta)
        ey = -dx*sin(theta) + dy*cos(theta)

        etheta  = self.path[self.curr_node][2] - theta
        etheta = atan2(sin(etheta), cos(etheta))

        return (ed, ey, etheta)


    def get_next_node(self, x, y, theta, dir):
        #Check if the old node is behind the robot
        #PREVENTS BACKWARDS MOVEMENT
        if self.curr_node != len(self.path) - 1:
            dx  = self.path[self.curr_node][0] - x 
            dy  = self.path[self.curr_node][1] - y

            ex = dx*cos(theta) + dy*sin(theta)

            if dir * ex < 0:
                self.curr_node += 1
                self.distanceTraveled = self.path[self.curr_node][4]
                if self.curr_node > =len(self.path):
                    self.path = []            


    def value_limit(self, value, min, max):
        if value < min:
            return min
        elif value < max:
            return value
        else:
            return max


    def reset(self):
        self.map = []
        self.curr_node = 0
        self.distanceTraveled = 0
        self.ied = 0


    def set_path(self, path):
        self.path = path