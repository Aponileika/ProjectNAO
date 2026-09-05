from math import *
import matplotlib.pyplot as plt
from time import sleep
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cascar_msgs.msg import CarCommand, CarMeasurement
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


class PurePursuit(Node):
    def __init__(self, Ts: float, vmax: float, delta_max:float, kx: float, kix: float, ky: float, ktheta:float):

        #Start node
        super().__init__("automatic_control")

        self.get_logger().info("Automatic Control Node Started")

        #Publish steering commands
        self.car_publisher_ = self.create_publisher(CarCommand, "car_command", 1)

        #Subscribe to measurements
        self.pos_sub = self.create_subscription(
            Pose2D,
            "pose",
            self.pos_callback,
            10
        )

        self.path_sub = self.create_subscription(
            String,
            "planned_path",
            self.path_callback,
            10
        )

        #Constants
        self.Ts = Ts
        self.vmax = vmax
        self.delta_max = delta_max      

        self.D = 0.08
        self.L = 0.285

        #System state
        self.x = 1
        self.y = 0
        self.theta = pi/2

        #Control system data
        self.v = 0
        self.delta = 0
        self.kx = kx
        self.kix = kix
        self.ied = 0
        self.ky = ky
        self.ktheta = ktheta

        self.path = []
        self.curr_node = 1

        self.distanceTraveled = 0

        #For debugging
        self.lastDist = self.distanceTraveled
        self.infoCounter = 0

        self.last_time = None
        self.timer = self.create_timer(self.Ts, self.control_loop)


    def pos_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    
    def path_callback(self, msg):
        recievedPath = json.loads(msg.data)
        if not self.path:
            self.path = recievedPath
        
            self.curr_node = 1
            self.distanceTraveled = 0

            #self.get_logger().info("PATH RECIEVED")

        elif recievedPath[0] != self.path[0]:
            self.path = recievedPath
        
            self.curr_node = 1
            self.distanceTraveled = 0

            #self.get_logger().info("PATH RECIEVED")


    def control_loop(self):
        if self.last_time is None:
            self.last_time = self.get_clock().now()

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if self.path:
            ed, ey, etheta = self.get_errors()

            #Motverka integraluppvridning
            if abs((self.ied + ed*dt)*self.kix) < 0.95 * self.vmax:
                self.ied += ed * dt

            self.control_system(ed, ey, etheta)

            msg = CarCommand()
            #self.get_logger().info(f"v={self.v:.3f}, delta={self.delta:.3f}")
            if self.infoCounter >= 1/self.Ts:
                self.get_logger().info(f"Driven={self.distanceTraveled:.3f}, ed={ed:.3f}, ey={ey:.3f}, et={etheta:.3f}, v={self.v:.3f}")
                self.infoCounter = 0

            speed_norm = self.v / self.vmax
            steer_norm = self.delta / self.delta_max

            max_speed = 100.0       #Actual command max
            max_steer = 95.0  

            msg.speed = speed_norm * max_speed
            msg.steer = steer_norm * max_steer

            self.car_publisher_.publish(msg)

            self.get_next_node()

            if self.lastDist != self.distanceTraveled:
                self.lastDist = self.distanceTraveled
                self.infoCounter += 1


    def control_system(self, ed, ey, etheta):
        # Velocity (allow reverse)
        if ed < 0.01:
            ed = 0
            self.ied = 0
        
        v = self.vmax * min(0.9*cos(etheta) + 0.1, self.kx * ed + self.kix * self.ied)
        self.v = self.value_limit(v, self.vmax, -self.vmax)

        # Adjust heading error for reverse
        if self.v < 0:
            etheta += pi

        etheta = atan2(sin(etheta), cos(etheta))

        # Steering (NO velocity division!)
        delta = self.ktheta * etheta + self.ky * atan(self.ky * ey)
        self.delta = self.value_limit(delta, self.delta_max, -self.delta_max)

        #self.get_logger().info(f"ed={ed:.3f}, ey={ey:.3f}, etheta={etheta:.3f}")
    

    def get_errors(self):
        dx  = self.path[self.curr_node][0] - self.x 
        dy  = self.path[self.curr_node][1] - self.y

        ed =  self.path[-1][3] - self.distanceTraveled
        #ex =  dx*cos(self.theta) + dy*sin(self.theta)
        ey = -dx*sin(self.theta) + dy*cos(self.theta)
        ey = np.sign(ey) * ((abs(ey) + 1)**2 - 1)

        etheta  = self.path[self.curr_node][2] - self.theta
        etheta = atan2(sin(etheta), cos(etheta))

        return (ed, ey, etheta)


    def get_next_node(self):
        #Check if the old node is behind the robot
        #PREVENTS BACKWARDS MOVEMENT
        if self.curr_node != len(self.path) - 1:
            dx  = self.path[self.curr_node][0] - self.x 
            dy  = self.path[self.curr_node][1] - self.y

            ex = dx*cos(self.theta) + dy*sin(self.theta)

            if ex < 0:
                self.curr_node += 1
                self.distanceTraveled = self.path[self.curr_node][3]


    def value_limit(self, value, max, min):
        if value < min:
            return min
        elif value < max:
            return value
        else:
            return max



def main(args=None):
    rclpy.init(args=args)

    node = PurePursuit(
        Ts=0.05,
        vmax=1,
        delta_max=pi/4,
        kx=2,
        kix = 1,
        ky=2,
        ktheta=3
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
