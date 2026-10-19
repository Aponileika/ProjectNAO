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

from .pursuit import *


class Controller(Node):
    def __init__(self, Ts: float, vmax: float, delta_max:float):

        #Start node
        super().__init__("control")

        self.get_logger().info("Control Node Started")

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
        self.x = 0
        self.y = 0
        self.theta = 0

        #Control system data
        kx = 2
        kix = 1
        ky = 2
        ktheta = 3
        self.controller = PurePursuit(vmax, delta_max, kx, kix, ky, ktheta)

        self.path = []

        self.infoCounter = 0
        self.last_time = None
        self.timer = self.create_timer(self.Ts, self.control_loop)


    def pos_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    
    def path_callback(self, msg):
        recievedPath = []
        for point in msg.points:
            recievedPath.append([point.x, point.y, point.theta, point.dir, point.distance])

        if not self.path:
            self.path = recievedPath
            self.controller.set_path(recievedPath)
        
            self.curr_node = 0
            self.distanceTraveled = 0

            #self.get_logger().info("PATH RECIEVED")

        elif (recievedPath[0][0] != self.path[0][0] and recievedPath[0][1] != self.path[0][1] or 
              recievedPath[-1][0] != self.path[-1][0] and recievedPath[-1][1] != self.path[-1][1]):
            self.path = recievedPath
            self.controller.set_path(recievedPath)
        
            self.curr_node = 0
            self.distanceTraveled = 0

            #self.get_logger().info("PATH RECIEVED")

        
    def publish_control(self, v, delta):
        speed_norm = v / self.vmax
        steer_norm = delta / self.delta_max

        max_speed = 100.0       #Actual command max
        max_steer = 95.0  

        msg = CarCommand()
        msg.speed = speed_norm * max_speed
        msg.steer = steer_norm * max_steer

        self.car_publisher_.publish(msg)


    def main_loop(self):
        if self.last_time is None:
            self.last_time = self.get_clock().now()

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        v, delta = self.controller.control_signal(self.x, self.y, self.theta, dt)
        self.publish_control(v, delta)


def main(args=None):
    rclpy.init(args=args)

    node = Controller(
        Ts=1/20,
        vmax=1,
        delta_max=pi/4
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
