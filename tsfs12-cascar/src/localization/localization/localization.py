from math import *
import matplotlib.pyplot as plt
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cascar_msgs.msg import CarCommand, CarMeasurement
from geometry_msgs.msg import Pose2D


class Localization(Node):
    def __init__(self, Ts = 0.05):

        super().__init__("localization")

        self.get_logger().info("Localization Node Started")

        self.pos_publisher = self.create_publisher(Pose2D, "pose", 1)

        self.imu_sub = self.create_subscription(
            CarMeasurement,
            "sensor/imu",
            self.imu_callback,
            10
        )
        self.odo_sub = self.create_subscription(
            CarMeasurement,
            "sensor/cascar",
            self.odo_callback,
            10
        )

        self.Ts = Ts

        self.calibDone = False
        self.calibData = []
        self.w = 0
        self.bw = 0
        self.last_time = self.get_clock().now()

        self.Vodo = 0
        self.oldVodo = None
        self.VodoTime = 0
        self.oldVodoTime = None

        self.standstillIterations = 0

        self.x = -2
        self.y = 0
        self.theta = 0

        self.timer = self.create_timer(self.Ts, self.updateState)


    def imu_callback(self, msg):
        self.w = msg.w - self.bw


    def odo_callback(self, msg):

        self.Vodo = msg.v
        stamp = msg.header.stamp
        self.VodoTime = stamp.sec + 1e-9*stamp.nanosec

        if self.oldVodo == None:
            self.oldVodo = self.Vodo
            self.oldVodoTime = self.VodoTime    


    def updateState(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.calibDone:
            self.calibrate()
        else:
            self.update_model(dt)

            #PUBLISH DATA
            msg = Pose2D()
            msg.x = float(self.x)
            msg.y = float(self.y)
            msg.theta = float(self.theta)

            self.pos_publisher.publish(msg)


    def calibrate(self):
        if len(self.calibData) < 100:
            self.calibData.append({"w":self.w})
        else:
            bwS = 0
            for data in self.calibData:
                bwS += data["w"]

            self.bw = bwS / len(self.calibData)

            self.calibDone = True
            self.get_logger().info("CALIBRATION COMPLETE")
            self.get_logger().info(f"Found biases: bw={self.bw:.3f}")


    def update_model(self, dt):
        if (self.Vodo != self.oldVodo) and (self.oldVodo != None) and (self.oldVodoTime != None):
            odoT = self.VodoTime - self.oldVodoTime

            self.oldVodoTime = self.VodoTime
            self.oldVodo = self.Vodo
            self.standstillIterations = 0

        elif (self.Vodo == self.oldVodo):
            self.standstillIterations += 1
            if self.standstillIterations > 10:
                self.Vodo = 0
                self.oldVodo = 0

        if abs(self.w) < 1e-4:
            self.x += self.Vodo * dt * cos(self.theta)
            self.y += self.Vodo * dt * sin(self.theta)

        else:
            temp = self.w * dt / 2
            self.x += 2*self.Vodo/self.w*sin(temp)*cos(self.theta + temp)
            self.y += 2*self.Vodo/self.w*sin(temp)*sin(self.theta + temp)

        if self.Vodo != 0:
            theta = self.theta + self.w * dt
            self.theta = atan2(sin(theta), cos(theta))

        self.get_logger().info(f"dt={dt:.4f}s, x={self.x:.3f}, y={self.y:.3f}, vodo={self.Vodo:.3f}, theta={self.theta:.3f}, w={self.w:.3f}")

        return


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()