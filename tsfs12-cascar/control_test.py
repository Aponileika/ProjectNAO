from math import *
import matplotlib.pyplot as plt
from time import sleep

import rclpy
from rclpy.node import Node
from cascar_msgs.msg import CarCommand, CarMeasurement


class AutomaticControl(Node):
    def __init__(self, Ts: int, vmax: float, delta_max:float, kx: float, ky: float, ktheta:float, path:list):

        #Start node
        super().__init__("automatic_control")

        self.get_logger().info("Automatic Control Node Started")

        #Publish steering commands
        self.car_publisher_ = self.create_publisher(CarCommand, "car_command", 1)

        #Subscribe to measurements
        self.state_sub = self.create_subscription(
            CarMeasurement,
            "sensor/cascar",
            self.state_callback,
            10
        )


        #Constants
        self.Ts = Ts
        self.vmax = vmax
        self.delta_max = delta_max

        self.max_speed: float = 100.0       #Actual command max
        self.max_steer: float = 95.0        

        self.D = 0.15
        self.L = 0.285

        #System state
        self.theta = pi*3/4
        self.x = 0.5 / sqrt(2)
        self.y = 0.5 / sqrt(2)

        #Control system data
        self.v = 0
        self.delta = 0
        self.kx = kx
        self.ky = ky
        self.ktheta = ktheta

        self.path = path
        self.curr_node = 1

        self.timer = self.create_timer(self.Ts, self.control_loop)


    def control_loop(self):
        self.update_model()

        ed, ey, etheta = self.get_errors()

        self.control_system(ed, ey, etheta)

        msg = CarCommand()

        speed_norm = self.value_limit(self.v / self.vmax, 1.0, -1.0)
        steer_norm = self.value_limit(self.delta / self.delta_max, 1.0, -1.0)

        msg.speed = speed_norm * self.max_speed
        msg.steer = steer_norm * self.max_steer

        self.car_publisher_.publish(msg)

        self.get_next_node()

    def state_callback(self, msg):
        pass

    def update_model(self):
        xdot = self.v * cos(self.theta)
        ydot = self.v * sin(self.theta)
        theta_dot = self.v / self.L * tan(self.delta)

        self.x += xdot * self.Ts
        self.y += ydot * self.Ts
        self.theta += theta_dot * self.Ts

        return


    def control_system(self, ed, ey, etheta):
        # Velocity (allow reverse)
        if ed < 0.1:
            ed = 0

        v = self.vmax * min(0.9*cos(etheta) + 0.1, self.kx * ed)
        self.v = self.value_limit(v, self.vmax, -self.vmax)

        # Adjust heading error for reverse
        if self.v < 0:
            etheta += pi

        etheta = atan2(sin(etheta), cos(etheta))

        # Steering (NO velocity division!)
        delta = etheta + atan(self.ky * ey)
        self.delta = self.value_limit(delta, self.delta_max, -self.delta_max)
    

    def get_errors(self):
        dx  = self.path[self.curr_node]["x"] - self.x 
        dy  = self.path[self.curr_node]["y"] - self.y

        ed =  sqrt( (self.path[-1]["x"] - self.x)**2 + (self.path[-1]["y"] - self.y)**2)
        ey = -dx*sin(self.theta) + dy*cos(self.theta)

        etheta  = self.path[self.curr_node]["theta"] - self.theta
        etheta = atan2(sin(etheta), cos(etheta))

        return (ed, ey, etheta)


    def get_next_node(self):
        #Check if the old node is behind the robot
        #PREVENTS BACKWARDS MOVEMENT
        if self.curr_node != len(self.path) - 1:
            dx  = self.path[self.curr_node]["x"] - self.x 
            dy  = self.path[self.curr_node]["y"] - self.y

            ed = dx*cos(self.theta) + dy*sin(self.theta)

            if ed < 0:
                self.curr_node += 1


    def value_limit(self, value, max, min):
        if value < min:
            return min
        elif value < max:
            return value
        else:
            return max
    

    
def create_path(type, omega):
    path = []

    if type == "square":
        x = 0
        y = -1

        for i in range(1000):
            if int(i / omega) % 2 == 0:
                path.append({"x": i / 100, "y": - 1})
            else:
                path.append({"x": i / 100, "y": + 1})

    elif type == "circle":
        for i in range(180):
            path.append({"x": omega * cos(i * 2*pi / 180), "y": omega * sin(i * 2*pi / 180)})

    else:
        for i in range(1000):
            path.append({"x": i / 100, "y": sin(omega * i)})

    for i in range(len(path) - 1):
        start_node = path[i]
        next_node = path[i+1]

        angle = atan2(next_node["y"] - start_node["y"],
                      next_node["x"] - start_node["x"])
        
        next_node["theta"] = angle

    return path


def main(args=None):
    rclpy.init(args=args)

    path = create_path("circle", 0.5)

    node = AutomaticControl(
        Ts=0.1,
        vmax=1,
        delta_max=pi/4,
        kx=1,
        ky=15,
        ktheta=1,
        path=path
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()