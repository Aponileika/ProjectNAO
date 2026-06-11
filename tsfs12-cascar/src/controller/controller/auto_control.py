from math import *
import matplotlib.pyplot as plt
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cascar_msgs.msg import CarCommand, CarMeasurement


class AutomaticControl(Node):
    def __init__(self, Ts: float, vmax: float, delta_max:float, kx: float, ky: float, ktheta:float, path:list):

        #Start node
        super().__init__("automatic_control")

        self.get_logger().info("Automatic Control Node Started")

        #Publish steering commands
        self.car_publisher_ = self.create_publisher(CarCommand, "car_command", 1)

        #Subscribe to measurements
        self.state_sub = self.create_subscription(
            CarMeasurement,
            "sensor/imu",
            self.imu_callback,
            10
        )


        #Constants
        self.Ts = Ts
        self.vmax = vmax
        self.delta_max = delta_max      

        self.D = 0.08
        self.L = 0.285

        #Sensor readings
        self.ax = 0
        self.ay = 0
        self.w = 0
        self.calibDone = False
        self.calibData = []
        self.bx = 0
        self.by = 0
        self.bw = 0
        self.last_time = self.get_clock().now()

        #System state
        self.theta = pi/2
        self.x = 1
        self.y = 0
        self.vx = 0
        self.vy = 0

        #Control system data
        self.v = 0
        self.delta = 0
        self.kx = kx
        self.ky = ky
        self.ktheta = ktheta

        self.path = path
        self.curr_node = 1

        self.distanceTraveled = 0

        self.timer = self.create_timer(self.Ts, self.control_loop)


    def imu_callback(self, msg):
        self.ax = msg.a_x - self.bx
        self.ay = msg.a_y - self.by
        self.w = msg.w - self.bw


    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.calibDone:
            self.calibrate()
        else:
            self.update_model(dt)

            ed, ey, etheta = self.get_errors()

            self.control_system(ed, ey, etheta)

            msg = CarCommand()
            #self.get_logger().info(f"v={self.v:.3f}, delta={self.delta:.3f}")
            self.get_logger().info(f"Driven={self.distanceTraveled:.3f}, ed={ed:.3f}, ey={ey:.3f}, et={etheta:.3f}")

            speed_norm = self.v / self.vmax
            steer_norm = self.delta / self.delta_max

            max_speed = 100.0       #Actual command max
            max_steer = 95.0  

            msg.speed = speed_norm * max_speed
            msg.steer = steer_norm * max_steer

            self.car_publisher_.publish(msg)

            self.get_next_node()


    def calibrate(self):
        if len(self.calibData) < 100:
            self.calibData.append({"ax":self.ax, "ay":self.ay, "w":self.w})
        else:
            bxS = 0
            byS = 0
            bwS = 0
            for data in self.calibData:
                bxS += data["ax"]
                byS += data["ay"]
                bwS += data["w"]

            self.bx = bxS / len(self.calibData)
            self.by = byS / len(self.calibData)
            self.bw = bwS / len(self.calibData)

            self.calibDone = True
            self.get_logger().info("CALIBRATION COMPLETE")
            self.get_logger().info(f"Found biases: bx={self.bx:.3f}, by={self.by:.3f}, bw={self.bw:.3f}")
        

    def update_model(self, dt):
        ax = self.ax * cos(self.theta) + self.ay * sin(self.theta)
        ay = -self.ax * sin(self.theta) + self.ay * cos(self.theta)

        self.vx += ax * dt
        self.vy += - ay * dt
        self.vx = self.value_limit(self.vx, self.vmax, -self.vmax)
        self.vy = self.value_limit(self.vy, self.vmax, -self.vmax)

        self.x += self.vx * dt
        self.y += self.vy * dt

        theta = self.theta + self.w * dt
        self.theta = atan2(sin(theta), cos(theta))

        self.get_logger().info(f"dt={dt:.4f}s, x={self.x:.3f}, vx= {self.vx:.3f}, ax={ax:.3f}, y={self.y:.3f}, vy={self.vy:.3f}, ay={ay:.3f}, theta={self.theta:.3f}, w={self.w:.3f}")

        return


    def control_system(self, ed, ey, etheta):
        # Velocity (allow reverse)
        if ed < 0.01:
            ed = 0
        
        v = self.vmax * min(0.9*cos(etheta) + 0.1, self.kx * ed)
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
        dx  = self.path[self.curr_node]["x"] - self.x 
        dy  = self.path[self.curr_node]["y"] - self.y

        ed =  self.path[-1]['distance'] - self.distanceTraveled
        #ex =  dx*cos(self.theta) + dy*sin(self.theta)
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

            ex = dx*cos(self.theta) + dy*sin(self.theta)

            if ex < 0:
                self.curr_node += 1
                self.distanceTraveled = self.path[self.curr_node]['distance']



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
                path.append({"x": i / 200, "y": - 1})
            else:
                path.append({"x": i / 200, "y": + 1})

    elif type == "circle":
        for i in range(180*5):
            path.append({"x": omega * cos(i * 2*pi / 180), "y": omega * sin(i * 2*pi / 180)})

    else:
        for i in range(1000):
            path.append({"x": i / 200, "y": sin(omega * i)})

    distance = 0
    for i in range(len(path) - 1):
        start_node = path[i]
        next_node = path[i+1]

        angle = atan2(next_node["y"] - start_node["y"],
                      next_node["x"] - start_node["x"])
        
        distance += sqrt( (next_node['x'] - start_node['x'])**2 + (next_node['x'] - start_node['x'])**2 )
        
        next_node["distance"] = distance
        next_node["theta"] = angle

    return path


def main(args=None):
    rclpy.init(args=args)

    path = create_path("circle", 1)

    node = AutomaticControl(
        Ts=0.05,
        vmax=0.75,
        delta_max=pi/4,
        kx=1,
        ky=3,
        ktheta=1,
        path=path
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
