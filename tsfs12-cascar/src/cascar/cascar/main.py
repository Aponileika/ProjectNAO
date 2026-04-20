import os
from time import sleep
import serial
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from cascar_msgs.msg import CarCommand, CarMeasurement


# from cascar_msgs.msg import CarTicks


class CasCar(Node):
    def __init__(self) -> None:
        super().__init__('cascar')
        self.get_logger().info("Cascar Node Started")

        # Initialize serial communication
        self.port = '/dev/arduino'
        self.rate = 115200
        self.ser = self.open_serial_connection()

        if self.ser is None:
            self.get_logger().error("Failed to connect to serial port. Retrying once...")
            sleep(2)
            ser = self.open_serial_connection()
            if ser is None:
                self.get_logger().error("Failed to connect to serial port. Exiting...")
                exit(1)
            else:
                self.ser = ser

        # Initialize states
        self.steer = 0.
        self.speed = 0.

        # Run start sequence
        self.start_sequence()

        # Subscribers
        self.car_command_sub_ = self.create_subscription(CarCommand, 'car_command', self.car_cmd_callback, 1)
        self.speed_command_sub_ = self.create_subscription(CarCommand, 'speed_command', self.speed_cmd_callback, 1)
        self.steer_command_sub_ = self.create_subscription(CarCommand, 'steer_command', self.steer_cmd_callback, 1)

        # Publishers
        self.car_pub_ = self.create_publisher(CarMeasurement, 'sensor/cascar', 1)

        # Initialize variables
        self.wheel_radius = 78.0 / 2 / 1000  # m
        self.num_magnets = 10
        self.steer_params = [0.02840205, 0.00335285]

        fs = 100  # Hz
        timer_period = 1 / fs  # seconds
        self.timer = self.create_timer(timer_period, self.run)

    def start_sequence(self) -> None:
        self.get_logger().info("Waiting for communications to settle...")
        sleep(2)

        self.get_logger().info("Sending start sequence...")

        self.send_cmd(steer=20)
        sleep(0.5)
        self.send_cmd(steer=-20)
        sleep(0.5)
        self.send_cmd(steer=0)

        self.get_logger().info("Start sequence completed")

    def open_serial_connection(self) -> serial.Serial | None:
        if os.path.exists(self.port):
            ser = serial.Serial(self.port, self.rate, timeout=1)
            ser.flush()

            print(f"Wating for serial data on port {self.port}...")
            sleep(2)
            ser.write('PING;\r'.encode())
            sleep(0.05)

            found_serial = False
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8')
                msg = line.split(';')[0]
                if msg == 'PONG':
                    found_serial = True
                    break

            if found_serial:
                print(f"Connected to serial port {self.port}")
                return ser
            else:
                print(f"Failed to connect to serial port {self.port}")
                ser.close()
                return None

    def check_writeable(self) -> bool:
        return self.ser and self.ser.writable()

    @staticmethod
    def speed_cmd(speed: float) -> str:
        try:
            if np.isnan(speed) or not np.isfinite(speed):
                return ''
            speed = int(max(min(speed, 100.), -100.))
            command = f'T;{speed}\r'
            return command
        except (TypeError, ValueError):
            return ''

    @staticmethod
    def steer_cmd(steer: float) -> str:
        try:
            if np.isnan(steer) or not np.isfinite(steer):
                return ''
            steer = int(max(min(steer, 100.), -100.))
            command = f'S;{steer}\r'
            return command
        except (TypeError, ValueError):
            return ''

    def send_cmd(self,
                 speed: Optional[float] = None,
                 steer: Optional[float] = None) -> None:

        # check if both speed and steer are None
        if speed is None and steer is None:
            return

        if self.check_writeable():
            cmd_speed = self.speed_cmd(speed) if speed is not None else ''
            cmd_steer = self.steer_cmd(steer) if steer is not None else ''
            command = cmd_speed + cmd_steer
            self.ser.write(command.encode())
            self.steer = steer if steer is not None else self.steer
            self.speed = speed if speed is not None else self.speed

    def speed_cmd_callback(self, msg: CarCommand) -> None:
        self.send_cmd(speed=msg.speed)

    def steer_cmd_callback(self, msg: CarCommand) -> None:
        self.send_cmd(steer=msg.steer)

    def car_cmd_callback(self, msg: CarCommand) -> None:
        self.send_cmd(msg.speed, msg.steer)

    def steer_to_rad(self, steer: float) -> float:
        return self.steer_params[0] + steer * self.steer_params[1]

    def get_odom_msg(self):
        if not (self.ser and self.ser.readable()):
            return None, None
        try:
            msg = self.ser.readline().decode('utf-8')
            wheel, delta_t = msg.split(';')
            delta_t = float(delta_t) / 1e6
            if wheel in ('L', 'R') and delta_t > 0:
                return wheel, delta_t
        except Exception as e:
            self.get_logger().error(f"Strange message: {e}")
        return None, None

    def run(self) -> None:
        # Read serial data
        while self.ser.in_waiting > 0:

            # Get the wheel and delta_t
            wheel, delta_t = self.get_odom_msg()
            if delta_t and wheel == 'R':  # We only need to read one wheel

                # Compute the distance traveled
                sgn = np.sign(self.speed) if self.speed != 0 else 1
                ds = np.pi * 2 * self.wheel_radius / self.num_magnets * sgn

                # Construct the message
                msg = CarMeasurement()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.v = ds / delta_t
                msg.df = self.steer_to_rad(self.steer)

                # Publish the message
                self.car_pub_.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CasCar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
