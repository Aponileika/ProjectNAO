from time import sleep

import rclpy
from rclpy.node import Node

from cascar_msgs.msg import CarMeasurement
from .sensors.intertial_measurement_unit import InertialMeasurementUnit as IMU


class IMUNode(Node):
    def __init__(self, imu_instance = None) -> None:
        super().__init__('imu')
        self.get_logger().info("IMU Node Started")

        # Publishers
        self.imu_pub_ = self.create_publisher(CarMeasurement, 'sensor/imu', 10)

        # Initialize IMU
        self.imu = imu_instance

        if self.imu is None:
            self.get_logger().error("IMU not found. Exiting...")
            exit(1)

        # Initialize states
        self.yaw = 0.

        # Initialize timer
        fs = 100
        self.dt = 1/fs
        self.timer = self.create_timer(self.dt, self.run)

    def run(self) -> None:
        # Read IMU data
        ax, ay, _ = self.imu.get_accel_data()
        _, _, gz = self.imu.get_gyro_data()

        # Estimate the yaw angle
        self.yaw += gz * self.dt

        # Publish IMU data
        msg = CarMeasurement()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.a_x = ax
        msg.a_y = ay

        msg.w = gz
        msg.th = self.yaw

        self.imu_pub_.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    imu = None

    while rclpy.ok():
        try:
            imu = IMU(0x68)
        except OSError:
            print("Failed to connect to IMU. Retrying...")
            sleep(1)
        else:
            break

    node = IMUNode(imu)

    if node is not None:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
