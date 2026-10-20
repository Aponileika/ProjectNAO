"""Console entry points for the CasCar hardware nodes."""

from __future__ import annotations

from time import sleep

import rclpy

from cascar_hardware.drivers.mpu6050 import MPU6050
from cascar_hardware.nodes.car_node import CasCarNode
from cascar_hardware.nodes.imu_node import IMUNode


def main(args: list[str] | None = None) -> None:
    """Run the Arduino-backed car node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CasCarNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def imu_main(args: list[str] | None = None) -> None:
    """Run the MPU-6050-backed IMU node."""
    rclpy.init(args=args)
    node = None
    try:
        while rclpy.ok():
            try:
                sensor = MPU6050(address=0x68)
                node = IMUNode(sensor)
            except OSError:  # noqa: PERF203
                print("Failed to connect to IMU. Retrying...")
                sleep(1)
            else:
                break

        if node is not None:
            rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
