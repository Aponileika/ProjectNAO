"""ROS node that publishes MPU acceleration and yaw-rate measurements."""

from __future__ import annotations

from typing import Protocol

from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUSensor(Protocol):
    """Sensor operations required by :class:`IMUNode`."""

    def get_accel_data(self) -> tuple[float, float, float]:
        """Return acceleration along all three axes."""
        ...

    def get_gyro_data(self) -> tuple[float, float, float]:
        """Return angular velocity along all three axes."""
        ...

    def close(self) -> None:
        """Close the sensor connection."""
        ...


class IMUNode(Node):
    """Publish raw accelerometer and gyroscope measurements."""

    def __init__(self, sensor: IMUSensor) -> None:
        """Configure ROS interfaces around an initialized IMU sensor."""
        super().__init__("imu_driver")
        self.get_logger().info("IMU node started")
        self.sensor = sensor
        self.declare_parameter("sample_frequency_hz", 100.0)
        self.declare_parameter("angular_velocity_variance", 0.01)
        self.declare_parameter("linear_acceleration_variance", 0.1)
        sample_frequency = max(
            1.0,
            float(self.get_parameter("sample_frequency_hz").value),
        )
        self.angular_velocity_variance = max(
            0.0,
            float(self.get_parameter("angular_velocity_variance").value),
        )
        self.linear_acceleration_variance = max(
            0.0,
            float(self.get_parameter("linear_acceleration_variance").value),
        )
        self.dt = 1 / sample_frequency
        namespace = self.get_namespace().strip("/")
        prefix = f"{namespace}/" if namespace else ""
        self.frame_id = prefix + "base_link"
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.timer = self.create_timer(self.dt, self.run)

    def run(self) -> None:
        """Read and publish one IMU sample."""
        accel_x, accel_y, accel_z = self.sensor.get_accel_data()
        gyro_x, gyro_y, gyro_z = self.sensor.get_gyro_data()

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.orientation_covariance[0] = -1.0
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = self.angular_velocity_variance
            message.linear_acceleration_covariance[index] = (
                self.linear_acceleration_variance
            )
        message.angular_velocity.x = gyro_x
        message.angular_velocity.y = gyro_y
        message.angular_velocity.z = gyro_z
        message.linear_acceleration.x = accel_x
        message.linear_acceleration.y = accel_y
        message.linear_acceleration.z = accel_z
        self.imu_publisher.publish(message)

    def destroy_node(self) -> None:
        """Close the sensor before destroying the ROS node."""
        self.sensor.close()
        super().destroy_node()
