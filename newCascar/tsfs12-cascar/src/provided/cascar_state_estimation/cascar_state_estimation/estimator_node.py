"""ROS node for planar Cascar state estimation."""

from __future__ import annotations

from collections import deque
from math import isfinite

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped
from mocap4r2_msgs.msg import RigidBodies
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from cascar_state_estimation.ekf import PlanarEkf


class EKFStateEstimator(Node):
    """Fuse wheel speed, yaw rate, and optional Qualisys measurements."""

    def __init__(self) -> None:
        super().__init__("ekf_state_estimator")

        self.declare_parameter("verbose", False)
        self.declare_parameter("qualisys_used", False)
        self.declare_parameter("agent_name", "")
        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter(
            "initial_covariance",
            [2.0, 2.0, 0.1, 0.1, 1e-3, 1e-2],
        )
        self.declare_parameter(
            "process_noise_rates",
            [0.1, 0.1, 0.1, 0.1, 0.1, 1e-5],
        )
        self.declare_parameter("qualisys_position_variance", 1e-4)
        self.declare_parameter("qualisys_orientation_variance", 1e-4)
        self.declare_parameter("wheel_velocity_variance", 0.01)
        self.declare_parameter("imu_yaw_rate_variance", 0.01)
        self.declare_parameter("measurement_gate_confidence", 0.95)
        self.declare_parameter("use_accelerometer_stop_detection", True)
        self.declare_parameter("stop_window_size", 10)
        self.declare_parameter("stop_acceleration_threshold", 0.6)
        self.declare_parameter("stop_confidence_samples", 3)
        self.declare_parameter("stale_wheel_seconds", 2.0)
        self.declare_parameter("stop_threshold_margin", 0.05)
        self.declare_parameter("stopped_velocity_variance", 0.01)
        self.declare_parameter("qualisys_wait_log_period_sec", 5.0)

        self.verbose = bool(self.get_parameter("verbose").value)
        self.qualisys_used = bool(self.get_parameter("qualisys_used").value)
        agent_name = str(self.get_parameter("agent_name").value)
        namespace = f"{agent_name}/" if agent_name else ""
        self.agent_name = agent_name or "cascar"
        self.header_frame_id = namespace + "odom"
        self.child_frame_id = namespace + "base_link"

        self.qualisys_position_variance = self._positive_parameter(
            "qualisys_position_variance",
        )
        self.qualisys_orientation_variance = self._positive_parameter(
            "qualisys_orientation_variance",
        )
        self.wheel_velocity_variance = self._positive_parameter(
            "wheel_velocity_variance",
        )
        self.imu_yaw_rate_variance = self._positive_parameter(
            "imu_yaw_rate_variance",
        )
        self.gate_confidence = float(
            self.get_parameter("measurement_gate_confidence").value,
        )
        if not 0.0 < self.gate_confidence < 1.0:
            raise ValueError("measurement_gate_confidence must be between 0 and 1")

        initial_covariance = np.asarray(
            self.get_parameter("initial_covariance").value,
            dtype=float,
        )
        process_noise_rates = np.asarray(
            self.get_parameter("process_noise_rates").value,
            dtype=float,
        )
        self.ekf = PlanarEkf(initial_covariance, process_noise_rates)

        update_rate_hz = self._positive_parameter("update_rate_hz")
        self.use_accelerometer = bool(
            self.get_parameter("use_accelerometer_stop_detection").value,
        )
        self.stop_window_size = max(
            1,
            int(self.get_parameter("stop_window_size").value),
        )
        self.acc_threshold = self._nonnegative_parameter(
            "stop_acceleration_threshold",
        )
        self.stop_confidence = max(
            1,
            int(self.get_parameter("stop_confidence_samples").value),
        )
        self.stale_wheel_seconds = self._positive_parameter("stale_wheel_seconds")
        self.stop_threshold_margin = self._nonnegative_parameter(
            "stop_threshold_margin",
        )
        self.stopped_velocity_variance = self._positive_parameter(
            "stopped_velocity_variance",
        )
        self.qualisys_wait_log_period_sec = self._positive_parameter(
            "qualisys_wait_log_period_sec",
        )

        self.waiting_for_data = self.qualisys_used
        self.latest_yaw_rate = 0.0
        self.latest_yaw_rate_variance = self.imu_yaw_rate_variance
        self.latest_wheel_time_ns: int | None = None
        self.wheel_initialized = False
        self.last_prediction_ns = self.get_clock().now().nanoseconds
        self.threshold_reset = False
        self.acc_buffer: deque[float] = deque(maxlen=self.stop_window_size)
        self.is_stopped = True
        self.stop_counter = 0
        self.running_avg = 0.0

        self.qual_subscription = None
        if self.qualisys_used:
            self.qual_subscription = self.create_subscription(
                RigidBodies,
                "/rigid_bodies",
                self.qualisys_callback,
                1,
            )
        self.imu_subscription = self.create_subscription(
            Imu,
            "imu/data_raw",
            self.imu_callback,
            1,
        )
        self.car_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            "wheel/twist",
            self.car_callback,
            1,
        )
        self.odom_publisher = self.create_publisher(Odometry, "odom", 100)
        self.timer = self.create_timer(1.0 / update_rate_hz, self.timer_callback)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def qualisys_callback(self, message: RigidBodies) -> None:
        """Fuse the configured vehicle's Qualisys rigid body."""
        for rigid_body in message.rigidbodies:
            if rigid_body.rigid_body_name != self.agent_name:
                continue
            position = rigid_body.pose.position
            orientation = rigid_body.pose.orientation
            if self.waiting_for_data:
                accepted = self.ekf.initialize_pose(
                    position.x,
                    position.y,
                    orientation.w,
                    orientation.z,
                    self.qualisys_position_variance,
                    self.qualisys_orientation_variance,
                )
            else:
                accepted = self.ekf.update_pose(
                    position.x,
                    position.y,
                    orientation.w,
                    orientation.z,
                    self.qualisys_position_variance,
                    self.qualisys_orientation_variance,
                    self.gate_confidence,
                )
            if not accepted:
                if self.verbose:
                    self.get_logger().warning("Rejected invalid Qualisys measurement")
                return
            if self.waiting_for_data:
                self.get_logger().info("Data received from Qualisys")
                self.waiting_for_data = False
                self.last_prediction_ns = self.get_clock().now().nanoseconds
            return

    def car_callback(self, message: TwistWithCovarianceStamped) -> None:
        """Fuse a wheel-speed measurement and its reported variance."""
        velocity = message.twist.twist.linear.x
        if not isfinite(velocity):
            if self.verbose:
                self.get_logger().warning("Rejected non-finite wheel speed")
            return
        variance = message.twist.covariance[0]
        if not isfinite(variance) or variance <= 0.0:
            variance = self.wheel_velocity_variance
        if self.wheel_initialized:
            accepted = self.ekf.update_velocity(
                velocity,
                variance,
                self.gate_confidence,
            )
        else:
            accepted = self.ekf.initialize_velocity(velocity, variance)
            self.wheel_initialized = accepted
        if not accepted and self.verbose:
            self.get_logger().warning("Rejected wheel-speed measurement")
        self.latest_wheel_time_ns = self.get_clock().now().nanoseconds
        self.threshold_reset = True

    def detect_stop(self, acceleration_x: float, acceleration_y: float) -> None:
        """Detect stationary periods from horizontal acceleration magnitude."""
        if not (isfinite(acceleration_x) and isfinite(acceleration_y)):
            return
        self.acc_buffer.append((acceleration_x**2 + acceleration_y**2) ** 0.5)
        self.running_avg = sum(self.acc_buffer) / len(self.acc_buffer)

        if self.running_avg < self.acc_threshold:
            self.stop_counter += 1
            if self.stop_counter >= self.stop_confidence and not self.is_stopped:
                self.is_stopped = True
                if self.verbose:
                    self.get_logger().info("Vehicle stop detected")
        else:
            self.stop_counter = 0
            if self.is_stopped and self.verbose:
                self.get_logger().info("Vehicle motion detected")
            self.is_stopped = False

    def imu_callback(self, message: Imu) -> None:
        """Record yaw rate and feed acceleration to the stop detector."""
        if isfinite(message.angular_velocity.z):
            self.latest_yaw_rate = message.angular_velocity.z
        variance = message.angular_velocity_covariance[8]
        self.latest_yaw_rate_variance = (
            variance
            if isfinite(variance) and variance > 0.0
            else self.imu_yaw_rate_variance
        )
        if self.use_accelerometer:
            self.detect_stop(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
            )

    @staticmethod
    def _set_unknown_covariances(covariance: list[float]) -> None:
        for index in (2, 3, 4):
            covariance[index * 6 + index] = 1e6

    def broadcast_odom(self) -> None:
        """Publish the current state and mapped covariance."""
        state = self.ekf.state
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.header_frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = state[PlanarEkf.X]
        odom.pose.pose.position.y = state[PlanarEkf.Y]
        odom.pose.pose.orientation = Quaternion(
            w=state[PlanarEkf.QW],
            x=0.0,
            y=0.0,
            z=state[PlanarEkf.QZ],
        )
        odom.twist.twist.linear.x = state[PlanarEkf.VELOCITY]
        odom.twist.twist.angular.z = self.latest_yaw_rate - state[PlanarEkf.GYRO_BIAS]

        planar_covariance = self.ekf.planar_pose_covariance()
        ros_indices = (0, 1, 5)
        for row, ros_row in enumerate(ros_indices):
            for column, ros_column in enumerate(ros_indices):
                odom.pose.covariance[ros_row * 6 + ros_column] = planar_covariance[
                    row, column
                ]
        self._set_unknown_covariances(odom.pose.covariance)

        odom.twist.covariance[0] = self.ekf.covariance[
            PlanarEkf.VELOCITY,
            PlanarEkf.VELOCITY,
        ]
        odom.twist.covariance[35] = (
            self.latest_yaw_rate_variance
            + self.ekf.covariance[PlanarEkf.GYRO_BIAS, PlanarEkf.GYRO_BIAS]
        )
        self._set_unknown_covariances(odom.twist.covariance)
        odom.twist.covariance[7] = 1e6
        self.odom_publisher.publish(odom)

    def timer_callback(self) -> None:
        """Predict, publish, and maintain the adaptive stop threshold."""
        now_ns = self.get_clock().now().nanoseconds
        if self.waiting_for_data:
            self.get_logger().info(
                "Waiting for data from Qualisys...",
                throttle_duration_sec=self.qualisys_wait_log_period_sec,
            )
            self.last_prediction_ns = now_ns
            return

        if self.latest_wheel_time_ns is not None:
            wheel_age = (now_ns - self.latest_wheel_time_ns) / 1e9
            if wheel_age > self.stale_wheel_seconds:
                if self.threshold_reset:
                    self.acc_threshold = self.running_avg + self.stop_threshold_margin
                    self.threshold_reset = False
                    if self.verbose:
                        self.get_logger().info(
                            f"Setting stop acceleration threshold to {self.acc_threshold}",
                        )
                # Quiet acceleration alone also occurs at constant speed. Only
                # constrain speed once wheel events cease, and keep doing so
                # even if a final encoder sample arrived after stop detection.
                if (
                    self.use_accelerometer
                    and self.is_stopped
                    and self.stop_counter >= self.stop_confidence
                ):
                    self.ekf.constrain_velocity_to_zero(
                        self.stopped_velocity_variance,
                    )

        dt = (now_ns - self.last_prediction_ns) / 1e9
        self.last_prediction_ns = now_ns
        if dt > 0.0:
            self.ekf.predict(self.latest_yaw_rate, dt)
        self.broadcast_odom()


def main(args: list[str] | None = None) -> None:
    """Run the Cascar state estimator."""
    rclpy.init(args=args)
    node = EKFStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
