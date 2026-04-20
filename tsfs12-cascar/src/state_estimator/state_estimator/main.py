from __future__ import annotations

from collections import deque
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from mocap4r2_msgs.msg import RigidBodies
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.stats import chi2
from sensor_msgs.msg import LaserScan

from cascar_msgs.msg import CarMeasurement, SensorBias

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.time import Time
    from rclpy.timer import Timer


TWO_S_NANO = 2_000_000_000


class EKFStateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("ekf_state_estimator")

        # Declare parameters
        self.declare_parameter("verbose", value=False)
        self.declare_parameter("qualisys_used", value=False)
        self.declare_parameter("agent_name", value="")
        self.declare_parameter("const_acc", value=False)

        # Get parameters
        self.verbose: bool = (
            self.get_parameter("verbose").get_parameter_value().bool_value
        )
        self.qualisys_used: bool = (
            self.get_parameter("qualisys_used").get_parameter_value().bool_value
        )
        self.const_acc_mdl: bool = (
            self.get_parameter("const_acc").get_parameter_value().bool_value
        )
        agent_name: str = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )
        namespace: str = agent_name + "/" if agent_name else ""
        self.agent_name: str = agent_name or "cascar"

        self.waiting_for_data: bool = self.qualisys_used

        # Subscribers
        self.qual_subscription_: Subscription = self.create_subscription(
            RigidBodies,
            "/rigid_bodies",
            self.qualisys_callback,
            1,
        )  # "/" to make topic global
        self.imu_subscription_: Subscription = self.create_subscription(
            CarMeasurement,
            "sensor/imu",
            self.imu_callback,
            1,
        )
        self.car_subscription_: Subscription = self.create_subscription(
            CarMeasurement,
            "sensor/cascar",
            self.car_callback,
            1,
        )
        self.lidar_subscription_: Subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_callback,
            1,
        )

        # Publishers
        self.odom_publisher_: Publisher = self.create_publisher(
            Odometry,
            "odom",
            100,
        )
        self.laser_publisher_: Publisher = self.create_publisher(
            LaserScan,
            "scan",
            100,
        )
        self.bias_publisher_: Publisher = self.create_publisher(
            SensorBias,
            "sensor/bias",
            100,
        )

        # TF broadcaster frame IDs
        self.header_frame_id: str = namespace + "odom"
        self.child_frame_id: str = namespace + "base_link"
        self.laser_frame_id: str = namespace + "laser_link"

        # Initialize state vector [x, y, qw, qz, v, a, bw]
        self.num_states: int = 7
        self.state: np.ndarray = np.zeros(self.num_states)
        self.state[2] = 1.0  # Initialize quaternion to identity rotation

        # Initialize covariance matrix
        self.P: np.ndarray = np.diag([
            2.0,
            2.0,
            0.1,
            0.1,
            1e-3,
            1e-3,
            1e-2,
        ])  # [x, y, qw, qz, v, a, bw]

        # Process noise
        self.Q: np.ndarray = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 1e-3, 1e-5])

        # Measurement noise
        self.R_qualisys: np.ndarray = np.eye(4) * 1e-4  # [x, y, qw, qz]
        self.R_velocity: np.ndarray = np.array([[0.01]])
        self.a_max: float = 3.0  # Maximum acceleration

        # Timer setup
        # self.dt: float = 0.1
        self.dt: float = 0.01
        self.timer: Timer = self.create_timer(
            self.dt,
            self.timer_callback,
        )

        # Latest measurements
        self.latest_w: float = 0.0
        self.latest_v: float = 0.0
        self.latest_v_time: Time | None = None

        # Stop detection setup
        self.use_accelerometer: bool = True
        self.window_size: int = 10
        self.acc_buffer: deque = deque(maxlen=self.window_size)
        self.acc_threshold: float = 0.6  # 0.7 for panther
        self.is_stopped: bool = True
        self.stop_counter: int = 0
        self.stop_confidence: int = 3
        self.running_avg: float = 0.0
        self.threshold_reset: bool = False

    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions (restricted to 2D yaw only)"""
        w1, z1 = q1
        w2, z2 = q2
        return np.array([w1 * w2 - z1 * z2, w1 * z2 + z1 * w2])

    def quaternion_jacobian(self, w: float) -> np.ndarray:
        """Compute the quaternion rate matrix for given angular velocity"""
        return np.array([[0, -w], [w, 0]])

    def quaternion_rot_mat(self, qw, qz) -> np.ndarray:
        """Compute the 2D rotation matrix from quaternion"""
        return np.array([
            [qw**2 - qz**2, -2 * qw * qz],
            [2 * qw * qz, qw**2 - qz**2],
        ])

    def process_model(self, state: np.ndarray, dt: float) -> np.ndarray:
        """Process model for the reduced quaternion-based EKF
        state = [x, y, qw, qz, v, a, bw]
        """
        x, y = state[0:2]
        q = state[2:4]  # [qw, qz]
        v, a, bw = state[4:]
        w = self.latest_w + bw

        # Position update using current quaternion
        R = self.quaternion_rot_mat(q[0], q[1])
        velocity_world = R @ np.array([v, 0])
        dx, dy = velocity_world
        dv = a if self.const_acc_mdl else 0.0
        da = 0.0
        dbw = 0.0

        # Quaternion update using angular velocity
        w_quat = np.array([0, w])  # Angular velocity as quaternion
        dq = 0.5 * self.quaternion_multiply(w_quat, q)

        # State update equations (Euler integration)
        new_state = np.zeros_like(state)
        new_state[0] = x + dx * dt
        new_state[1] = y + dy * dt
        new_state[2:4] = q + dq * dt
        new_state[4] = v + dv * dt
        new_state[5] = a + da * dt
        new_state[6] = bw + dbw * dt

        # Normalize quaternion
        q_norm = np.linalg.norm(new_state[2:4])
        new_state[2:4] /= q_norm

        return new_state

    def calculate_jacobian(self, state: np.ndarray) -> np.ndarray:
        """Compute the Jacobian of the process model"""
        F = np.eye(self.num_states)
        q = state[2:4]
        v = state[4]
        bw = state[6]
        w = self.latest_w + bw

        # Position derivatives with respect to quaternion components
        F[0:2, 2:4] = self.dt * np.array([
            [2 * v * q[0], -2 * v * q[1]],
            [2 * v * q[1], 2 * v * q[0]],
        ])

        # Quaternion derivative
        F[2:4, 2:4] += self.dt * 0.5 * self.quaternion_jacobian(w)

        # Quaternion derivatives with respect to bias
        F[2:4, 6] = (self.dt * 0.5) * np.array([-q[1], q[0]])

        return F

    def qualisys_measurement_update(
        self,
        x: float,
        y: float,
        quat: Quaternion,
    ) -> None:
        """Update step for Qualisys measurements using 2D quaternion"""
        H = np.zeros((4, self.num_states))
        H[0:2, 0:2] = np.eye(2)  # Position measurements
        H[2:4, 2:4] = np.eye(2)  # Quaternion (yaw only) measurements

        z = np.array([x, y, quat.w, quat.z])
        z_pred = H @ self.state

        # Handle quaternion sign ambiguity
        if np.dot(z[2:4], z_pred[2:4]) < 0:
            z[2:4] = -z[2:4]

        y_np = z - z_pred
        S = H @ self.P @ H.T + self.R_qualisys
        K = self.P @ H.T @ np.linalg.inv(S)

        new_state = self.state + K @ y_np
        new_state[2:4] /= np.linalg.norm(new_state[2:4])  # Normalize quaternion

        if not (np.isnan(new_state).any() or np.isinf(new_state).any()):
            self.state = new_state
            self.P = (np.eye(self.num_states) - K @ H) @ self.P
        elif self.verbose:
            self.get_logger().warn("Invalid state after update from Qualisys")

    def qualisys_callback(self, msg: RigidBodies) -> None:
        for rigid_body in msg.rigidbodies:
            # Filter out data from other agents
            if rigid_body.rigid_body_name == self.agent_name:
                position = rigid_body.pose.position
                quaternion = rigid_body.pose.orientation
                self.qualisys_measurement_update(position.x, position.y, quaternion)

                if self.waiting_for_data:
                    self.get_logger().info("Data received from Qualisys")
                    self.waiting_for_data = False

                # Exit loop after finding the correct agent
                return

    def broadcast_odom(self) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.header_frame_id  # 'odom'
        odom.child_frame_id = self.child_frame_id  # 'base_link'

        x, y = self.state[0], self.state[1]
        q = self.state[2:4]
        v = self.state[4]
        w = self.latest_w + self.state[6]  # Yaw rate + bias

        # Position
        odom.pose.pose.position = Point(x=x, y=y, z=0.0)

        # Orientation
        odom.pose.pose.orientation = Quaternion(
            w=q[0],
            x=0.0,
            y=0.0,
            z=q[1],  # Only qw, qz
        )

        # Rotation matrix
        R = self.quaternion_rot_mat(q[0], q[1])

        # Velocity
        vel_body = np.array([v, 0])  # v in body frame
        vel_world = R @ vel_body  # Apply 2D rotation

        # Set Twist values
        odom.twist.twist = Twist(
            linear=Vector3(x=vel_world[0], y=vel_world[1], z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=w),
        )

        self.odom_publisher_.publish(odom)

    def detect_stop(self, ax: float, ay: float) -> None:
        """Detect if vehicle is stopped using filtered accelerometer data"""
        # Calculate total acceleration
        total_acc = (ax * ax + ay * ay) ** 0.5

        # Update moving average buffer - deque handles size automatically
        self.acc_buffer.append(total_acc)

        # Calculate filtered acceleration
        if len(self.acc_buffer) > 0:
            avg_acc = sum(self.acc_buffer) / len(self.acc_buffer)
        else:
            avg_acc = float("inf")  # Assume not stopped

        self.running_avg = avg_acc

        # Stop detection logic with hysteresis
        if avg_acc < self.acc_threshold:
            self.stop_counter += 1
            if self.stop_counter >= self.stop_confidence and not self.is_stopped:
                if self.verbose:
                    self.get_logger().info("Vehicle stopped detected")
                self.is_stopped = True
                self.state[4] = 0.0  # Set velocity to zero
                self.state[5] = 0.0  # Set acceleration to zero
                self.P[4, 4] = 0.01  # Increase certainty about zero velocity
                self.P[5, 5] = 0.001  # Increase certainty about zero acceleration
        else:
            self.stop_counter = 0
            if self.is_stopped and self.verbose:
                self.get_logger().info("Vehicle motion detected")
            self.is_stopped = False

    def imu_callback(self, msg: CarMeasurement) -> None:
        self.latest_w = msg.w
        if self.use_accelerometer:
            self.detect_stop(msg.a_x, msg.a_y)

    def time_update(self) -> None:
        """EKF prediction step"""
        # Skip state update if stopped
        if not self.is_stopped:
            self.state = self.process_model(self.state, self.dt)

        # Calculate Jacobian
        F = self.calculate_jacobian(self.state)

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    @staticmethod
    def outlier_detection(
        innovation: np.ndarray,
        s: np.ndarray,
        dof: int,
        confidence_level: float = 0.95,
    ) -> bool:
        """Validate measurement using chi-square test.

        Args:
            innovation: Innovation vector (y = z - z_pred)
            S: Innovation covariance matrix
            dof: Degrees of freedom (dimension of measurement)
            confidence_level: Desired confidence level (default 0.95 or 95%)

        Returns:
            bool: True if measurement is valid, False otherwise

        """
        # Compute Mahalanobis distance
        d2 = innovation.T @ np.linalg.inv(s) @ innovation

        # Get chi-square threshold for given confidence level and DOF
        threshold = chi2.ppf(confidence_level, dof)

        return d2 <= threshold

    def speed_measurement_update(self, v: float) -> None:
        """EKF update step for speed measurements with statistical validation"""
        H = np.zeros((1, self.num_states))
        H[0, 4] = 1.0  # speed

        # Compute innovation
        innovation = np.array([v - self.state[4]])

        # Compute innovation covariance
        S = H @ self.P @ H.T + self.R_velocity

        # Validate measurement
        if self.outlier_detection(innovation, S, dof=1):
            # Proceed with update if measurement is valid
            K = self.P @ H.T @ np.linalg.inv(S)
            self.state = self.state + K @ innovation
            self.P = (np.eye(self.num_states) - K @ H) @ self.P
        elif self.verbose:
            self.get_logger().warn("Speed measurement rejected by validation gate")

    def car_callback(self, msg: CarMeasurement) -> None:
        self.latest_v = msg.v
        if self.const_acc_mdl:
            self.speed_measurement_update(msg.v)
        else:
            self.state[4] = msg.v

        # get time of latest velocity measurement
        self.latest_v_time = self.get_clock().now()
        self.threshold_reset = True

    def lidar_callback(self, msg: LaserScan) -> None:
        # Fix timing issues by copying the scan message
        scan_msg = msg

        # Update the header with new frame and time
        scan_msg.header.frame_id = self.laser_frame_id  # 'laser_link'
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the new scan message
        self.laser_publisher_.publish(scan_msg)

    def timer_callback(self) -> None:
        if self.waiting_for_data:
            self.get_logger().info("Waiting for data from Qualisys...")
            return
        self.time_update()
        self.broadcast_odom()

        msg = SensorBias()
        msg.bias = self.state[6]
        self.bias_publisher_.publish(msg)

        # Check if the latest velocity measurement is older than 2.0 seconds
        if self.latest_v_time is not None:
            diff = self.get_clock().now() - self.latest_v_time
            if diff.nanoseconds > TWO_S_NANO and self.threshold_reset:
                self.acc_threshold = self.running_avg + 0.05
                if self.verbose:
                    self.get_logger().info(
                        f"Setting acc_threshold to {self.acc_threshold}",
                    )
                self.threshold_reset = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EKFStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
