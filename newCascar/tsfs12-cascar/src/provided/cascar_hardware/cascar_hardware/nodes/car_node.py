"""ROS node for motor commands and wheel odometry."""

from __future__ import annotations

from collections.abc import Callable
from math import isfinite, pi
from time import monotonic, sleep

import serial
from cascar_interfaces.msg import ActuatorCommand
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cascar_hardware.drivers.arduino import ArduinoDriver, SerialConnectionError


class CasCarNode(Node):
    """Bridge ROS commands and measurements to the Arduino controller."""

    def __init__(
        self,
        driver_factory: Callable[..., ArduinoDriver] = ArduinoDriver.connect,
    ) -> None:
        """Connect to the controller and configure ROS interfaces."""
        super().__init__("car_driver")
        self.get_logger().info("CasCar node started")
        self.declare_parameter("serial_port", "/dev/arduino")
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("max_throttle_percent", 100.0)
        self.declare_parameter("max_steering_percent", 95.0)
        self.declare_parameter("startup_steering_sweep_enabled", value=False)
        self.declare_parameter("startup_steering_sweep_percent", 30.0)
        self.declare_parameter("startup_settle_sec", 2.0)
        self.declare_parameter("startup_steering_hold_sec", 1.0)
        self.declare_parameter("wheel_radius_m", 0.039)
        self.declare_parameter("wheel_magnets", 10)
        self.declare_parameter("steering_offset_rad", 0.02840205)
        self.declare_parameter("steering_scale_rad_per_percent", 0.00335285)
        self.declare_parameter("max_steering_angle_rad", 0.35)
        self.declare_parameter("wheel_velocity_variance", 0.01)
        port = str(self.get_parameter("serial_port").value)
        self.command_timeout_sec = max(
            0.0,
            float(self.get_parameter("command_timeout_sec").value),
        )
        self.max_throttle_percent = max(
            0.0,
            float(self.get_parameter("max_throttle_percent").value),
        )
        self.max_steering_percent = max(
            0.0,
            float(self.get_parameter("max_steering_percent").value),
        )
        self.wheel_radius_m = max(
            0.0,
            float(self.get_parameter("wheel_radius_m").value),
        )
        self.wheel_magnets = max(1, int(self.get_parameter("wheel_magnets").value))
        self.steering_offset_rad = float(
            self.get_parameter("steering_offset_rad").value,
        )
        self.steering_scale_rad_per_percent = float(
            self.get_parameter("steering_scale_rad_per_percent").value,
        )
        self.max_steering_angle_rad = max(
            0.0,
            float(self.get_parameter("max_steering_angle_rad").value),
        )
        self.wheel_velocity_variance = max(
            0.0,
            float(self.get_parameter("wheel_velocity_variance").value),
        )
        self.driver = self._connect(driver_factory, port)

        self.steer = 0.0
        self.speed = 0.0
        self.last_command_time: float | None = None
        self.watchdog_applied: bool = True
        namespace = self.get_namespace().strip("/")
        prefix = f"{namespace}/" if namespace else ""
        self.base_link_frame = prefix + "base_link"
        self.steering_joint_names = [
            prefix + "base_to_front_left_wheel",
            prefix + "base_to_front_right_wheel",
        ]
        self.joint_state_publisher = self.create_publisher(
            JointState,
            "joint_states",
            1,
        )
        self._run_start_sequence()

        self.command_subscription = self.create_subscription(
            ActuatorCommand,
            "actuator_command",
            self.car_cmd_callback,
            1,
        )
        self.wheel_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            "wheel/twist",
            1,
        )
        self.timer = self.create_timer(1 / 100, self.run)

    def _connect(
        self,
        driver_factory: Callable[..., ArduinoDriver],
        port: str,
    ) -> ArduinoDriver:
        for attempt in range(2):
            try:
                driver = driver_factory(port=port, baud_rate=115200)
            except SerialConnectionError as error:  # noqa: PERF203
                if attempt == 1:
                    raise
                self.get_logger().warning(f"{error}. Retrying once...")
                sleep(2)
            else:
                self.get_logger().info(f"Connected to serial port {port}")
                return driver
        msg = f"Failed to connect to {port}"
        raise SerialConnectionError(msg)

    def _run_start_sequence(self) -> None:
        if not bool(self.get_parameter("startup_steering_sweep_enabled").value):
            self.get_logger().info("Startup steering sweep disabled")
            return

        settle_sec = max(0.0, float(self.get_parameter("startup_settle_sec").value))
        hold_sec = max(
            0.0,
            float(self.get_parameter("startup_steering_hold_sec").value),
        )
        sweep_percent = min(
            self.max_steering_percent,
            abs(float(self.get_parameter("startup_steering_sweep_percent").value)),
        )
        self.get_logger().info("Waiting for communications to settle...")
        sleep(settle_sec)
        self.get_logger().info("Sending start sequence...")
        self.send_command(steer=sweep_percent)
        sleep(hold_sec)
        self.send_command(steer=-sweep_percent)
        sleep(hold_sec)
        self.send_command(steer=0)
        self.get_logger().info("Start sequence completed")

    def send_command(
        self,
        speed: float | None = None,
        steer: float | None = None,
    ) -> None:
        """Send optional speed and steering percentages to the controller."""
        result = self.driver.send(speed=speed, steer=steer)
        if result is None:
            return
        if result.speed is not None:
            self.speed = result.speed
        if result.steer is not None:
            self.steer = result.steer

    def car_cmd_callback(self, message: ActuatorCommand) -> None:
        """Map a normalized ROS command to the Arduino controller."""
        if not (isfinite(message.throttle) and isfinite(message.steering)):
            self.get_logger().warning(
                "Ignoring actuator command containing a non-finite value",
                throttle_duration_sec=2.0,
            )
            return
        self.send_command(
            speed=self._map_normalized(message.throttle, self.max_throttle_percent),
            steer=self._map_normalized(message.steering, self.max_steering_percent),
        )
        self.last_command_time = monotonic()
        self.watchdog_applied = False

    @staticmethod
    def _map_normalized(value: float, maximum: float) -> float:
        """Map a normalized value to a symmetric target range."""
        return max(-1.0, min(1.0, value)) * maximum

    def publish_joint_states(self) -> None:
        """Publish steering joints represented by the last command."""
        steering_angle = (
            self.steering_offset_rad
            + self.steer * self.steering_scale_rad_per_percent
        )
        steering_angle = max(
            -self.max_steering_angle_rad,
            min(self.max_steering_angle_rad, steering_angle),
        )
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.steering_joint_names
        message.position = [steering_angle, steering_angle]
        self.joint_state_publisher.publish(message)

    def run(self) -> None:
        """Check the watchdog and publish a bounded batch of encoder samples."""
        if (
            not self.watchdog_applied
            and self.last_command_time is not None
            and monotonic() - self.last_command_time > self.command_timeout_sec
        ):
            self.send_command(speed=0.0, steer=0.0)
            self.watchdog_applied = True
            self.get_logger().warning("Actuator command timed out; stopping the car")

        self.publish_joint_states()
        # Yield to command callbacks even if serial input arrives continuously.
        for _ in range(32):
            if self.driver.in_waiting <= 0:
                break
            reading = self.driver.read_encoder()
            if reading is None or reading.wheel != "R":
                continue
            if reading.delta_seconds <= 0.0:
                self.get_logger().warning(
                    "Ignoring encoder sample with a non-positive time delta",
                    throttle_duration_sec=2.0,
                )
                continue

            # The encoder does not report direction. Infer it from the latest
            # throttle command; external rolling and uncommanded reversals cannot
            # be represented correctly by this sensor interface.
            direction = -1 if self.speed < 0 else 1
            distance = 2 * pi * self.wheel_radius_m / self.wheel_magnets * direction
            message = TwistWithCovarianceStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.base_link_frame
            message.twist.twist.linear.x = distance / reading.delta_seconds
            message.twist.covariance[0] = self.wheel_velocity_variance
            self.wheel_publisher.publish(message)

    def destroy_node(self) -> None:
        """Stop the car and close its serial connection before shutdown."""
        try:
            self.send_command(speed=0.0)
            self.driver.close()
        except serial.SerialException:
            self.get_logger().error("Failed to stop the car during shutdown")
        super().destroy_node()
