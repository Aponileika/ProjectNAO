
#!/usr/bin/env python3

import asyncio
import threading
import time

import numpy as np
import qtm
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class QualisysPoseNode(Node):

    def __init__(self):
        super().__init__("qualisys_pose_node")

        # ROS 2 parameters
        self.declare_parameter("qualisys_ip", "192.168.0.50")
        self.declare_parameter("body_index", 2)
        self.declare_parameter("publish_rate", 20.0)

        self.qualisys_ip = str(
            self.get_parameter("qualisys_ip").value
        )
        self.body_index = int(
            self.get_parameter("body_index").value
        )
        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        if self.body_index < 0:
            raise ValueError("body_index cannot be negative")

        # Never exceed 20 Hz
        self.publish_rate = min(self.publish_rate, 20.0)
        self.min_period = 1.0 / self.publish_rate

        # Publisher
        self.publisher = self.create_publisher(
            Pose2D,
            "qualisys_pose",
            10
        )

        self.last_publish_time = 0.0
        self.stop_event = threading.Event()

        # Run QTM's asyncio client in a separate thread
        self.qtm_thread = threading.Thread(
            target=self.run_qtm,
            daemon=True
        )
        self.qtm_thread.start()

        self.get_logger().info(
            f"Connecting to Qualisys at {self.qualisys_ip}"
        )
        self.get_logger().info(
            f"Publishing Pose2D on /qualisys_pose "
            f"at a maximum of {self.publish_rate:.1f} Hz"
        )

    def run_qtm(self):
        try:
            asyncio.run(self.qualisys_data())
        except Exception as e:
            if not self.stop_event.is_set():
                self.get_logger().error(
                    f"Qualisys connection error: {e}"
                )

    async def qualisys_data(self):
        connection = await qtm.connect(self.qualisys_ip)

        if connection is None:
            self.get_logger().error(
                "Failed to connect to Qualisys QTM"
            )
            return

        self.get_logger().info("Connected to Qualisys QTM")

        def on_packet(packet):
            if self.stop_event.is_set():
                return

            # Limit packet processing and publishing rate
            now = time.monotonic()

            if now - self.last_publish_time < self.min_period:
                return

            header, body = packet.get_6d_euler()

            if not body or self.body_index >= len(body):
                return

            # Extract selected rigid body's position and orientation
            pos, euler = body[self.body_index]

            # Convert millimeters to meters and degrees to radians
            x = pos.x / 1000.0
            y = pos.y / 1000.0
            theta = np.deg2rad(euler[2])

            # Do not publish NaN or infinite values
            if not np.all(np.isfinite([x, y, theta])):
                return

            # Create and publish Pose2D
            msg = Pose2D()
            msg.x = float(x)
            msg.y = float(y)
            msg.theta = float(theta)

            self.publisher.publish(msg)
            self.last_publish_time = now

        try:
            self.get_logger().info(
                "Starting Qualisys 6D Euler stream"
            )

            await connection.stream_frames(
                components=["6deuler"],
                on_packet=on_packet
            )

            while (
                rclpy.ok()
                and not self.stop_event.is_set()
            ):
                await asyncio.sleep(0.1)

        finally:
            try:
                await connection.stream_frames_stop()
            except Exception:
                pass

            self.get_logger().info(
                "Qualisys streaming stopped"
            )

    def destroy_node(self):
        self.stop_event.set()

        if self.qtm_thread.is_alive():
            self.qtm_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QualisysPoseNode()

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
