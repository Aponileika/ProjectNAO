#!/usr/bin/env python3

import asyncio
import math
import threading
import time

import numpy as np
import qtm
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster


class QualisysPoseNode(Node):

    def __init__(self):
        super().__init__("qualisys_pose_node")

        self.declare_parameter("qualisys_ip", "192.168.0.50")
        self.declare_parameter("body_index", 2)
        self.declare_parameter("publish_rate", 20.0)

        # TF frame configuration
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "turtle/base_link")

        # Qualisys pose corresponding to the desired origin.
        # Set these to the measured Qualisys coordinates of the
        # robot's starting position if the robot should start at 0,0,0.
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("origin_theta", 0.0)

        self.qualisys_ip = str(
            self.get_parameter("qualisys_ip").value
        )
        self.body_index = int(
            self.get_parameter("body_index").value
        )
        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        self.odom_frame = str(
            self.get_parameter("odom_frame").value
        )
        self.base_frame = str(
            self.get_parameter("base_frame").value
        )

        self.origin_x = float(
            self.get_parameter("origin_x").value
        )
        self.origin_y = float(
            self.get_parameter("origin_y").value
        )
        self.origin_theta = float(
            self.get_parameter("origin_theta").value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        if self.body_index < 0:
            raise ValueError("body_index cannot be negative")

        self.publish_rate = min(self.publish_rate, 20.0)
        self.min_period = 1.0 / self.publish_rate

        self.pose_publisher = self.create_publisher(
            Pose2D,
            "/qualisys_pose",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_publish_time = 0.0
        self.stop_event = threading.Event()

        self.qtm_thread = threading.Thread(
            target=self.run_qtm,
            daemon=True
        )
        self.qtm_thread.start()

        self.get_logger().info(
            f"Connecting to Qualisys at {self.qualisys_ip}"
        )
        self.get_logger().info(
            f"Publishing /qualisys_pose and TF "
            f"{self.odom_frame} -> {self.base_frame}"
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

            now = time.monotonic()

            if now - self.last_publish_time < self.min_period:
                return

            header, body = packet.get_6d_euler()

            if not body or self.body_index >= len(body):
                return

            pos, euler = body[self.body_index]

            # Qualisys position: millimeters -> meters.
            raw_x = pos.x / 1000.0
            raw_y = pos.y / 1000.0

            # Qualisys Euler angles: degrees -> radians.
            raw_theta = math.radians(euler[2])

            if not np.all(
                np.isfinite([raw_x, raw_y, raw_theta])
            ):
                return

            # Express the pose relative to the configured origin.
            dx = raw_x - self.origin_x
            dy = raw_y - self.origin_y

            c = math.cos(self.origin_theta)
            s = math.sin(self.origin_theta)

            x = c * dx + s * dy
            y = -s * dx + c * dy

            theta = math.atan2(
                math.sin(raw_theta - self.origin_theta),
                math.cos(raw_theta - self.origin_theta)
            )

            # Publish the relative planar pose.
            pose_msg = Pose2D()
            pose_msg.x = float(x)
            pose_msg.y = float(y)
            pose_msg.theta = float(theta)

            self.pose_publisher.publish(pose_msg)

            # Broadcast odom -> base_link.
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = float(x)
            tf_msg.transform.translation.y = float(y)
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = math.sin(theta / 2.0)
            tf_msg.transform.rotation.w = math.cos(theta / 2.0)

            self.tf_broadcaster.sendTransform(tf_msg)

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
