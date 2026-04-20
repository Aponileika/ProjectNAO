from __future__ import annotations

import time
from pathlib import Path
from typing import TYPE_CHECKING

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros2node.api import get_node_names

if TYPE_CHECKING:
    from collections.abc import Iterator

    from launch import LaunchDescriptionEntity


def with_qualisys() -> list[LaunchDescriptionEntity]:
    """Add the Qualisys launch file to the launch description.

    Will be empty if qualisys node is already used.

    Returns:
        list[LaunchDescriptionEntity]: A list containing qualisys launch
            description entities.

    """
    qualisys_launch_dir = get_package_share_directory("qualisys_driver")
    qualisys_launch_file = (
        Path(qualisys_launch_dir) / "launch" / "qualisys.launch.py"
    )

    def is_qualisys_node(node_name: tuple[str, str, str]) -> bool:
        name, _, _ = node_name
        return name == "qualisys_driver_node"

    # If the Qualisys node is active, we don't need to include it again
    if len(list(filter(is_qualisys_node, active_ros2_nodes()))) > 0:
        return []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(qualisys_launch_file)),
        ),
    ]


def static_transform_publisher_node() -> Node:
    """Create a static transform publisher node.

    Returns:
        Node: The static transform publisher node.

    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map2world",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "world_points",
        ],
    )


def rviz_node(rviz_file: str) -> Node | None:
    """Create an RViz node.

    Returns:
        Node: The RViz node or None if RViz is already running.

    """

    def is_rviz_node(node_name: tuple[str, str, str]) -> bool:
        name, _, _ = node_name
        return name == "rviz2"

    if len(list(filter(is_rviz_node, active_ros2_nodes()))) > 0:
        return None

    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )


def robot_state_publisher_node(agent_name: LaunchConfiguration) -> Node:
    """Create a robot state publisher node.

    Args:
        agent_name (str): The name of the agent.

    Returns:
        Node: The robot state publisher node.

    """
    urdf_path = PathJoinSubstitution([
        get_package_share_directory("cascar"),
        "model",
        [agent_name, "_cascar.urdf"],
    ])

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=agent_name,
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_path]),
                "publish_frequency": 100.0,
                "tf_prefix": agent_name,
            },
        ],
    )


def active_ros2_nodes() -> Iterator[tuple[str, str, str]]:
    """Get the names and namespaces of active ROS 2 nodes.

    Yields:
        Iterator[tuple[str, str, str]]: (name, namespace, fullname)

    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node("list_nodes")
    # It wont work without a small delay, as it takes some time to discover
    # nodes
    time.sleep(1)
    yield from get_node_names(node=node)
