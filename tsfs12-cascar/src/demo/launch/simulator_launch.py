from pathlib import Path

import tomllib
from demo.utils import (
    robot_state_publisher_node,
    rviz_node,
    static_transform_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the simulator.

    Returns:
        LaunchDescription: The launch description.

    """
    agent_name = LaunchConfiguration("agent_name")

    with Path("config.toml").open("rb") as f:
        data = tomllib.load(f)

    rviz_file = data["rviz"]["file"]
    default_agent_name: str = data["agent"]["name"]

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value=default_agent_name,
            description="Specify the name of the robot",
        ),
        Node(
            package="state_publisher",
            executable="state_publisher",
            name="state_publisher",
            namespace=agent_name,
            parameters=[{"agent_name": agent_name}],
        ),
        Node(
            package="car_tracker",
            executable="car_tracker",
            name="car_tracker",
            namespace=agent_name,
            parameters=[{"agent_name": agent_name}],
        ),
        Node(
            package="simulator",
            executable="multi_simulator",
            name="multi_simulator",
            namespace=agent_name,
            parameters=[{"agent_name": agent_name}],
        ),
        # Below are ROS2 nodes that are needed for the visualization
        static_transform_publisher_node(),
        robot_state_publisher_node(agent_name),
    ])

    if rviz_file and (node := rviz_node(rviz_file)) is not None:
        launch_description.add_entity(node)

    if data.get("simulator_demo", {}).get("launch_manual_control", False):
        launch_description.add_entity(
            Node(
                package="manual_control",
                executable="manual_control",
                name="keyboard_control",
                namespace=agent_name,
            ),
        )

    return launch_description
