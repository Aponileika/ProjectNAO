from pathlib import Path

import tomllib
from demo.utils import with_qualisys
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Manual control launch description.

    Returns:
        LaunchDescription: The launch description.

    """
    agent_name = LaunchConfiguration("agent_name")
    with Path("config.toml").open("rb") as f:
        data = tomllib.load(f)

    default_agent_name: str = data["agent"]["name"]
    use_qualisys: bool = data["qualisys"]["use"]
    qualisys_args: list[LaunchDescriptionEntity] = (
        with_qualisys() if use_qualisys else []
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_name",
                default_value=default_agent_name,
                description="Specify the name of the robot",
            ),
            Node(
                package="manual_control",
                namespace=agent_name,
                executable="manual_control",
                name="keyboard_control",
            ),
            *qualisys_args,
        ],
    )
