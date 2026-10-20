"""Launch the manual controller in the configured vehicle namespace."""

from cascar.config import TomlConfigValue, default_config_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the manual controller for the selected vehicle."""
    agent_name = LaunchConfiguration("agent_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config_path()),
                description="Path to the Cascar TOML configuration",
            ),
            DeclareLaunchArgument(
                "agent_name",
                default_value=TomlConfigValue("agent", "name"),
                description="Vehicle namespace",
            ),
            Node(
                package="cascar_examples",
                executable="manual_controller",
                namespace=agent_name,
                output="screen",
            ),
        ],
    )
