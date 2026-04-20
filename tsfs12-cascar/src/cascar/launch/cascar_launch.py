from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value="",
            description="Specify the name of the robot",
        ),

        Node(
            package="cascar",
            namespace=agent_name,  # Ensure a unique namespace for each robot
            executable="cascar",
            name="cascar",
        ),
        Node(
            package="cascar",
            namespace=agent_name,
            executable="imu",
            name="imu",
        ),
    ])
