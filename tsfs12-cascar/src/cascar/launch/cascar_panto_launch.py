from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_name = LaunchConfiguration("agent_name")

    basics_launch = PathJoinSubstitution([
        FindPackageShare("cascar"),
        "launch",
        "cascar_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value="",
            description="Specify the name of the robot",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basics_launch),
            launch_arguments={
                "agent_name": agent_name
            }.items()
        ),

        Node(
            package="controller",
            namespace=agent_name,
            executable="pursuit",
            name="pursuit",
        ),
        Node(
            package="pathPlanning",
            namespace=agent_name,
            executable="pathPlanning",
            name="pathPlanning",
        ),
        Node(
            package="localization",
            namespace=agent_name,
            executable="localization",
            name="localization",
        ),
        Node(
            package="diagnostic",
            namespace=agent_name,
            executable="diagnostic",
            name="diagnostic",
        ),
    ])