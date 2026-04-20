import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_launch_dir = get_package_share_directory("sllidar_ros2")
    lidar_launch_file = os.path.join(lidar_launch_dir, "launch", "sllidar_a2m8_launch.py")
    # lidar_launch_file = os.path.join(lidar_launch_dir, 'launch', 'sllidar_a3_launch.py')  # for A3

    agent_name = LaunchConfiguration("agent_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value="",
            description="Specify the name of the robot",
        ),

        # Include the Lidar launch file and pass the agent_name argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            launch_arguments={"agent_name": agent_name}.items(),
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
