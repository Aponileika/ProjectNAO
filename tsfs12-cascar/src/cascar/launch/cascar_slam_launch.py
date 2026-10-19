import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_config = LaunchConfiguration("slam_params_file")

    rviz_config = LaunchConfiguration("rviz_config")

    default_slam_config = "/tmp/cascar_slam.yaml"

    default_rviz_config = os.path.join(
        get_package_share_directory("cascar"),
        "rviz",
        "default.rviz",
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_config,
            description="SLAM Toolbox configuration file",
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz configuration file",
        ),

        # Temporary odometry-to-base transform.
        # All six values are zero.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="slam_initial_odom_transform",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "odom",
                "--child-frame-id", "base_link",
            ],
            output="screen",
        ),

        # SLAM Toolbox
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_config,
                {"use_sim_time": False},
            ],
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                {"use_sim_time": False},
            ],
        ),

    ])
