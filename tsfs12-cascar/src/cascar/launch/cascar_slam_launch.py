import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription([

        DeclareLaunchArgument(
            "slam_params_file",
            default_value="/tmp/cascar_slam.yaml",
            description="Path to SLAM Toolbox parameters",
        ),

        # Publish the robot's fixed and articulated link transforms.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": open(
                    "/cascar_ws/src/cascar/model/turtle_cascar.urdf"
                ).read(),
                "use_sim_time": False,
            }],
        ),

        # Connect the lidar frame to the robot base.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "turtle/base_link", "laser",
            ],
            output="screen",
        ),

        # Publish the initial odom -> base transform using Qualisys.
        Node(
            package='qualisys',
            executable='qualisys',
            name='qualisys_odom',
            output='screen',
            parameters=[{
                'qualisys_ip': '192.168.0.50',
                'body_index': 2,
                'publish_rate': 20.0,
            }],
        ),


        # SLAM Toolbox.
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params_file,
                {"use_sim_time": False},
            ],
        ),

    ])
