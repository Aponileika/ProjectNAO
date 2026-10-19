import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    cascar_share = get_package_share_directory('cascar')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    # Robot description
    urdf_path = os.path.join(
        cascar_share,
        'model',
        'turtle_cascar.urdf'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # SLAM configuration
    slam_params_file = os.path.join(
        cascar_share,
        'config',
        'cascar_slam.yaml'
    )

    # Existing robot and LiDAR launch
    robot_launch_file = os.path.join(
        cascar_share,
        'launch',
        'cascar_full_launch.py'
    )

    # Standard SLAM Toolbox launch
    slam_launch_file = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([

        # 1. Start existing robot, IMU and LiDAR nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_file),
        ),

        # 2. Publish the robot's fixed URDF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': False,
                }
            ],
        ),

        # 3. Temporary initial odometry transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='initial_odom_to_base',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'turtle/base_link',
            ],
        ),

        # 4. Temporary LiDAR mounting transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'turtle/base_link',
                '--child-frame-id', 'laser',
            ],
        ),

        # 5. Start SLAM Toolbox without RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
