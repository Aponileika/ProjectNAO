from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    lidar_launch_dir = Path(get_package_share_directory("sllidar_ros2"))
    agent_name = LaunchConfiguration("agent_name")
    lidar_model = LaunchConfiguration("lidar_model")
    lidar_port = LaunchConfiguration("lidar_port")

    def lidar_launch(model: str) -> IncludeLaunchDescription:
        launch_file = lidar_launch_dir / "launch" / f"sllidar_{model}_launch.py"
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            condition=IfCondition(
                PythonExpression(["'", lidar_model, f"' == '{model}'"]),
            ),
            launch_arguments={
                "agent_name": agent_name,
                "frame_id": PythonExpression(
                    [
                        "'",
                        agent_name,
                        "' + '/laser_link' if '",
                        agent_name,
                        "' else 'laser_link'",
                    ],
                ),
                "serial_port": lidar_port,
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_name",
                default_value="",
                description="Specify the name of the robot",
            ),
            DeclareLaunchArgument(
                "lidar_model",
                default_value="a2m8",
                choices=["a2m8", "a3"],
                description="LiDAR model",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/rplidar",
                description="LiDAR serial device",
            ),
            lidar_launch("a2m8"),
            lidar_launch("a3"),
        ]
    )


"""Launch the LiDAR driver for a physical Cascar vehicle."""
