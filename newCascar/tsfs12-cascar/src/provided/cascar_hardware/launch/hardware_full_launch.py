"""Launch all physical Cascar hardware, including the optional LiDAR."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    launch_dir = Path(get_package_share_directory("cascar_hardware")) / "launch"

    agent_name = LaunchConfiguration("agent_name")
    lidar_model = LaunchConfiguration("lidar_model")
    arduino_port = LaunchConfiguration("arduino_port")
    lidar_port = LaunchConfiguration("lidar_port")
    vehicle_config_file = LaunchConfiguration("vehicle_config_file")
    default_vehicle_config = PathJoinSubstitution(
        [FindPackageShare("cascar_interfaces"), "config", "vehicle_parameters.yaml"],
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "hardware_launch.py"),
        ),
        launch_arguments={
            "agent_name": agent_name,
            "arduino_port": arduino_port,
            "vehicle_config_file": vehicle_config_file,
        }.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_dir / "lidar_launch.py"),
        ),
        launch_arguments={
            "agent_name": agent_name,
            "lidar_model": lidar_model,
            "lidar_port": lidar_port,
        }.items(),
    )

    return LaunchDescription([
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
            "arduino_port",
            default_value="/dev/arduino",
            description="Arduino serial device",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/rplidar",
            description="LiDAR serial device",
        ),
        DeclareLaunchArgument(
            "vehicle_config_file",
            default_value=default_vehicle_config,
            description="ROS parameters shared by the car, estimator, and simulator",
        ),
        core,
        lidar,
    ])
