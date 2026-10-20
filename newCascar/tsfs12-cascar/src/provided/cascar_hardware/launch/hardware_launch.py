"""Launch the Arduino and IMU drivers for a physical Cascar vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    agent_name = LaunchConfiguration("agent_name")
    arduino_port = LaunchConfiguration("arduino_port")
    vehicle_config_file = LaunchConfiguration("vehicle_config_file")
    default_vehicle_config = PathJoinSubstitution(
        [FindPackageShare("cascar_interfaces"), "config", "vehicle_parameters.yaml"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value="",
            description="Specify the name of the robot",
        ),
        DeclareLaunchArgument(
            "arduino_port",
            default_value="/dev/arduino",
            description="Arduino serial device",
        ),
        DeclareLaunchArgument(
            "vehicle_config_file",
            default_value=default_vehicle_config,
            description="ROS parameters shared by the car, estimator, and simulator",
        ),
        Node(
            package="cascar_hardware",
            namespace=agent_name,  # Ensure a unique namespace for each robot
            executable="car_driver",
            name="car_driver",
            parameters=[
                vehicle_config_file,
                {
                    "serial_port": ParameterValue(
                        arduino_port,
                        value_type=str,
                    ),
                },
            ],
        ),
        Node(
            package="cascar_hardware",
            namespace=agent_name,
            executable="imu_driver",
            name="imu_driver",
            parameters=[vehicle_config_file],
        ),
    ])
