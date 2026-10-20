"""Launch the provided Cascar platform with a simulator or vehicle backend."""

from cascar.config import TomlConfigValue, default_config_path
from cascar.launch_support import (
    qualisys_launch,
    robot_state_publisher_node,
    rviz_node,
    static_transform_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the common project-facing runtime around the selected backend."""
    agent_name = LaunchConfiguration("agent_name")
    backend = LaunchConfiguration("backend")
    visualization = LaunchConfiguration("visualization")
    qualisys_used = LaunchConfiguration("qualisys_used")
    rviz_file = LaunchConfiguration("rviz_file")
    vehicle_config_file = LaunchConfiguration("vehicle_config_file")
    default_vehicle_config = PathJoinSubstitution(
        [FindPackageShare("cascar_interfaces"), "config", "vehicle_parameters.yaml"],
    )

    simulator = IfCondition(EqualsSubstitution(backend, "simulator"))
    vehicle = IfCondition(EqualsSubstitution(backend, "vehicle"))
    use_qualisys = IfCondition(
        AndSubstitution(EqualsSubstitution(backend, "vehicle"), qualisys_used),
    )
    show_visualization = IfCondition(visualization)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config_path()),
                description="Path to the Cascar TOML configuration",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value=TomlConfigValue("backend", "type"),
                choices=["simulator", "vehicle"],
                description="Select the simulated or physical vehicle data source",
            ),
            DeclareLaunchArgument(
                "agent_name",
                default_value=TomlConfigValue("agent", "name"),
                description="Robot namespace and frame prefix",
            ),
            DeclareLaunchArgument(
                "visualization",
                default_value=TomlConfigValue("rviz", "enabled"),
                choices=["true", "false"],
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value=TomlConfigValue("rviz", "file"),
                description="RViz configuration filename",
            ),
            DeclareLaunchArgument(
                "qualisys_used",
                default_value=TomlConfigValue("qualisys", "use"),
                choices=["true", "false"],
                description="Use Qualisys measurements in vehicle mode",
            ),
            DeclareLaunchArgument(
                "vehicle_config_file",
                default_value=default_vehicle_config,
                description="ROS parameters shared by the car, estimator, and simulator",
            ),
            # The student computer runs one backend. Hardware drivers are
            # started separately by the Raspberry Pi's deployed image.
            Node(
                package="cascar_simulator",
                executable="simulator",
                name="simulator",
                namespace=agent_name,
                parameters=[vehicle_config_file, {"agent_name": agent_name}],
                condition=simulator,
            ),
            Node(
                package="cascar_state_estimation",
                executable="state_estimator",
                name="state_estimator",
                namespace=agent_name,
                parameters=[
                    vehicle_config_file,
                    {
                        "qualisys_used": ParameterValue(
                            qualisys_used,
                            value_type=bool,
                        ),
                        "agent_name": agent_name,
                    },
                ],
                condition=vehicle,
            ),
            Node(
                package="cascar_visualization",
                executable="odom_tf_publisher",
                name="odom_tf_publisher",
                namespace=agent_name,
                parameters=[{"agent_name": agent_name}],
            ),
            robot_state_publisher_node(agent_name),
            static_transform_publisher_node(condition=simulator),
            qualisys_launch(condition=use_qualisys),
            rviz_node(
                rviz_file,
                agent_name,
                condition=show_visualization,
            ),
        ],
    )
