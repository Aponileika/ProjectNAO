from pathlib import Path

import tomllib
from demo.utils import robot_state_publisher_node, rviz_node, with_qualisys
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """State estimation launch description.

    Raises:
        RuntimeError: If Qualisys is required but not used.

    Returns:
        LaunchDescription: The launch description.

    """
    agent_name = LaunchConfiguration("agent_name")
    qualisys_used = LaunchConfiguration("qualisys_used")
    const_acc = LaunchConfiguration("const_acc")

    with Path("config.toml").open("rb") as f:
        data = tomllib.load(f)

    default_agent_name: str = data["agent"]["name"]
    use_qualisys: bool = data["qualisys"]["use"]
    qualisys_args: list[LaunchDescriptionEntity] = (
        with_qualisys() if use_qualisys else []
    )
    rviz_file: str = data["rviz"]["file"]

    estim_use_qualisys: bool = data["state_estimator_demo"]["use_qualisys"]
    estim_use_const_acc: bool = data["state_estimator_demo"][
        "use_constant_acceleration"
    ]
    visualization: bool = data["state_estimator_demo"]["visualization"]
    launch_manual_control: bool = data["state_estimator_demo"][
        "launch_manual_control"
    ]

    if estim_use_qualisys and not use_qualisys:
        msg = "qualisys is required for the state estimator demo, but it is not being used."
        raise RuntimeError(msg)

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            "agent_name",
            default_value=default_agent_name,
            description="Specify the name of the robot",
        ),
        DeclareLaunchArgument(
            "qualisys_used",
            default_value="true" if estim_use_qualisys else "false",
            description="Specify whether Qualisys system is used",
        ),
        DeclareLaunchArgument(
            "const_acc",
            default_value="true" if estim_use_const_acc else "false",
            description="Specify whether we should use a constant acceleration model or not",
        ),
        Node(
            package="state_publisher",
            executable="state_publisher",
            name="state_publisher",
            namespace=agent_name,
            parameters=[{"agent_name": agent_name}],
        ),
        Node(
            package="car_tracker",
            executable="car_tracker",
            name="car_tracker",
            namespace=agent_name,
            parameters=[{"agent_name": agent_name}],
        ),
        Node(
            package="state_estimator",
            executable="state_estimator",
            namespace=agent_name,
            name="state_estimator",
            parameters=[
                {
                    "qualisys_used": qualisys_used,
                    "agent_name": agent_name,
                    "const_acc": const_acc,
                },
            ],
        ),
        robot_state_publisher_node(agent_name),
        *qualisys_args,
    ])

    if visualization and (node := rviz_node(rviz_file)) is not None:
        launch_description.add_entity(node)

    if launch_manual_control:
        launch_description.add_entity(
            Node(
                package="manual_control",
                namespace=agent_name,
                executable="manual_control",
                name="keyboard_control",
            ),
        )

    return launch_description
