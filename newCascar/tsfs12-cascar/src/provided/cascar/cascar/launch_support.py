"""Supporting visualization and motion-capture actions for the project launch."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from ament_index_python.packages import get_package_share_directory
from launch import Substitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node

if TYPE_CHECKING:
    from launch.conditions import Condition


def qualisys_launch(condition: Condition | None = None) -> IncludeLaunchDescription:
    """Include the Qualisys driver when motion capture is requested."""
    launch_dir = Path(get_package_share_directory("qualisys_driver")) / "launch"
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "qualisys.launch.py")),
        condition=condition,
    )


def static_transform_publisher_node(condition: Condition | None = None) -> Node:
    """Align simulator map coordinates with the visualization's world frame."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map2world",
        output="screen",
        condition=condition,
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "world_points",
        ],
    )


def rviz_node(
    rviz_file: str | Substitution,
    agent_name: LaunchConfiguration,
    condition: Condition | None = None,
) -> Node:
    """Open a packaged RViz configuration in the selected vehicle namespace."""
    rviz_path = PathJoinSubstitution(
        [get_package_share_directory("cascar_visualization"), "rviz", rviz_file],
    )

    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=agent_name,
        output="screen",
        arguments=["-d", rviz_path],
        condition=condition,
    )


def robot_state_publisher_node(
    agent_name: LaunchConfiguration,
    condition: Condition | None = None,
) -> Node:
    """Expand the selected car's Xacro wrapper and publish its link transforms."""
    urdf_path = PathJoinSubstitution(
        [
            get_package_share_directory("cascar_visualization"),
            "urdf",
            PythonExpression(["'", agent_name, "' + '_cascar.urdf'"]),
        ],
    )

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=agent_name,
        condition=condition,
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_path]),
                "publish_frequency": 100.0,
                "tf_prefix": agent_name,
            },
        ],
    )
