"""Compatibility imports; use cascar.config or cascar.launch_support in new code."""

from cascar.config import TomlConfigValue, default_config_path, load_config
from cascar.launch_support import (
    qualisys_launch,
    robot_state_publisher_node,
    rviz_node,
    static_transform_publisher_node,
)

__all__ = [
    "TomlConfigValue",
    "default_config_path",
    "load_config",
    "robot_state_publisher_node",
    "rviz_node",
    "static_transform_publisher_node",
    "with_qualisys",
]


def with_qualisys(condition=None):
    """Preserve the original list-returning helper for existing launch files."""
    return [qualisys_launch(condition=condition)]
