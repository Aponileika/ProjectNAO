"""Resolve student project settings from cascar.toml and launch arguments."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import tomllib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, Substitution
from launch.substitutions import LaunchConfiguration


def default_config_path() -> Path:
    """Resolve the default project configuration path."""
    environment_path = os.environ.get("CASCAR_CONFIG_FILE")
    if environment_path:
        return Path(environment_path).expanduser()

    workspace_path = Path.cwd() / "cascar.toml"
    if workspace_path.is_file():
        return workspace_path

    return Path(get_package_share_directory("cascar")) / "config" / "cascar.toml"


def load_config(config_file: str | Path | None = None) -> dict[str, Any]:
    """Load an explicit configuration or the resolved project default."""
    config_file = (
        Path(config_file).expanduser()
        if config_file is not None
        else default_config_path()
    )
    with config_file.open("rb") as file:
        return tomllib.load(file)


class TomlConfigValue(Substitution):
    """Resolve a launch default from the selected ``config_file``."""

    def __init__(self, *keys: str) -> None:
        super().__init__()
        self.keys = keys

    def describe(self) -> str:
        """Describe which configuration value supplies the launch default."""
        return f"config_file[{'.'.join(self.keys)}]"

    def perform(self, context: LaunchContext) -> str:
        value: Any = load_config(LaunchConfiguration("config_file").perform(context))
        for key in self.keys:
            value = value[key]
        return str(value).lower() if isinstance(value, bool) else str(value)
