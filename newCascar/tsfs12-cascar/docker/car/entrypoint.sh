#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/cascar/setup.bash
set -u

if [[ $# -eq 0 ]]; then
  if [[ -z "${CASCAR_NAME}" ]]; then
    echo "CASCAR_NAME must be set" >&2
    exit 2
  fi
  if [[ "${CASCAR_LIDAR_MODEL}" != "a2m8" && "${CASCAR_LIDAR_MODEL}" != "a3" ]]; then
    echo "CASCAR_LIDAR_MODEL must be a2m8 or a3" >&2
    exit 2
  fi

  set -- ros2 launch cascar_hardware hardware_full_launch.py \
    "agent_name:=${CASCAR_NAME}" \
    "lidar_model:=${CASCAR_LIDAR_MODEL}" \
    "arduino_port:=${CASCAR_ARDUINO_PORT}" \
    "lidar_port:=${CASCAR_LIDAR_PORT}"
fi

exec "$@"
