# Third-party ROS packages

This directory contains vendored upstream projects that are built as part of
the ROS workspace but are maintained independently from the Cascar packages.

- `mocap4r2_msgs`: motion-capture message definitions.
- `mocap`: motion-capture lifecycle, visualization, CLI, and support packages.
- `mocap4ros2_qualisys`: the Qualisys ROS driver and its C++ SDK.
- `sllidar_ros2`: the Slamtec lidar driver and SDK.

Keep upstream package names and internal layouts unchanged so that colcon and
ROS dependency resolution continue to work. Cascar-owned packages live directly
under `src/`.
