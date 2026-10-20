ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS builder

SHELL ["/bin/bash", "-c"]
WORKDIR /cascar_ws

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

COPY src/provided/cascar_interfaces src/provided/cascar_interfaces
COPY src/provided/cascar_hardware src/provided/cascar_hardware
COPY src/third_party/sllidar_ros2 src/third_party/sllidar_ros2

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build \
    --merge-install \
    --packages-up-to cascar_hardware sllidar_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:${ROS_DISTRO}-ros-base AS runtime

ARG VCS_REF=unknown
LABEL org.opencontainers.image.title="CASCAR car runtime" \
    org.opencontainers.image.revision="${VCS_REF}"

SHELL ["/bin/bash", "-c"]

# Keep package manifests temporarily so rosdep can install only runtime dependencies.
COPY src/provided/cascar_interfaces /tmp/cascar_src/provided/cascar_interfaces
COPY src/provided/cascar_hardware /tmp/cascar_src/provided/cascar_hardware
COPY src/third_party/sllidar_ros2 /tmp/cascar_src/third_party/sllidar_ros2

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install \
    --from-paths /tmp/cascar_src \
    --ignore-src \
    --dependency-types exec \
    -r -y \
    && rm -rf /var/lib/apt/lists/* /tmp/cascar_src /root/.ros/rosdep

COPY --from=builder /cascar_ws/install /opt/cascar
COPY docker/car/entrypoint.sh /usr/local/bin/cascar-entrypoint

ENV CASCAR_NAME="" \
    CASCAR_LIDAR_MODEL="a2m8" \
    CASCAR_ARDUINO_PORT="/dev/arduino" \
    CASCAR_LIDAR_PORT="/dev/rplidar"

ENTRYPOINT ["/usr/local/bin/cascar-entrypoint"]
