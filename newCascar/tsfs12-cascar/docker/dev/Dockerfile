ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Package manifests are copied separately so source edits do not invalidate
# the dependency installation layer.
COPY src /tmp/cascar_src

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install \
    --from-paths /tmp/cascar_src \
    --ignore-src \
    --skip-keys qualisys_cpp_sdk \
    -r -y \
    && rm -rf /var/lib/apt/lists/* /tmp/cascar_src /root/.ros/rosdep

ARG DEV_USER=developer
ARG DEV_UID=1000
ARG DEV_GID=1000

RUN set -euo pipefail; \
    existing_user="$(getent passwd "${DEV_UID}" | cut -d: -f1 || true)"; \
    if [[ -n "${existing_user}" && "${existing_user}" != "${DEV_USER}" ]]; then \
        userdel --remove "${existing_user}"; \
    fi; \
    if ! getent group "${DEV_GID}" >/dev/null; then \
        groupadd --gid "${DEV_GID}" "${DEV_USER}"; \
    fi; \
    useradd \
        --uid "${DEV_UID}" \
        --gid "${DEV_GID}" \
        --create-home \
        --shell /bin/bash \
        "${DEV_USER}"; \
    echo "${DEV_USER} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${DEV_USER}"; \
    chmod 0440 "/etc/sudoers.d/${DEV_USER}"; \
    printf '\nsource "/opt/ros/%s/setup.bash"\nif [[ -f /workspace/install/setup.bash ]]; then\n    source /workspace/install/setup.bash\nfi\n' \
        "${ROS_DISTRO}" >> "/home/${DEV_USER}/.bashrc"

ENV HOME=/home/${DEV_USER}

WORKDIR /workspace
USER ${DEV_USER}

CMD ["sleep", "infinity"]
