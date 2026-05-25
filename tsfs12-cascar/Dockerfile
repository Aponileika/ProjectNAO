# Use an official ROS2 Jazzy base image
FROM ghcr.io/sloretz/ros:jazzy-desktop

# Install necessary dependencies for vcs import and ROS workspace
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep only if it hasn't been initialized already
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && rosdep update

# Create the workspace directory
WORKDIR /cascar_ws

# Copy the source code
COPY src ./src

# Copy the config.yaml file
COPY config.toml config.toml

# Install any dependencies specified in package.xml
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Release'

# Source the setup script for future container sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /cascar_ws/install/setup.bash" >> ~/.bashrc

# Ensure `python` refers to `python3`
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Copy and install Python dependencies
# [This is not necessary if rosdep is set up correctly, and all package dependencies are specified in package.xml]
# COPY requirements.txt requirements.txt
# RUN pip3 install --break-system-packages -r requirements.txt

# Default command to run when container starts
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /cascar_ws/install/setup.bash && exec bash"]
