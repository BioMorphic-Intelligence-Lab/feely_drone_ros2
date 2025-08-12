# Build arguments
ARG ROS_DISTRO=jazzy

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Restate the arg to make it available in later stage
ARG ROS_DISTRO

# Install dependencies
RUN apt-get update && apt-get install -y \
    libboost-dev \
    python3-pip \
    build-essential \
    cmake \
    python-is-python3 \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install adafruit-circuitpython-mpr121 --break-system-packages
RUN pip3 install RPi.GPIO --break-system-packages

# Add copy of local workspace 
WORKDIR /home/user/ros
ADD ./src ./ws/src

WORKDIR /home/user/ros/ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select custom_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select px4_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select px4_interface
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select tactile_perch_ctrl

# Add the entrypoint script
ADD ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]