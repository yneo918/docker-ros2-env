# syntax=docker/dockerfile:1.5

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# Install basic build tools for ROS 2 workspaces
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions \
       python3-pip \
       build-essential \
       git \
       ros-${ROS_DISTRO}-demo-nodes-cpp \
       ros-${ROS_DISTRO}-xacro \
       ros-${ROS_DISTRO}-rviz2 \
       ros-${ROS_DISTRO}-joy \
       gosu \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies specified by the host project
ENV PIP_BREAK_SYSTEM_PACKAGES=1
COPY requirements.txt /tmp/pip/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/pip/requirements.txt

# Copy our lightweight entrypoint wrapper that sources the host workspace if available
COPY docker/ros2_entrypoint.sh /usr/local/bin/ros2_entrypoint.sh
RUN chmod +x /usr/local/bin/ros2_entrypoint.sh

WORKDIR /workspaces/host_ws

ENTRYPOINT ["/usr/local/bin/ros2_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]
