# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

# The base image comes from the official ROS repository hosted on Docker Hub
# You can find available ROS images here: https://hub.docker.com/_/ros/tags
# We're using the ros-base image which includes core ROS 2 packages
FROM ros:${ROS_DISTRO}-ros-base

# Set the maintainer information for this Dockerfile
LABEL maintainer="AutomaticAddison<automatic_addison@todo.com>"

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive

# Set the default shell to bash for RUN commands
# This ensures all RUN commands use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Install only the tools needed to build and run the workspace.
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    build-essential \
    curl \
    git \
    libserial-dev \
    locales \
    lsb-release \
    wget

# Install additional tools required for ROS 2 development.
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    gnupg2 \
    iputils-ping \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-networkx \
    python3-rosdep \
    python3-vcstool \
    usbutils

# Set up the ROS 2 environment
# This ensures that ROS 2 commands are available in the shell
# rosdep is a tool for installing system dependencies for ROS packages
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Create necessary directories
RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p /root/ros2_ws/src/rover

# Install the ROS packages needed to build and launch the rover workspace.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-nmea-navsat-driver \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-xacro

# Copy the entire project into the container
COPY rover /root/ros2_ws/src/rover

# Copy configuration files from docker directory
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

WORKDIR /root

# Ensure the ROS 2 workspace is sourced in every new shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint for the container
# This script will be run every time the container starts
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
# This keeps the container running indefinitely, allowing you to exec into it
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
