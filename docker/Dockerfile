# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

# The base image comes from the official ROS repository hosted on Docker Hub
# You can find available ROS images here: https://hub.docker.com/_/ros/tags
# We're using the ros-base image which includes core ROS 2 packages
FROM docker.io/library/ros:${ROS_DISTRO}-ros-base

# Set the maintainer information for this Dockerfile
LABEL maintainer="AutomaticAddison<automatic_addison@todo.com>"

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/root/ros2_ws

# Set the default shell to bash for RUN commands
# This ensures all RUN commands use bash instead of sh
SHELL ["/bin/bash", "-c"]

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    bash-completion \
    build-essential \
    curl \
    git \
    gosu \
    i2c-tools \
    iputils-ping \
    libgpiod2 \
    libserial-dev \
    locales \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-networkx \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    tzdata \
    usbutils && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    adafruit-blinka \
    adafruit-circuitpython-icm20x \
    adafruit-extended-bus

RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p ${ROS_WS}/src/rover/rover_description && \
    mkdir -p ${ROS_WS}/src/rover/rover_hardware && \
    mkdir -p ${ROS_WS}/src/rover/rover_navigation && \
    mkdir -p ${ROS_WS}/src/rover/baseboard/extra_packages/nmea_msgs

COPY rover/rover_description/package.xml ${ROS_WS}/src/rover/rover_description/package.xml
COPY rover/rover_hardware/package.xml ${ROS_WS}/src/rover/rover_hardware/package.xml
COPY rover/rover_navigation/package.xml ${ROS_WS}/src/rover/rover_navigation/package.xml
COPY rover/baseboard/extra_packages/nmea_msgs/package.xml ${ROS_WS}/src/rover/baseboard/extra_packages/nmea_msgs/package.xml

# Copy configuration files from docker directory
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

WORKDIR /root

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${ROS_WS} && apt-get update && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y && rm -rf /var/lib/apt/lists/*"

# Copy the entire project into the container after dependency resolution so
# source-only edits can reuse the rosdep layer.
COPY rover ${ROS_WS}/src/rover

RUN ROS_SKIP_DEP_INSTALL=1 ROS_SKIP_ROSDEP_UPDATE=1 ./workspace.sh

# Ensure the ROS 2 workspace is sourced in every new shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint for the container
# This script will be run every time the container starts
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
# This keeps the container running indefinitely, allowing you to exec into it
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
