# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

# The base image comes from the official ROS repository hosted on Docker Hub
# You can find available ROS images here: https://hub.docker.com/_/ros/tags
# We're using the ros-base image which includes core ROS 2 packages
FROM ros:${ROS_DISTRO}-ros-base

# Set timezone and ensure time sync with host
ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ntpdate to sync time with host
RUN apt-get update -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false && \
    apt-get install -y --no-install-recommends ntpdate && \
    ntpdate -s time.nist.gov && \
    rm -rf /var/lib/apt/lists/*

# Set the maintainer information for this Dockerfile
LABEL maintainer="AutomaticAddison<automatic_addison@todo.com>"

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive

# Set the default shell to bash for RUN commands
# This ensures all RUN commands use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update the system and install essential tools
# This step upgrades all packages and installs utilities needed for development
# Install essential packages
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    apt-utils wget curl git build-essential \
    vim sudo lsb-release locales bash-completion tzdata gosu gedit htop nano libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools required for ROS 2 development
# These packages help with building and managing ROS 2 workspaces
RUN apt-get update -q && \
    apt-get install -y gnupg2 iputils-ping usbutils \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-pip python3-rosdep python3-vcstool

# Set up the ROS 2 environment
# This ensures that ROS 2 commands are available in the shell
# rosdep is a tool for installing system dependencies for ROS packages
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Install additional ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-reconfigure

# Install FastDDS v3 and its dependencies
RUN apt-get update -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false && \
    apt-get install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    libyaml-cpp-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-chrono-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-atomic-dev \
    libboost-date-time-dev \
    libssl3 \
    libssl-dev \
    git \
    cmake \
    g++ \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install foonathan_memory (required by Fast-DDS)
WORKDIR /tmp
RUN git clone https://github.com/foonathan/memory.git foonathan_memory_vendor \
    && mkdir -p foonathan_memory_vendor/build \
    && cd foonathan_memory_vendor/build \
    && cmake .. \
        -DCMAKE_CXX_STANDARD=17 \
        -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF \
        -DFOONATHAN_MEMORY_BUILD_TESTS=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build . --target install \
    && ldconfig \
    && cd /tmp \
    && rm -rf foonathan_memory_vendor

# Clone Fast-DDS source (separate step for better caching)
WORKDIR /tmp
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.13.4 Fast-DDS-2.13.4

# Build and install Fast-DDS with minimal memory usage
WORKDIR /tmp/Fast-DDS-2.13.4/build
# Build with a single job and disable parallel building to reduce memory usage
RUN cmake .. \
    -DTHIRDPARTY=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release \
    -DCOMPILE_EXAMPLES=OFF \
    -DBUILD_SHARED_LIBS=ON \
    -DSECURITY=OFF \
    -DCMAKE_CXX_FLAGS="-w -O1" \
    && MAKEFLAGS="-j1" make \
    && make install \
    && ldconfig

# Clean up
WORKDIR /tmp
RUN rm -rf Fast-DDS-2.13.4

# Set FastDDS environment variables
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Create FastDDS configuration directory
RUN mkdir -p /root/.ros

# Install Mesa graphics drivers
# Mesa is an open-source implementation of OpenGL and other graphics APIs
# It's crucial for 3D rendering in many applications, including RViz in ROS
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:kisak/kisak-mesa

# Create necessary directories
RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p /root/ros2_ws/src/rover

# Copy the entire project into the container
COPY rover /root/ros2_ws/src/rover

# Copy configuration files from docker directory
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

# Run the workspace setup script
# This typically installs workspace dependencies and builds the ROS 2 packages
WORKDIR /root
RUN ./workspace.sh

# Ensure the ROS 2 workspace is sourced in every new shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint for the container
# This script will be run every time the container starts
ENTRYPOINT ["/root/entrypoint.sh"]

# Set the default command
# This keeps the container running indefinitely, allowing you to exec into it
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
