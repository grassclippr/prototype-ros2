#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="jazzy"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Navigate back to the workspace root
cd /root/ros2_ws

if [ "${ROS_SKIP_DEP_INSTALL:-0}" != "1" ]; then
    echo "Installing ROS 2 dependencies..."
    apt-get update
    if [ "${ROS_SKIP_ROSDEP_UPDATE:-0}" != "1" ]; then
        rosdep update
    fi
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
fi

# Build the packages
echo "Building packages..."
colcon build
source install/setup.bash

echo "Workspace setup completed!"
