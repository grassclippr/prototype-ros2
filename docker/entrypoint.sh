#!/bin/bash -e

# Set ROS distribution
ROS_DISTRO="jazzy"

echo "🐢 Starting ROS 2 container with distro: $ROS_DISTRO"

# Set up XDG runtime directory
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
echo "📁 Setting up XDG_RUNTIME_DIR at $XDG_RUNTIME_DIR"
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Set base paths
ROS_WS="/root/ros2_ws"
SHARED_ROS2="/root/shared/ros2"
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

mkdir -p $SHARED_ROS2

# Ensure ROS_DOMAIN_ID file exists
if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    echo "📄 Creating $ROS_DOMAIN_ID_FILE with default value 0"
    echo "0" > "$ROS_DOMAIN_ID_FILE"
else
    echo "✅ Found existing $ROS_DOMAIN_ID_FILE"
fi

# Export domain ID for this session
export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
echo "🌐 ROS_DOMAIN_ID set to \"$ROS_DOMAIN_ID\""

# Source ROS setup
echo "📡 Sourcing ROS 2 environment: /opt/ros/$ROS_DISTRO/setup.bash"
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the mounted workspace before launching so bind-mounted source changes are
# reflected in the install space after container recreation or host reboot.
if [ "${ROS_AUTOBUILD:-1}" != "0" ]; then
  echo "🔨 Building ROS 2 workspace at $ROS_WS"
  cd "$ROS_WS"
  colcon build
elif [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "🔨 Building ROS 2 workspace at $ROS_WS"
  /root/workspace.sh
fi

# Source workspace setup now that it exists.
echo "📦 Sourcing local workspace: $ROS_WS/install/setup.bash"
source "$ROS_WS/install/setup.bash"

echo "🚀 Entrypoint setup complete. Executing: $@"

# Start the passed command
exec "$@"
