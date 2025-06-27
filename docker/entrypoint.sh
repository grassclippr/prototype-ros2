#!/bin/bash

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

# Ensure ROS_DOMAIN_ID file exists
if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    echo "📄 Creating $ROS_DOMAIN_ID_FILE with default value 0"
    echo "0" > "$ROS_DOMAIN_ID_FILE"
else
    echo "✅ Found existing $ROS_DOMAIN_ID_FILE"
fi

# Add ROS_DOMAIN_ID to .bashrc if missing
if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
  ros_domain_id=$(cat "$ROS_DOMAIN_ID_FILE")
  echo "🔧 Adding ROS_DOMAIN_ID=$ros_domain_id to .bashrc"
  echo "export ROS_DOMAIN_ID=$ros_domain_id" >> /root/.bashrc
else
  echo "✅ ROS_DOMAIN_ID already set in .bashrc"
fi

# Export domain ID for this session
export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
echo "🌐 ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"

# Source ROS setup
echo "📡 Sourcing ROS 2 environment: /opt/ros/$ROS_DISTRO/setup.bash"
source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace setup
echo "📦 Sourcing local workspace: $ROS_WS/install/setup.bash"
source $ROS_WS/install/setup.bash

# Source updated .bashrc
echo "🔁 Reloading /root/.bashrc"
source /root/.bashrc

# Build ROS 2 workspace
echo "🔨 Building ROS 2 workspace at $ROS_WS"
cd $ROS_WS
colcon build

# Return to home
cd
echo "🏠 Returned to home directory"

# Reload environment one last time
echo "🔁 Final .bashrc reload"
source /root/.bashrc

echo "🚀 Entrypoint setup complete. Executing: $@"

# Start the passed command
exec "$@"
