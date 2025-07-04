#!/bin/bash

set -e  # Exit on errors

# Detect container runtime (prefer podman if available)
if command -v podman >/dev/null 2>&1; then
  CONTAINER_RUNTIME="podman"
elif command -v docker >/dev/null 2>&1; then
  CONTAINER_RUNTIME="docker"
  # Docker needs special handling for rootless mode
  if [ "$(id -u)" -ne 0 ] && [ "$CONTAINER_RUNTIME" = "docker" ]; then
    DOCKER_OPTS="--user $(id -u):$(id -g)"
  fi
else
  echo "❌ Error: Neither Podman nor Docker is installed or available in PATH"
  exit 1
fi

echo -e "🛠️  Starting container build with $CONTAINER_RUNTIME..."

SCRIPT_PATH=$(dirname $(realpath "$0"))
# PROJECT_ROOT=$(dirname "$SCRIPT_PATH")
PROJECT_ROOT=$SCRIPT_PATH

# Show full build log
$CONTAINER_RUNTIME build -f docker/Dockerfile $PROJECT_ROOT

# Get the image ID using quiet mode (-q)
image_id=$($CONTAINER_RUNTIME build -q -f docker/Dockerfile $PROJECT_ROOT)

echo -e "\n✅ Build complete. Image ID: $image_id"
echo "🚀 Starting container..."

# Run the container interactively and then remove it
DOCKER_OPTS="--rm -it \
    -v $PROJECT_ROOT/rover:/root/ros2_ws/src/rover \
    -v $PROJECT_ROOT/uros:/root/ros2_ws/src/uros \
    --privileged \
    -v /dev/serial/by-id:/dev/serial/by-id \
    -v /dev/ttyAMA0:/dev/ttyAMA0 \
    "

# Detect Wayland (Ubuntu 22.04+ etc.)
if [ "$XDG_SESSION_TYPE" = "wayland" ]; then
    echo "🌀 Detected Wayland session."

    if [ -n "$XDG_RUNTIME_DIR" ]; then
        echo "⚙️  Enabling Wayland GUI support for container."

        DOCKER_OPTS="$DOCKER_OPTS \
            --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
            --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
            --volume $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/run/$WAYLAND_DISPLAY \
            --device /dev/dri"  # Allow hardware acceleration
    else
        echo "⚠️  Warning: XDG_RUNTIME_DIR not set. Skipping Wayland GUI setup."
    fi

# Fallback to X11
elif [ -n "$DISPLAY" ]; then
    echo "🖥️  Detected X11 environment. Enabling GUI support for container."

    xhost +local:docker > /dev/null 2>&1 || true

    DOCKER_OPTS="$DOCKER_OPTS \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw"
else
    echo "📟 No GUI session detected. Running in headless mode."
fi

# Run container
$CONTAINER_RUNTIME run $DOCKER_OPTS "$image_id" /bin/bash
