#!/usr/bin/env bash
set -euo pipefail

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
PROXY_PORT="${PROXY_PORT:-8888}"
source "$(dirname "$0")/resolve_serial_dev.sh"
SERIAL_DEV="$(resolve_serial_dev)"
UPLOAD_PORT="${UPLOAD_PORT:-${SERIAL_DEV}}"
COMPOSE_CMD="${COMPOSE_CMD:-podman-compose}"
services="core serial_mux_proxy micro_ros_agent"
restore=""
host_proxy_pids=""

for svc in $services; do
  if "$COMPOSE_CMD" ps -q "$svc" >/dev/null 2>&1; then
    restore="$restore $svc"
  fi
done

"$COMPOSE_CMD" stop $services >/dev/null 2>&1 || true

if command -v lsof >/dev/null 2>&1; then
  host_proxy_pids="$(lsof -t "${SERIAL_DEV}" 2>/dev/null | tr '\n' ' ' || true)"
  if [ -n "${host_proxy_pids// }" ]; then
    echo "Stopping processes using ${SERIAL_DEV}: ${host_proxy_pids}"
    kill ${host_proxy_pids} >/dev/null 2>&1 || true
    sleep 1
  fi
fi

. ~/.platformio/penv/bin/activate

# Ensure micro-ROS headers exist
HEADER_PATH="$PIO_PROJECT/.pio/libdeps/LynxAdapter_v1_0/micro_ros_platformio/libmicroros/include/nmea_msgs/msg/sentence.h"
if [ ! -f "$HEADER_PATH" ]; then
    echo "Warning: micro-ROS headers not found. Attempting a full build..."
    pio run -d "$PIO_PROJECT" -e LynxAdapter_v1_0
fi

if [ -n "${PIO_BUILD_FLAGS:-}" ]; then
  PIO_BUILD_FLAGS="${PIO_BUILD_FLAGS}" pio run -t upload -d "$PIO_PROJECT" --upload-port "$UPLOAD_PORT"
else
  pio run -t upload -d "$PIO_PROJECT" --upload-port "$UPLOAD_PORT"
fi

if [ -n "$restore" ]; then
  BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" PROXY_PORT="${PROXY_PORT}" "$COMPOSE_CMD" up -d $restore
fi
