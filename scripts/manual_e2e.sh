#!/usr/bin/env bash
set -euo pipefail

BAUDRATE="${BAUDRATE:-921600}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-15}"
ROS_TIMEOUT="${ROS_TIMEOUT:-10}"

if ! command -v rg >/dev/null 2>&1; then
  echo "error: rg is required for this script" >&2
  exit 1
fi

if [ ! -e "${SERIAL_DEV}" ]; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

echo "Starting containers (core, proxy, agent)..."
BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" docker compose up -d core serial_mux_proxy micro_ros_agent

echo "Waiting for heartbeat output (timeout: ${HEARTBEAT_TIMEOUT}s)..."
if ! timeout "${HEARTBEAT_TIMEOUT}s" sh -c \
  "docker logs -f serial_mux_proxy 2>/dev/null | rg -m1 'heartbeat [0-9]+'"; then
  echo "error: no heartbeat seen in proxy logs" >&2
  exit 1
fi

echo "Checking /baseboard ROS data (timeout: ${ROS_TIMEOUT}s)..."
if ! timeout "${ROS_TIMEOUT}s" make ros ARGS="topic echo /baseboard --once" | rg -m1 '^data:'; then
  echo "error: no /baseboard data received" >&2
  exit 1
fi

echo "E2E check passed: heartbeat + /baseboard data OK"
