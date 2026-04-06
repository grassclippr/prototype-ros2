#!/usr/bin/env bash
set -euo pipefail

BAUDRATE="${BAUDRATE:-921600}"
PROXY_PORT="${PROXY_PORT:-8888}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-20}"
source "$(dirname "$0")/resolve_serial_dev.sh"
SERIAL_DEV="$(resolve_serial_dev)"
COMPOSE_CMD="${COMPOSE_CMD:-podman-compose}"
HOST_OS="$(uname -s)"
HOST_PROXY_LOG=""
HOST_PROXY_PID=""

cleanup() {
  if [ -n "${HOST_PROXY_PID}" ]; then
    kill "${HOST_PROXY_PID}" >/dev/null 2>&1 || true
    wait "${HOST_PROXY_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${HOST_PROXY_LOG}" ]; then
    rm -f "${HOST_PROXY_LOG}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if ! command -v rg >/dev/null 2>&1; then
  echo "error: rg is required for this script" >&2
  exit 1
fi

if [ ! -e "${SERIAL_DEV}" ]; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

if [ "${HOST_OS}" = "Darwin" ]; then
  if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 is required to run the host serial proxy on macOS" >&2
    exit 1
  fi
  HOST_PROXY_LOG="$(mktemp /tmp/serial-mux-proxy.XXXXXX.log)"
  echo "Starting containers (core, agent) and host serial proxy..."
  BAUDRATE="${BAUDRATE}" PROXY_PORT="${PROXY_PORT}" "$COMPOSE_CMD" up -d core micro_ros_agent
  echo "Starting host serial proxy..."
  PYTHONUNBUFFERED=1 PYTHONPATH=. python3 -u serial_mux_proxy.py \
    "${SERIAL_DEV}" "${PROXY_PORT}" "${BAUDRATE}" --agent-host 127.0.0.1 \
    >"${HOST_PROXY_LOG}" 2>&1 &
  HOST_PROXY_PID=$!
else
  echo "Starting containers (core, proxy, agent)..."
  BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" PROXY_PORT="${PROXY_PORT}" "$COMPOSE_CMD" up -d core serial_mux_proxy micro_ros_agent
fi

echo "Waiting for heartbeat output (timeout: ${HEARTBEAT_TIMEOUT}s)..."
if [ "${HOST_OS}" = "Darwin" ]; then
  if ! timeout "${HEARTBEAT_TIMEOUT}s" sh -c \
    "while [ ! -s '${HOST_PROXY_LOG}' ]; do sleep 1; done; rg -m1 'heartbeat [0-9]+' '${HOST_PROXY_LOG}'"; then
    echo "error: no heartbeat seen in host proxy logs" >&2
    [ -f "${HOST_PROXY_LOG}" ] && cat "${HOST_PROXY_LOG}" >&2
    exit 1
  fi
else
  if ! timeout "${HEARTBEAT_TIMEOUT}s" sh -c \
    "podman logs -f serial_mux_proxy 2>/dev/null | rg -m1 'heartbeat [0-9]+'"; then
    echo "error: no heartbeat seen in proxy logs" >&2
    exit 1
  fi
fi

echo "Starting keyboard teleop. Use w/s/a/d to move, space or x to stop, q to quit."
podman exec -it core bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run rover_description keyboard_twist_teleop.py"
