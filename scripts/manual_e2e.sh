#!/usr/bin/env bash
set -euo pipefail

BAUDRATE="${BAUDRATE:-921600}"
PROXY_PORT="${PROXY_PORT:-8888}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-15}"
ROS_TIMEOUT="${ROS_TIMEOUT:-10}"
BASEBOARD_TIMEOUT="${BASEBOARD_TIMEOUT:-15}"
CORE_READY_TIMEOUT="${CORE_READY_TIMEOUT:-120}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/resolve_serial_dev.sh"
SERIAL_DEV="$(resolve_serial_dev)"
COMPOSE_CMD="${COMPOSE_CMD:-$("${SCRIPT_DIR}/resolve_compose_cmd.sh")}"
read -r -a COMPOSE_WORDS <<< "${COMPOSE_CMD}"
HOST_OS="$(uname -s)"
HOST_PROXY_LOG=""
HOST_PROXY_PID=""
BUILD_CORE_IMAGE=0
CORE_IMAGE_STAMP="${SCRIPT_DIR}/../.make/core-image-inputs.stamp"

compose() {
  "${COMPOSE_WORDS[@]}" "$@"
}

recreate_stack() {
  if command -v podman >/dev/null 2>&1; then
    podman rm -f core micro_ros_agent serial_mux_proxy >/dev/null 2>&1 || true
  fi
  compose down --remove-orphans >/dev/null 2>&1 || true
}

wait_for_pattern_in_command() {
  local timeout_s="$1"
  local pattern="$2"
  local command="$3"
  local start_ts now_ts
  start_ts="$(date +%s)"
  while true; do
    if sh -c "${command}" 2>/dev/null | rg -m1 "${pattern}" >/dev/null 2>&1; then
      return 0
    fi
    now_ts="$(date +%s)"
    if [ $((now_ts - start_ts)) -ge "${timeout_s}" ]; then
      return 1
    fi
    sleep 1
  done
}

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

if "${SCRIPT_DIR}/core_image_needs_rebuild.sh"; then
  BUILD_CORE_IMAGE=1
fi

if [ "${HOST_OS}" = "Darwin" ]; then
  if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 is required to run the host serial proxy on macOS" >&2
    exit 1
  fi
  HOST_PROXY_LOG="$(mktemp -t serial-mux-proxy)"
  recreate_stack
  echo "Starting containers (core, agent) and host serial proxy..."
  if [ "${BUILD_CORE_IMAGE}" = "1" ]; then
    BAUDRATE="${BAUDRATE}" PROXY_PORT="${PROXY_PORT}" compose up -d --build core micro_ros_agent
    mkdir -p "$(dirname "${CORE_IMAGE_STAMP}")"
    touch "${CORE_IMAGE_STAMP}"
  else
    BAUDRATE="${BAUDRATE}" PROXY_PORT="${PROXY_PORT}" compose up -d core micro_ros_agent
  fi
  echo "Starting host serial proxy..."
  PYTHONUNBUFFERED=1 PYTHONPATH=. python3 -u serial_mux_proxy.py \
    "${SERIAL_DEV}" "${PROXY_PORT}" "${BAUDRATE}" --agent-host 127.0.0.1 \
    >"${HOST_PROXY_LOG}" 2>&1 &
  HOST_PROXY_PID=$!
else
  recreate_stack
  echo "Starting containers (core, proxy, agent)..."
  if [ "${BUILD_CORE_IMAGE}" = "1" ]; then
    BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" PROXY_PORT="${PROXY_PORT}" compose up -d --build core serial_mux_proxy micro_ros_agent
    mkdir -p "$(dirname "${CORE_IMAGE_STAMP}")"
    touch "${CORE_IMAGE_STAMP}"
  else
    BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" PROXY_PORT="${PROXY_PORT}" compose up -d core serial_mux_proxy micro_ros_agent
  fi
fi

echo "Waiting for heartbeat output (timeout: ${HEARTBEAT_TIMEOUT}s)..."
if [ "${HOST_OS}" = "Darwin" ]; then
  if ! wait_for_pattern_in_command "${HEARTBEAT_TIMEOUT}" 'heartbeat [0-9]+' "cat '${HOST_PROXY_LOG}'"; then
    echo "error: no heartbeat seen in host proxy logs" >&2
    [ -f "${HOST_PROXY_LOG}" ] && cat "${HOST_PROXY_LOG}" >&2
    exit 1
  fi
else
  if ! wait_for_pattern_in_command "${HEARTBEAT_TIMEOUT}" 'heartbeat [0-9]+' "${COMPOSE_CMD} logs --tail 200 serial_mux_proxy"; then
    echo "error: no heartbeat seen in proxy logs" >&2
    exit 1
  fi
fi

echo "Waiting for core workspace build (timeout: ${CORE_READY_TIMEOUT}s)..."
if [ "${HOST_OS}" = "Darwin" ]; then
  if ! wait_for_pattern_in_command "${CORE_READY_TIMEOUT}" '^ready$' "podman exec core bash -lc 'test -f /root/ros2_ws/install/setup.bash && echo ready'"; then
    echo "error: core workspace did not become ready" >&2
    podman logs --tail 200 core >&2 || true
    exit 1
  fi
else
  if ! wait_for_pattern_in_command "${CORE_READY_TIMEOUT}" '^ready$' "${COMPOSE_CMD} exec -T core bash -lc 'test -f /root/ros2_ws/install/setup.bash && echo ready'"; then
    echo "error: core workspace did not become ready" >&2
    exit 1
  fi
fi

echo "Checking /baseboard ROS data (timeout: ${ROS_TIMEOUT}s)..."
echo "Waiting for /baseboard topic (timeout: ${BASEBOARD_TIMEOUT}s)..."
if ! wait_for_pattern_in_command "${BASEBOARD_TIMEOUT}" '^/baseboard ' "make ros ARGS='topic list -t'"; then
  echo "error: /baseboard topic not found" >&2
  exit 1
fi

echo "Checking /baseboard ROS data (timeout: ${ROS_TIMEOUT}s)..."
if ! wait_for_pattern_in_command "${ROS_TIMEOUT}" '^data:' "make ros ARGS='topic echo /baseboard --once'"; then
  echo "error: no /baseboard data received" >&2
  exit 1
fi

echo "E2E check passed: heartbeat + /baseboard data OK"
