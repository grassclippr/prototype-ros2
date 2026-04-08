#!/usr/bin/env bash
set -euo pipefail

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
PROXY_PORT="${PROXY_PORT:-8888}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/resolve_serial_dev.sh"
SERIAL_DEV="$(resolve_serial_dev)"
UPLOAD_PORT="${UPLOAD_PORT:-${SERIAL_DEV}}"
COMPOSE_CMD="${COMPOSE_CMD:-$("${SCRIPT_DIR}/resolve_compose_cmd.sh")}"
read -r -a COMPOSE_WORDS <<< "${COMPOSE_CMD}"
PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-${PROJECT_ROOT}/.platformio}"
export PLATFORMIO_CORE_DIR
services="core serial_mux_proxy micro_ros_agent"
restore=""
host_proxy_pids=""

compose() {
  "${COMPOSE_WORDS[@]}" "$@"
}

mkdir -p "${PLATFORMIO_CORE_DIR}"

case "$(uname -s)" in
  Darwin)
    case "${UPLOAD_PORT}" in
      /dev/ttyACM0|/dev/ttyUSB0|"")
        UPLOAD_PORT="${SERIAL_DEV}"
        ;;
    esac
    ;;
esac

for svc in $services; do
  if compose ps -q "$svc" >/dev/null 2>&1; then
    restore="$restore $svc"
  fi
done

compose stop $services >/dev/null 2>&1 || true

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
  BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" PROXY_PORT="${PROXY_PORT}" compose up -d $restore
fi
