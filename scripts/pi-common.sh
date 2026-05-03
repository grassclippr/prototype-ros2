#!/usr/bin/env bash

set -euo pipefail
export PATH="${HOME}/.local/bin:${PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -f "${PROJECT_ROOT}/.env.deploy" ]; then
  set -a
  # shellcheck disable=SC1090
  . "${PROJECT_ROOT}/.env.deploy"
  set +a
fi

PI_HOST="${PI_HOST:-192.168.68.81}"
PI_USER="${PI_USER:-letharion}"
PI_APP_DIR="${PI_APP_DIR:-/home/letharion/grassclippr}"
TARGET_PLATFORM="${TARGET_PLATFORM:-linux/arm64}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
PROXY_PORT="${PROXY_PORT:-8888}"
BAUDRATE="${BAUDRATE:-921600}"
IMU_I2C_DEV="${IMU_I2C_DEV:-/dev/i2c-1}"
ROVER_MODE="${ROVER_MODE:-hw}"
USE_JOYSTICK="${USE_JOYSTICK:-true}"
JOY_BACKEND="${JOY_BACKEND:-game_controller_node}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
JOY_DEVICE_ID="${JOY_DEVICE_ID:-0}"
JOY_DEVICE_NAME="${JOY_DEVICE_NAME:-Pro Controller}"

PI_SSH="${PI_USER}@${PI_HOST}"

CORE_IMAGE_REPO="${CORE_IMAGE_REPO:-localhost/prototype-ros2-core}"
PROXY_IMAGE_REPO="${PROXY_IMAGE_REPO:-localhost/prototype-ros2-serial-mux-proxy}"
AGENT_IMAGE="${AGENT_IMAGE:-docker.io/microros/micro-ros-agent:jazzy}"
INCLUDE_AGENT_IMAGE="${INCLUDE_AGENT_IMAGE:-1}"
RSYNC_FLAGS="${RSYNC_FLAGS:--az --partial --partial-dir=.rsync-partial --delete}"
ZSTD_LEVEL="${ZSTD_LEVEL:-19}"
ZSTD_FLAGS="${ZSTD_FLAGS:---rsyncable -T0}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "error: required command not found: $1" >&2
    exit 1
  fi
}

gitish() {
  git -C "${PROJECT_ROOT}" rev-parse --short HEAD 2>/dev/null || printf 'workspace'
}

release_id() {
  if [ -n "${RELEASE_ID:-}" ]; then
    printf '%s\n' "${RELEASE_ID}"
  else
    printf '%s-%s\n' "$(date -u +%Y%m%dT%H%M%SZ)" "$(gitish)"
  fi
}

ssh_pi() {
  ssh "${PI_SSH}" "$@"
}

rsync_pi() {
  # Keep interrupted OTA transfers resumable without penalizing version-to-version
  # deltas across rsyncable compressed image archives.
  # shellcheck disable=SC2086
  rsync ${RSYNC_FLAGS} "$@"
}
