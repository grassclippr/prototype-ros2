#!/usr/bin/env bash
set -euo pipefail

BAUDRATE="${BAUDRATE:-921600}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-20}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-20}"
ACK_TIMEOUT="${ACK_TIMEOUT:-15}"
SERIAL_WAIT_TIMEOUT="${SERIAL_WAIT_TIMEOUT:-15}"

if ! command -v rg >/dev/null 2>&1; then
  echo "error: rg is required for this script" >&2
  exit 1
fi

echo "Waiting for serial device ${SERIAL_DEV} (timeout: ${SERIAL_WAIT_TIMEOUT}s)..."
if ! timeout "${SERIAL_WAIT_TIMEOUT}s" bash -lc "while ! python3 -c \"import os; raise SystemExit(0 if os.path.exists('${SERIAL_DEV}') else 1)\"; do sleep 1; done"; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

echo "Starting containers (core, proxy, agent)..."
BAUDRATE="${BAUDRATE}" SERIAL_DEV="${SERIAL_DEV}" podman-compose up -d --force-recreate core serial_mux_proxy micro_ros_agent

echo "Waiting for heartbeat output (timeout: ${HEARTBEAT_TIMEOUT}s)..."
if ! timeout "${HEARTBEAT_TIMEOUT}s" sh -c \
  "podman logs -f serial_mux_proxy 2>/dev/null | rg -m1 'heartbeat [0-9]+'"; then
  echo "error: no heartbeat seen in proxy logs" >&2
  exit 1
fi

echo "Waiting for /drive_command_ack topic (timeout: ${TOPIC_TIMEOUT}s)..."
if ! timeout "${TOPIC_TIMEOUT}s" sh -c \
  'while ! podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list -t" 2>/dev/null | rg -m1 "^/drive_command_ack "; do sleep 1; done'; then
  echo "error: /drive_command_ack topic not found" >&2
  exit 1
fi

echo "Waiting for /drive_command topic (timeout: ${TOPIC_TIMEOUT}s)..."
if ! timeout "${TOPIC_TIMEOUT}s" sh -c \
  'while ! podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list -t" 2>/dev/null | rg -m1 "^/drive_command "; do sleep 1; done'; then
  echo "error: /drive_command topic not found" >&2
  exit 1
fi

echo "Sending valid drive command and requiring ACCEPTED ack..."
if ! timeout "${ACK_TIMEOUT}s" sh -c \
  'podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run rover_description send_drive_command.py --linear-x 0.20 --angular-z 0.00 --timeout-ms 500 --seq 1001 --wait-seconds 5.0" | tee /tmp/drive_command_ack_valid.log | rg -m1 "ack seq=1001 status=ACCEPTED detail=accepted"'; then
  echo "error: valid drive command did not receive expected ACCEPTED ack" >&2
  exit 1
fi

echo "Sending invalid drive command and requiring REJECTED ack..."
if ! timeout "${ACK_TIMEOUT}s" sh -c \
  'podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run rover_description send_drive_command.py --linear-x 0.20 --angular-z 0.00 --timeout-ms 0 --seq 1002 --wait-seconds 5.0" | tee /tmp/drive_command_ack_invalid.log | rg -m1 "ack seq=1002 status=REJECTED detail=timeout_ms out of range"'; then
  echo "error: invalid drive command did not receive expected REJECTED ack" >&2
  exit 1
fi

echo "Drive command E2E check passed: heartbeat + ACCEPTED ack + REJECTED ack OK"
