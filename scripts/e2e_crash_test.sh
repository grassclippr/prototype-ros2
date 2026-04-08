#!/usr/bin/env bash
set -euo pipefail

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
BAUDRATE="${BAUDRATE:-115200}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/resolve_serial_dev.sh"
SERIAL_DEV="$(resolve_serial_dev)"
COMPOSE_CMD="${COMPOSE_CMD:-$("${SCRIPT_DIR}/resolve_compose_cmd.sh")}"
read -r -a COMPOSE_WORDS <<< "${COMPOSE_CMD}"

# Ensure rg is installed
if ! command -v rg >/dev/null 2>&1; then
  echo "error: rg is required for this script" >&2
  exit 1
fi

if [ ! -e "${SERIAL_DEV}" ]; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

cleanup() {
    echo "========================================="
    echo "Cleaning up and restoring normal firmware..."
    echo "========================================="

    # Stop containers to release serial port
    compose down >/dev/null 2>&1 || true

    # Re-flash without the crash test flag
    echo "Flashing standard firmware..."
    PIO_BUILD_FLAGS="" pio run -t upload -d "$PIO_PROJECT" >/dev/null 2>&1 || echo "Warning: Restore flash failed."

    # Restart services
    compose up -d core serial_mux_proxy micro_ros_agent >/dev/null 2>&1 || true

    echo "Restore complete."
}

compose() {
  "${COMPOSE_WORDS[@]}" "$@"
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

# Ensure cleanup runs on script exit, interrupt, or error
trap cleanup EXIT INT TERM

echo "========================================="
echo "Stopping Podman services to free serial port"
echo "========================================="
compose down || true

echo "========================================="
echo "Building and flashing CRASH TEST firmware (60s delay)"
echo "========================================="
PIO_BUILD_FLAGS="-D E2E_CRASH_TEST" pio run -t upload -d "$PIO_PROJECT"

echo "========================================="
echo "Starting Podman services (Proxy, Agent, Core)"
echo "========================================="
compose up -d core serial_mux_proxy micro_ros_agent

echo "========================================="
echo "Waiting for Serial Activity (Heartbeat)..."
echo "========================================="
if wait_for_pattern_in_command 30 'heartbeat [0-9]$' "${COMPOSE_CMD} logs --tail 10 serial_mux_proxy"; then
    echo "✅ Serial link active (Fresh heartbeat detected)."
else
    echo "❌ FAILED: No fresh serial activity seen in proxy logs."
    compose logs serial_mux_proxy
    exit 1
fi

echo "========================================="
echo "Waiting for ROS topic discovery (/baseboard)..."
echo "========================================="
if wait_for_pattern_in_command 40 '^/baseboard$' "${COMPOSE_CMD} exec -T core bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list'"; then
    echo "✅ ROS topic discovered."
else
    echo "❌ FAILED: Topic /baseboard not found in topic list."
    exit 1
fi

echo "========================================="
echo "Verifying ROS message receipt..."
echo "========================================="
if wait_for_pattern_in_command 30 '^data:' "${COMPOSE_CMD} exec -T core bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /baseboard --once --timeout 20'"; then
    echo "✅ ROS message received successfully."
else
    echo "❌ FAILED: Did not receive ROS data from /baseboard topic."
    # Check if we can see ANY data on that topic
    echo "Recent proxy logs for context:"
    compose logs --tail 20 serial_mux_proxy
    exit 1
fi

echo "========================================="
echo "Monitoring proxy logs for crash dump..."
echo "========================================="

# We expect the ESP32 to crash 60 seconds after boot.
if wait_for_pattern_in_command 80 'Guru Meditation Error|Backtrace:' "${COMPOSE_CMD} logs --tail 200 serial_mux_proxy"; then
    echo "✅ E2E Crash Test PASSED: Crash dump was successfully received and decoded by proxy."
    exit 0
else
    echo "❌ E2E Crash Test FAILED: Did not see the crash dump in proxy logs."
    compose logs --tail 50 serial_mux_proxy
    exit 1
fi
