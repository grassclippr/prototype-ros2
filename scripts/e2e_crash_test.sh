#!/usr/bin/env bash
set -euo pipefail

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
BAUDRATE="${BAUDRATE:-115200}"

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
    podman-compose down >/dev/null 2>&1 || true
    
    # Re-flash without the crash test flag
    echo "Flashing standard firmware..."
    PIO_BUILD_FLAGS="" pio run -t upload -d "$PIO_PROJECT" >/dev/null 2>&1 || echo "Warning: Restore flash failed."
    
    # Restart services
    podman-compose up -d core serial_mux_proxy micro_ros_agent >/dev/null 2>&1 || true
    
    echo "Restore complete."
}

# Ensure cleanup runs on script exit, interrupt, or error
trap cleanup EXIT INT TERM

echo "========================================="
echo "Stopping Podman services to free serial port"
echo "========================================="
podman-compose down || true

echo "========================================="
echo "Building and flashing CRASH TEST firmware (60s delay)"
echo "========================================="
PIO_BUILD_FLAGS="-D E2E_CRASH_TEST" pio run -t upload -d "$PIO_PROJECT"

echo "========================================="
echo "Starting Podman services (Proxy, Agent, Core)"
echo "========================================="
podman-compose up -d core serial_mux_proxy micro_ros_agent

echo "========================================="
echo "Waiting for Serial Activity (Heartbeat)..."
echo "========================================="
# Wait up to 30s for a FRESH heartbeat
if timeout 30s sh -c 'while ! podman logs --tail 10 serial_mux_proxy 2>&1 | rg -m1 "heartbeat [0-9]$"; do sleep 1; done'; then
    echo "✅ Serial link active (Fresh heartbeat detected)."
else
    echo "❌ FAILED: No fresh serial activity seen in proxy logs."
    podman logs serial_mux_proxy
    exit 1
fi

echo "========================================="
echo "Waiting for ROS topic discovery (/baseboard)..."
echo "========================================="
# Wait up to 40s for the topic to appear in the list
if timeout 40s sh -c 'while ! podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list" 2>/dev/null | rg -q "/baseboard"; do sleep 1; done'; then
    echo "✅ ROS topic discovered."
else
    echo "❌ FAILED: Topic /baseboard not found in topic list."
    exit 1
fi

echo "========================================="
echo "Verifying ROS message receipt..."
echo "========================================="
# Wait for one message with a timeout
# We increase the timeout and window here
if timeout 30s podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic echo /baseboard --once --timeout 20" 2>/dev/null | rg -m1 "^data:"; then
    echo "✅ ROS message received successfully."
else
    echo "❌ FAILED: Did not receive ROS data from /baseboard topic."
    # Check if we can see ANY data on that topic
    echo "Recent proxy logs for context:"
    podman logs --tail 20 serial_mux_proxy
    exit 1
fi

echo "========================================="
echo "Monitoring proxy logs for crash dump..."
echo "========================================="

# We expect the ESP32 to crash 60 seconds after boot.
if timeout 80s sh -c "podman logs -f serial_mux_proxy 2>&1 | awk '/Initiating deliberate crash/ {seen_init=1} /Guru Meditation Error/ || /Backtrace:/ {if (seen_init) {print \"SUCCESS: Caught crash dump!\"; exit 0}}'"; then
    echo "✅ E2E Crash Test PASSED: Crash dump was successfully received and decoded by proxy."
    exit 0
else
    echo "❌ E2E Crash Test FAILED: Did not see the crash dump in proxy logs."
    podman logs --tail 50 serial_mux_proxy
    exit 1
fi
