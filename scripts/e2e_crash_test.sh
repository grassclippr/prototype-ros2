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
    
    # Kill the proxy process if it's still running
    if [ -n "${PROXY_PID:-}" ]; then
        kill "$PROXY_PID" 2>/dev/null || true
    fi
    
    # Re-flash without the crash test flag
    PIO_BUILD_FLAGS="" pio run -t upload -d "$PIO_PROJECT" >/dev/null 2>&1
    
    echo "Restore complete."
}

# Ensure cleanup runs on script exit, interrupt, or error
trap cleanup EXIT INT TERM

echo "========================================="
echo "Building and flashing CRASH TEST firmware"
echo "========================================="
PIO_BUILD_FLAGS="-D E2E_CRASH_TEST" pio run -t upload -d "$PIO_PROJECT"

echo "========================================="
echo "Starting native serial_mux_proxy"
echo "========================================="
# Create a temporary log file
PROXY_LOG=$(mktemp)

# Start proxy natively in the background and log to the file
# We don't need micro_ros_agent for the crash test, just the proxy parsing the serial
PYTHONPATH=. uv run serial_mux_proxy.py "${SERIAL_DEV}" "${BAUDRATE}" > "$PROXY_LOG" 2>&1 &
PROXY_PID=$!

# Give proxy a moment to start
sleep 1

echo "========================================="
echo "Monitoring proxy logs for crash dump..."
echo "========================================="

# We expect the ESP32 to crash 5 seconds after boot.
# We will watch the logs for 15 seconds.
# We use tail -f to watch the log file and awk to grep for the success condition.
timeout 15s sh -c "tail -f \"$PROXY_LOG\" | awk '/Initiating deliberate crash/ {seen_init=1} /Guru Meditation Error/ || /Backtrace:/ {if (seen_init) {print \"SUCCESS: Caught crash dump!\"; exit 0}}'"

if [ $? -eq 0 ]; then
    echo "✅ E2E Crash Test PASSED: Crash dump was successfully received and decoded by proxy."
    rm "$PROXY_LOG"
    exit 0
else
    echo "❌ E2E Crash Test FAILED: Did not see the crash dump in proxy logs within 15 seconds."
    # Dump recent logs for debugging
    tail -n 50 "$PROXY_LOG"
    rm "$PROXY_LOG"
    exit 1
fi
