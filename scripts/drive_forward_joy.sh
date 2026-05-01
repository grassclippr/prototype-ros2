#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-core}"
DURATION_SEC="${DURATION_SEC:-10}"
RATE_HZ="${RATE_HZ:-20}"
FORWARD_AXIS="${FORWARD_AXIS:-0.30}"
STOP_SEC="${STOP_SEC:-1}"

echo "Publishing synthetic /joy to container '$CONTAINER': forward_axis=$FORWARD_AXIS duration=${DURATION_SEC}s rate=${RATE_HZ}Hz"

podman exec "$CONTAINER" bash -lc "
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
python3 - <<'PY'
import os
import time

import rclpy
from sensor_msgs.msg import Joy

duration = float(os.environ.get('DURATION_SEC', '${DURATION_SEC}'))
rate_hz = float(os.environ.get('RATE_HZ', '${RATE_HZ}'))
forward_axis = float(os.environ.get('FORWARD_AXIS', '${FORWARD_AXIS}'))
stop_sec = float(os.environ.get('STOP_SEC', '${STOP_SEC}'))

rclpy.init()
node = rclpy.create_node('synthetic_forward_joy')
pub = node.create_publisher(Joy, '/joy', 10)

period = 1.0 / rate_hz

def publish(axis_1: float, enabled: bool) -> None:
    msg = Joy()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'joy'
    msg.axes = [0.0, axis_1, 0.0, 0.0, 0.0, 0.0]
    msg.buttons = [0] * 21
    if enabled:
        msg.buttons[7] = 1
    pub.publish(msg)

try:
    end = time.monotonic() + duration
    while time.monotonic() < end:
        publish(forward_axis, True)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)
finally:
    stop_end = time.monotonic() + stop_sec
    while time.monotonic() < stop_end:
        publish(0.0, False)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()
PY
"

echo "Done. Published neutral stop input for ${STOP_SEC}s."
