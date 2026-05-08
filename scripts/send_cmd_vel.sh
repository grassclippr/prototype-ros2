#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LINEAR_X="${LINEAR_X:-0.08}"
ANGULAR_Z="${ANGULAR_Z:-0.0}"
RATE_HZ="${RATE_HZ:-10}"
DURATION_SEC="${DURATION_SEC:-2}"
COMPOSE_CMD="${COMPOSE_CMD:-$("${SCRIPT_DIR}/resolve_compose_cmd.sh")}"
read -r -a COMPOSE_WORDS <<< "${COMPOSE_CMD}"

if command -v podman >/dev/null 2>&1 && podman ps --format '{{.Names}}' 2>/dev/null | grep -qx core; then
  EXEC_CMD=(podman exec core)
else
  EXEC_CMD=("${COMPOSE_WORDS[@]}" exec -T core)
fi

"${EXEC_CMD[@]}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && \
   LINEAR_X='${LINEAR_X}' ANGULAR_Z='${ANGULAR_Z}' RATE_HZ='${RATE_HZ}' DURATION_SEC='${DURATION_SEC}' \
   python3 - <<'PY'
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped

linear_x = float(os.environ['LINEAR_X'])
angular_z = float(os.environ['ANGULAR_Z'])
rate_hz = float(os.environ['RATE_HZ'])
duration_sec = float(os.environ['DURATION_SEC'])
period_sec = 1.0 / rate_hz

rclpy.init()
node = rclpy.create_node('send_cmd_vel')
publisher = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

try:
    end_time = time.monotonic() + duration_sec
    while time.monotonic() < end_time:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period_sec)

    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.0)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY"
