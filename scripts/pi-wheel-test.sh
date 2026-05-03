#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

require_cmd ssh

LINEAR_X="${LINEAR_X:-0.20}"
ANGULAR_Z="${ANGULAR_Z:-0.0}"
RATE_HZ="${RATE_HZ:-10}"
DURATION_SEC="${DURATION_SEC:-20}"
OBSERVE_MOTOR_SEC="${OBSERVE_MOTOR_SEC:-24}"
OBSERVE_ENCODER_SEC="${OBSERVE_ENCODER_SEC:-24}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${PROJECT_ROOT}/.artifacts/wheel-tests}"
RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
ARTIFACT_DIR="${ARTIFACT_DIR:-${ARTIFACT_ROOT}/${RUN_ID}}"

mkdir -p "${ARTIFACT_DIR}"

echo "Running wheel test on ${PI_SSH}"
ssh_pi \
  "LINEAR_X='${LINEAR_X}' ANGULAR_Z='${ANGULAR_Z}' RATE_HZ='${RATE_HZ}' DURATION_SEC='${DURATION_SEC}' OBSERVE_MOTOR_SEC='${OBSERVE_MOTOR_SEC}' OBSERVE_ENCODER_SEC='${OBSERVE_ENCODER_SEC}' bash -s" <<'REMOTE' | tee "${ARTIFACT_DIR}/summary.txt"
set -euo pipefail

remote_dir="$(mktemp -d /tmp/wheel-test.XXXXXX)"
motor_log="${remote_dir}/motor.log"
feedback_log="${remote_dir}/feedback.log"
cmd_vel_log="${remote_dir}/cmd_vel.log"
topic_list_log="${remote_dir}/topics.log"

cleanup() {
  rm -rf "${remote_dir}"
}
trap cleanup EXIT

ros_env='source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash'

podman exec core bash -lc "${ros_env} && ros2 topic list -t" >"${topic_list_log}"
required_topics=(
  '/diff_drive_controller/cmd_vel '
  '/wheel_cmd '
  '/wheel_velocities '
)
for topic in "${required_topics[@]}"; do
  if ! grep -Fq "${topic}" "${topic_list_log}"; then
    echo "error: required topic missing: ${topic% }" >&2
    cat "${topic_list_log}" >&2
    exit 1
  fi
done

podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_MOTOR_SEC} ros2 topic echo /wheel_cmd" >"${motor_log}" 2>&1 &
motor_pid=$!
podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_ENCODER_SEC} ros2 topic echo /wheel_velocities" >"${feedback_log}" 2>&1 &
feedback_pid=$!
podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_ENCODER_SEC} ros2 topic echo /diff_drive_controller/cmd_vel" >"${cmd_vel_log}" 2>&1 &
cmd_vel_pid=$!

sleep 1
podman exec core bash -lc "${ros_env} && \
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
node = rclpy.create_node('pi_wheel_test_cmd_vel')
publisher = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

try:
    end_time = time.monotonic() + duration_sec
    while time.monotonic() < end_time:
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period_sec)

    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.0)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY"

wait "${motor_pid}" || true
wait "${feedback_pid}" || true
wait "${cmd_vel_pid}" || true

python3 - "${motor_log}" "${feedback_log}" "${cmd_vel_log}" <<'PY'
import re
import sys
from pathlib import Path

motor_log = Path(sys.argv[1]).read_text()
feedback_log = Path(sys.argv[2]).read_text()
cmd_vel_log = Path(sys.argv[3]).read_text()

def count_blocks(text: str) -> int:
    return sum(1 for block in text.split('---') if block.strip())

motor_samples = re.findall(r"linear:\s+x: ([0-9.\-]+)\s+y: ([0-9.\-]+)", motor_log)
feedback_samples = re.findall(r"linear:\s+x: ([0-9.\-]+)\s+y: ([0-9.\-]+)", feedback_log)

print("Motor summary:")
if motor_samples:
    non_zero = [(float(left), float(right)) for left, right in motor_samples if float(left) != 0.0 or float(right) != 0.0]
    print(f"  samples={len(motor_samples)} non_zero_samples={len(non_zero)}")
    if non_zero:
        print(f"  first_non_zero={non_zero[0][0]}/{non_zero[0][1]}")
        print(f"  last_non_zero={non_zero[-1][0]}/{non_zero[-1][1]}")
else:
    print("  no motor samples captured")

print("Feedback summary:")
if feedback_samples:
    non_zero = [(float(left), float(right)) for left, right in feedback_samples if float(left) != 0.0 or float(right) != 0.0]
    print(f"  samples={len(feedback_samples)} non_zero_samples={len(non_zero)}")
    if non_zero:
        max_left = max(sample[0] for sample in non_zero)
        max_right = max(sample[1] for sample in non_zero)
        print(f"  max_velocity={max_left}/{max_right}")
else:
    print("  no feedback samples captured")

print("Controller summary:")
print(f"  cmd_vel_samples={count_blocks(cmd_vel_log)}")
PY

echo
echo "Topics:"
cat "${topic_list_log}"
echo
echo "Motor log:"
cat "${motor_log}"
echo
echo "Feedback log:"
cat "${feedback_log}"
echo
echo "cmd_vel log:"
cat "${cmd_vel_log}"
REMOTE

echo
echo "Saved wheel-test artifacts to ${ARTIFACT_DIR}"
