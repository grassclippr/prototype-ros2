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
encoder_log="${remote_dir}/encoder.log"
cmd_vel_log="${remote_dir}/cmd_vel.log"
cmd_vel_out_log="${remote_dir}/cmd_vel_out.log"
topic_list_log="${remote_dir}/topics.log"

cleanup() {
  rm -rf "${remote_dir}"
}
trap cleanup EXIT

ros_env='source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash'

podman exec core bash -lc "${ros_env} && ros2 topic list -t" >"${topic_list_log}"
required_topics=(
  '/diff_drive_controller/cmd_vel '
  '/diff_drive_controller/cmd_vel_out '
  '/baseboard/motor_command '
  '/baseboard/encoder_state '
)
for topic in "${required_topics[@]}"; do
  if ! grep -Fq "${topic}" "${topic_list_log}"; then
    echo "error: required topic missing: ${topic% }" >&2
    cat "${topic_list_log}" >&2
    exit 1
  fi
done

podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_MOTOR_SEC} ros2 topic echo /baseboard/motor_command --qos-reliability best_effort --qos-depth 1" >"${motor_log}" 2>&1 &
motor_pid=$!
podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_ENCODER_SEC} ros2 topic echo /baseboard/encoder_state --qos-profile sensor_data" >"${encoder_log}" 2>&1 &
encoder_pid=$!
podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_ENCODER_SEC} ros2 topic echo /diff_drive_controller/cmd_vel" >"${cmd_vel_log}" 2>&1 &
cmd_vel_pid=$!
podman exec core bash -lc "${ros_env} && timeout ${OBSERVE_ENCODER_SEC} ros2 topic echo /diff_drive_controller/cmd_vel_out" >"${cmd_vel_out_log}" 2>&1 &
cmd_vel_out_pid=$!

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

wait "${motor_pid}" || true
wait "${encoder_pid}" || true
wait "${cmd_vel_pid}" || true
wait "${cmd_vel_out_pid}" || true

python3 - "${motor_log}" "${encoder_log}" "${cmd_vel_log}" "${cmd_vel_out_log}" <<'PY'
import re
import sys
from pathlib import Path

motor_log = Path(sys.argv[1]).read_text()
encoder_log = Path(sys.argv[2]).read_text()
cmd_vel_log = Path(sys.argv[3]).read_text()
cmd_vel_out_log = Path(sys.argv[4]).read_text()

def count_blocks(text: str) -> int:
    return sum(1 for block in text.split('---') if block.strip())

motor_samples = re.findall(r"left_speed_percent: ([0-9.\-]+)\s+right_speed_percent: ([0-9.\-]+)", motor_log)
tick_samples = re.findall(r"left_ticks: (\d+)\s+right_ticks: (\d+)", encoder_log)
feedback_samples = re.findall(
    r"left_velocity_ticks_per_sec: ([0-9.\-]+)\s+right_velocity_ticks_per_sec: ([0-9.\-]+)",
    encoder_log,
)

print("Motor summary:")
if motor_samples:
    non_zero = [sample for sample in motor_samples if sample != ("0.0", "0.0")]
    print(f"  samples={len(motor_samples)} non_zero_samples={len(non_zero)}")
    if non_zero:
        print(f"  first_non_zero={non_zero[0][0]}/{non_zero[0][1]}")
        print(f"  last_non_zero={non_zero[-1][0]}/{non_zero[-1][1]}")
else:
    print("  no motor samples captured")

print("Encoder summary:")
if tick_samples:
    start_left, start_right = map(int, tick_samples[0])
    end_left, end_right = map(int, tick_samples[-1])
    print(f"  start_ticks={start_left}/{start_right}")
    print(f"  end_ticks={end_left}/{end_right}")
    print(f"  delta_ticks={end_left - start_left}/{end_right - start_right}")
else:
    print("  no encoder tick samples captured")

if feedback_samples:
    non_zero = [(float(left), float(right)) for left, right in feedback_samples if float(left) != 0.0 or float(right) != 0.0]
    print(f"  non_zero_velocity_samples={len(non_zero)}")
    if non_zero:
        max_left = max(sample[0] for sample in non_zero)
        max_right = max(sample[1] for sample in non_zero)
        print(f"  max_velocity={max_left}/{max_right}")
else:
    print("  no encoder velocity samples captured")

print("Controller summary:")
print(f"  cmd_vel_samples={count_blocks(cmd_vel_log)}")
print(f"  cmd_vel_out_samples={count_blocks(cmd_vel_out_log)}")
PY

echo
echo "Topics:"
cat "${topic_list_log}"
echo
echo "Motor log:"
cat "${motor_log}"
echo
echo "Encoder log:"
cat "${encoder_log}"
echo
echo "cmd_vel log:"
cat "${cmd_vel_log}"
echo
echo "cmd_vel_out log:"
cat "${cmd_vel_out_log}"
REMOTE

echo
echo "Saved wheel-test artifacts to ${ARTIFACT_DIR}"
