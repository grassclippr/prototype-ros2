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
   timeout ${DURATION_SEC}s ros2 topic pub -r ${RATE_HZ} /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
   \"{header: {frame_id: base_link}, twist: {linear: {x: ${LINEAR_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}}\""
