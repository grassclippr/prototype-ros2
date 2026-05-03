#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <ros2 args...>" >&2
  echo "example: $0 topic list -t" >&2
  exit 1
fi

ros_args="$(printf ' %q' "$@")"
remote_cmd="$(
  printf '%q' \
    "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && exec ros2${ros_args}"
)"

ssh -F /dev/null "${PI_SSH}" "podman exec -i core bash -lc ${remote_cmd}"
