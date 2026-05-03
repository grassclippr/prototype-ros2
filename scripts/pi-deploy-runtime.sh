#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

require_cmd podman
require_cmd rsync
require_cmd ssh

RID="$(release_id)"
ARTIFACT_DIR="${PROJECT_ROOT}/.deploy/${RID}/runtime"
CORE_IMAGE="${CORE_IMAGE_REPO}:${RID}"
PROXY_IMAGE="${PROXY_IMAGE_REPO}:${RID}"

mkdir -p "${ARTIFACT_DIR}"

echo "Building ${CORE_IMAGE} for ${TARGET_PLATFORM}"
podman build --platform "${TARGET_PLATFORM}" -t "${CORE_IMAGE}" -f "${PROJECT_ROOT}/docker/Dockerfile" "${PROJECT_ROOT}"

echo "Building ${PROXY_IMAGE} for ${TARGET_PLATFORM}"
podman build --platform "${TARGET_PLATFORM}" -t "${PROXY_IMAGE}" -f "${PROJECT_ROOT}/docker/Dockerfile.proxy" "${PROJECT_ROOT}"

echo "Saving runtime images"
podman save -o "${ARTIFACT_DIR}/core-image.tar" "${CORE_IMAGE}"
podman save -o "${ARTIFACT_DIR}/serial-mux-proxy-image.tar" "${PROXY_IMAGE}"

if [ "${INCLUDE_AGENT_IMAGE}" = "1" ]; then
  echo "Pulling and saving ${AGENT_IMAGE} for ${TARGET_PLATFORM}"
  podman pull --platform "${TARGET_PLATFORM}" "${AGENT_IMAGE}"
  podman save -o "${ARTIFACT_DIR}/micro-ros-agent-image.tar" "${AGENT_IMAGE}"
fi

cat > "${ARTIFACT_DIR}/manifest.env" <<EOF
RELEASE_ID='${RID}'
CORE_IMAGE='${CORE_IMAGE}'
PROXY_IMAGE='${PROXY_IMAGE}'
AGENT_IMAGE='${AGENT_IMAGE}'
SERIAL_DEV='${SERIAL_DEV}'
PROXY_PORT='${PROXY_PORT}'
BAUDRATE='${BAUDRATE}'
ROVER_MODE='${ROVER_MODE}'
USE_JOYSTICK='${USE_JOYSTICK}'
JOY_BACKEND='${JOY_BACKEND}'
JOY_DEV='${JOY_DEV}'
JOY_DEVICE_ID='${JOY_DEVICE_ID}'
JOY_DEVICE_NAME='${JOY_DEVICE_NAME}'
GIT_REV='$(gitish)'
CREATED_UTC='$(date -u +%Y-%m-%dT%H:%M:%SZ)'
EOF

cat > "${ARTIFACT_DIR}/remote-start.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

RELEASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${RELEASE_DIR}/manifest.env"

podman network exists micro_ros_net >/dev/null 2>&1 || podman network create micro_ros_net >/dev/null
podman rm -f core serial_mux_proxy micro_ros_agent >/dev/null 2>&1 || true

podman run -d --name micro_ros_agent \
  --network micro_ros_net \
  -p "${PROXY_PORT}:${PROXY_PORT}" \
  "${AGENT_IMAGE}" tcp4 --port "${PROXY_PORT}" -v4 >/dev/null

if [ ! -e "${SERIAL_DEV}" ]; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

podman run -d --name serial_mux_proxy \
  --network micro_ros_net \
  --privileged \
  --group-add keep-groups \
  --device "${SERIAL_DEV}:${SERIAL_DEV}" \
  "${PROXY_IMAGE}" "${SERIAL_DEV}" "${PROXY_PORT}" "${BAUDRATE}" --agent-host micro_ros_agent >/dev/null

core_args=(
  -d --name core
  --network micro_ros_net
  --privileged
  --group-add keep-groups
  -e ROS_AUTOBUILD=0
  -v /run/udev:/run/udev:ro
)

if [ -e /dev/input ]; then
  core_args+=(-v /dev/input:/dev/input)
fi

podman run "${core_args[@]}" "${CORE_IMAGE}" \
  ros2 launch rover_description robot_control.launch.py \
  "mode:=${ROVER_MODE}" \
  "use_joystick:=${USE_JOYSTICK}" \
  "joy_backend:=${JOY_BACKEND}" \
  "joy_dev:=${JOY_DEV}" \
  "joy_device_id:=${JOY_DEVICE_ID}" \
  "joy_device_name:=${JOY_DEVICE_NAME}" >/dev/null

podman ps --format '{{.Names}} {{.Image}} {{.Status}}'
EOF
chmod +x "${ARTIFACT_DIR}/remote-start.sh"

cat > "${ARTIFACT_DIR}/remote-activate.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

RELEASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${RELEASE_DIR}/../.." && pwd)"
IMAGE_CACHE_DIR="${APP_DIR}/image-cache"

podman load -i "${IMAGE_CACHE_DIR}/core-image.tar"
podman load -i "${IMAGE_CACHE_DIR}/serial-mux-proxy-image.tar"
if [ -f "${IMAGE_CACHE_DIR}/micro-ros-agent-image.tar" ]; then
  podman load -i "${IMAGE_CACHE_DIR}/micro-ros-agent-image.tar"
fi

if [ -L "${APP_DIR}/current" ]; then
  current_target="$(readlink -f "${APP_DIR}/current")"
  if [ "${current_target}" != "${RELEASE_DIR}" ]; then
    ln -sfn "${current_target}" "${APP_DIR}/previous"
  fi
fi

ln -sfn "${RELEASE_DIR}" "${APP_DIR}/current"
"${RELEASE_DIR}/remote-start.sh"
EOF
chmod +x "${ARTIFACT_DIR}/remote-activate.sh"

REMOTE_RELEASE_DIR="${PI_APP_DIR}/releases/${RID}"
REMOTE_IMAGE_CACHE_DIR="${PI_APP_DIR}/image-cache"
ssh_pi "mkdir -p '${REMOTE_RELEASE_DIR}' '${REMOTE_IMAGE_CACHE_DIR}'"
rsync_pi "${ARTIFACT_DIR}/manifest.env" "${ARTIFACT_DIR}/remote-start.sh" "${ARTIFACT_DIR}/remote-activate.sh" "${PI_SSH}:${REMOTE_RELEASE_DIR}/"
rsync_pi "${ARTIFACT_DIR}/core-image.tar" "${ARTIFACT_DIR}/serial-mux-proxy-image.tar" "${PI_SSH}:${REMOTE_IMAGE_CACHE_DIR}/"
if [ -f "${ARTIFACT_DIR}/micro-ros-agent-image.tar" ]; then
  rsync_pi "${ARTIFACT_DIR}/micro-ros-agent-image.tar" "${PI_SSH}:${REMOTE_IMAGE_CACHE_DIR}/"
fi
ssh_pi "'${REMOTE_RELEASE_DIR}/remote-activate.sh'"

echo "Deployed runtime release ${RID} to ${PI_SSH}:${REMOTE_RELEASE_DIR}"
