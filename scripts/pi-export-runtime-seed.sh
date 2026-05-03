#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

SOURCE_DIR="${1:-}"
if [ -z "${SOURCE_DIR}" ]; then
  SOURCE_DIR="$(find "${PROJECT_ROOT}/.deploy" -mindepth 2 -maxdepth 2 -type d -name runtime 2>/dev/null | sort | tail -1)"
fi

if [ -z "${SOURCE_DIR}" ] || [ ! -f "${SOURCE_DIR}/manifest.env" ]; then
  echo "error: no runtime deploy artifact found. Run make pi-deploy-runtime first." >&2
  exit 1
fi

DEST_DIR="${PROJECT_ROOT}/../pi-gen/seed/runtime"
rm -rf "${DEST_DIR}"
mkdir -p "${DEST_DIR}/release" "${DEST_DIR}/image-cache"

cp "${SOURCE_DIR}/manifest.env" "${SOURCE_DIR}/remote-start.sh" "${DEST_DIR}/release/"
cp "${SOURCE_DIR}/core-image.tar" "${SOURCE_DIR}/serial-mux-proxy-image.tar" "${DEST_DIR}/image-cache/"
if [ -f "${SOURCE_DIR}/micro-ros-agent-image.tar" ]; then
  cp "${SOURCE_DIR}/micro-ros-agent-image.tar" "${DEST_DIR}/image-cache/"
fi

cat > "${DEST_DIR}/release/remote-activate.sh" <<'EOF'
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
chmod +x "${DEST_DIR}/release/remote-activate.sh"

echo "Exported runtime seed from ${SOURCE_DIR} to ${DEST_DIR}"
