#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP_FILE="${PROJECT_ROOT}/.make/core-image-inputs.stamp"
IMAGE_NAME="${CORE_IMAGE_NAME:-localhost/prototype-ros2_new_branch_2_core:latest}"

if [ "${BUILD:-0}" = "1" ]; then
  exit 0
fi

if ! command -v podman >/dev/null 2>&1; then
  exit 0
fi

if ! podman image exists "${IMAGE_NAME}" >/dev/null 2>&1; then
  exit 0
fi

if [ ! -f "${STAMP_FILE}" ]; then
  exit 0
fi

inputs=(
  "${PROJECT_ROOT}/docker-compose.yml"
  "${PROJECT_ROOT}/docker/Dockerfile"
  "${PROJECT_ROOT}/docker/workspace.sh"
  "${PROJECT_ROOT}/docker/entrypoint.sh"
)

while IFS= read -r manifest; do
  inputs+=("${manifest}")
done < <(find "${PROJECT_ROOT}/rover" -type f -name package.xml -not -path '*/.pio/*' | sort)

for input in "${inputs[@]}"; do
  if [ "${input}" -nt "${STAMP_FILE}" ]; then
    exit 0
  fi
done

exit 1
