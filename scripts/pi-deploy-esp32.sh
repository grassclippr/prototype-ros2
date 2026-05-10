#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

require_cmd rsync
require_cmd ssh

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
PIO_ENV="${PIO_ENV:-LynxAdapter_v1_0}"
UPLOAD_SPEED="${UPLOAD_SPEED:-115200}"
RID="$(release_id)"
ARTIFACT_DIR="${PROJECT_ROOT}/.deploy/${RID}/esp32"
BUILD_DIR="${PROJECT_ROOT}/${PIO_PROJECT}/.pio/build/${PIO_ENV}"

if command -v pio >/dev/null 2>&1; then
  PIO_CMD=(pio)
elif command -v platformio >/dev/null 2>&1; then
  PIO_CMD=(platformio)
elif [ -f "${HOME}/.platformio/penv/bin/activate" ]; then
  # shellcheck disable=SC1091
  . "${HOME}/.platformio/penv/bin/activate"
  PIO_CMD=(pio)
else
  echo "error: PlatformIO is required locally to build ESP32 firmware" >&2
  exit 1
fi

mkdir -p "${ARTIFACT_DIR}"

echo "Building ESP32 firmware: ${PIO_PROJECT} ${PIO_ENV}"
if [ -n "${PIO_BUILD_FLAGS:-}" ]; then
  PIO_BUILD_FLAGS="${PIO_BUILD_FLAGS}" "${PIO_CMD[@]}" run -d "${PROJECT_ROOT}/${PIO_PROJECT}" -e "${PIO_ENV}"
else
  "${PIO_CMD[@]}" run -d "${PROJECT_ROOT}/${PIO_PROJECT}" -e "${PIO_ENV}"
fi

for artifact in firmware.bin bootloader.bin partitions.bin; do
  if [ ! -f "${BUILD_DIR}/${artifact}" ]; then
    echo "error: expected PlatformIO artifact missing: ${BUILD_DIR}/${artifact}" >&2
    exit 1
  fi
  cp "${BUILD_DIR}/${artifact}" "${ARTIFACT_DIR}/${artifact}"
done

BOOT_APP0_SRC="${BOOT_APP0_SRC:-}"
if [ -z "${BOOT_APP0_SRC}" ]; then
  for candidate in \
    "${BUILD_DIR}/boot_app0.bin" \
    "${HOME}/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"; do
    if [ -f "${candidate}" ]; then
      BOOT_APP0_SRC="${candidate}"
      break
    fi
  done
fi

BOOT_APP0_PRESENT=0
if [ -n "${BOOT_APP0_SRC}" ] && [ -f "${BOOT_APP0_SRC}" ]; then
  cp "${BOOT_APP0_SRC}" "${ARTIFACT_DIR}/boot_app0.bin"
  BOOT_APP0_PRESENT=1
fi

cat > "${ARTIFACT_DIR}/manifest.env" <<EOF
RELEASE_ID='${RID}'
PIO_PROJECT='${PIO_PROJECT}'
PIO_ENV='${PIO_ENV}'
PIO_BUILD_FLAGS='${PIO_BUILD_FLAGS:-}'
SERIAL_DEV='${SERIAL_DEV}'
UPLOAD_SPEED='${UPLOAD_SPEED}'
BOOT_APP0_PRESENT='${BOOT_APP0_PRESENT}'
GIT_REV='$(gitish)'
CREATED_UTC='$(date -u +%Y-%m-%dT%H:%M:%SZ)'
EOF

cat > "${ARTIFACT_DIR}/remote-flash.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
export PATH="${HOME}/.local/bin:${PATH}"

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${BUNDLE_DIR}/manifest.env"

sudo systemctl stop grassclippr-serial-mux-proxy grassclippr-core grassclippr-micro-ros-agent >/dev/null 2>&1 || true
podman rm -f core serial_mux_proxy micro_ros_agent >/dev/null 2>&1 || true

if command -v lsof >/dev/null 2>&1; then
  pids="$(lsof -t "${SERIAL_DEV}" 2>/dev/null | tr '\n' ' ' || true)"
  if [ -n "${pids// }" ]; then
    kill ${pids} >/dev/null 2>&1 || true
    sleep 1
  fi
elif command -v fuser >/dev/null 2>&1; then
  fuser -k "${SERIAL_DEV}" >/dev/null 2>&1 || true
  sleep 1
fi

if [ ! -e "${SERIAL_DEV}" ]; then
  echo "error: serial device not found: ${SERIAL_DEV}" >&2
  exit 1
fi

if command -v esptool.py >/dev/null 2>&1; then
  ESPTOOL=(esptool.py)
elif command -v esptool >/dev/null 2>&1; then
  ESPTOOL=(esptool)
elif python3 -m esptool version >/dev/null 2>&1; then
  ESPTOOL=(python3 -m esptool)
else
  echo "error: esptool is required on the Pi to flash the ESP32" >&2
  exit 1
fi

flash_args=(
  --chip esp32s3
  --port "${SERIAL_DEV}"
  --baud "${UPLOAD_SPEED}"
  --before default_reset
  --after hard_reset
  write_flash
  -z
  0x0000 "${BUNDLE_DIR}/bootloader.bin"
  0x8000 "${BUNDLE_DIR}/partitions.bin"
)

if [ "${BOOT_APP0_PRESENT}" = "1" ]; then
  flash_args+=(0xe000 "${BUNDLE_DIR}/boot_app0.bin")
fi

flash_args+=(0x10000 "${BUNDLE_DIR}/firmware.bin")

"${ESPTOOL[@]}" "${flash_args[@]}"

APP_DIR="$(cd "${BUNDLE_DIR}/../.." && pwd)"
if [ -x "${APP_DIR}/current/remote-start.sh" ]; then
  "${APP_DIR}/current/remote-start.sh"
fi
EOF
chmod +x "${ARTIFACT_DIR}/remote-flash.sh"

REMOTE_BUNDLE_DIR="${PI_APP_DIR}/firmware/${RID}"
ssh_pi "mkdir -p '${REMOTE_BUNDLE_DIR}'"
rsync_pi "${ARTIFACT_DIR}/" "${PI_SSH}:${REMOTE_BUNDLE_DIR}/"
ssh_pi "'${REMOTE_BUNDLE_DIR}/remote-flash.sh'"

echo "Flashed ESP32 firmware bundle ${RID} via ${PI_SSH}:${SERIAL_DEV}"
