#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

missing_local=0
for cmd in ssh rsync podman; do
  if command -v "$cmd" >/dev/null 2>&1; then
    printf 'local ok      %s\n' "$cmd"
  else
    printf 'local missing %s\n' "$cmd"
    missing_local=1
  fi
done

if ! command -v pio >/dev/null 2>&1 && ! command -v platformio >/dev/null 2>&1; then
  printf 'local missing platformio/pio\n'
  missing_local=1
else
  printf 'local ok      platformio/pio\n'
  pio_bin="$(command -v pio || command -v platformio)"
  pio_python="$(sed -n '1s/^#!//p' "$pio_bin")"
  if [ -n "$pio_python" ] && [ -x "$pio_python" ] &&
    "$pio_python" -m pip --version >/dev/null 2>&1 &&
    "$pio_python" -c 'import yaml, catkin_pkg, lark, colcon_core, importlib_resources, pytz, markupsafe, em' >/dev/null 2>&1; then
    printf 'local ok      platformio micro-ROS python deps\n'
  else
    printf 'local missing platformio micro-ROS python deps\n'
    missing_local=1
  fi
fi

ssh_pi "PI_APP_DIR='${PI_APP_DIR}' SERIAL_DEV='${SERIAL_DEV}' bash -s" <<'REMOTE'
set -euo pipefail
export PATH="${HOME}/.local/bin:${PATH}"
missing=0
printf 'remote user    %s\n' "$(whoami)"
printf 'remote host    %s\n' "$(hostname)"
printf 'remote kernel  %s\n' "$(uname -a)"

if [ "$(uname -m)" = "aarch64" ]; then
  printf 'remote ok      aarch64\n'
else
  printf 'remote missing expected aarch64, got %s\n' "$(uname -m)"
  missing=1
fi

for cmd in podman rsync systemctl python3; do
  if command -v "$cmd" >/dev/null 2>&1; then
    printf 'remote ok      %s\n' "$cmd"
  else
    printf 'remote missing %s\n' "$cmd"
    missing=1
  fi
done

if command -v esptool.py >/dev/null 2>&1 || command -v esptool >/dev/null 2>&1 || python3 -m esptool version >/dev/null 2>&1; then
  printf 'remote ok      esptool\n'
else
  printf 'remote missing esptool\n'
  missing=1
fi

mkdir -p "$PI_APP_DIR"
if [ -w "$PI_APP_DIR" ]; then
  printf 'remote ok      writable %s\n' "$PI_APP_DIR"
else
  printf 'remote missing writable %s\n' "$PI_APP_DIR"
  missing=1
fi

if [ -e "$SERIAL_DEV" ]; then
  printf 'remote ok      serial %s\n' "$SERIAL_DEV"
else
  printf 'remote missing serial %s\n' "$SERIAL_DEV"
fi

exit "$missing"
REMOTE

exit "$missing_local"
