#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

ssh_pi "PI_APP_DIR='${PI_APP_DIR}' bash -s" <<'REMOTE'
set -euo pipefail

if [ ! -L "${PI_APP_DIR}/previous" ]; then
  echo "error: no previous release symlink at ${PI_APP_DIR}/previous" >&2
  exit 1
fi

old_current=""
if [ -L "${PI_APP_DIR}/current" ]; then
  old_current="$(readlink -f "${PI_APP_DIR}/current")"
fi

previous="$(readlink -f "${PI_APP_DIR}/previous")"
ln -sfn "${previous}" "${PI_APP_DIR}/current"
if [ -n "${old_current}" ]; then
  ln -sfn "${old_current}" "${PI_APP_DIR}/previous"
fi

"${PI_APP_DIR}/current/remote-start.sh"
REMOTE
