#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${SCRIPT_DIR}/pi-common.sh"

SERVICE="${SERVICE:-${1:-core}}"
TAIL="${TAIL:-200}"

ssh_pi "podman logs --tail '${TAIL}' '${SERVICE}'"
