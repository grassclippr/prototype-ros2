#!/usr/bin/env bash

set -euo pipefail

if command -v podman-compose >/dev/null 2>&1; then
  printf '%s\n' "podman-compose"
  exit 0
fi

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  printf '%s\n' "docker compose"
  exit 0
fi

echo "error: neither podman-compose nor docker compose is available" >&2
exit 1
