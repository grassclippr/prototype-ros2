#!/usr/bin/env bash
set -euo pipefail

PIO_PROJECT="${PIO_PROJECT:-rover/baseboard}"
services="serial_mux_proxy micro_ros_agent"
restore=""

for svc in $services; do
  if docker compose ps --status running -q "$svc" >/dev/null 2>&1; then
    restore="$restore $svc"
  fi
done

docker compose stop $services >/dev/null 2>&1 || true

. ~/.platformio/penv/bin/activate
if [ -n "${PIO_BUILD_FLAGS:-}" ]; then
  PIO_BUILD_FLAGS="${PIO_BUILD_FLAGS}" pio run -t upload -d "$PIO_PROJECT"
else
  pio run -t upload -d "$PIO_PROJECT"
fi

if [ -n "$restore" ]; then
  docker compose up -d $restore
fi
