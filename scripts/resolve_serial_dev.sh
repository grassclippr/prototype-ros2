#!/usr/bin/env bash

resolve_serial_dev() {
  local serial_dev="${SERIAL_DEV:-/dev/ttyACM0}"

  if [ -e "$serial_dev" ]; then
    printf '%s\n' "$serial_dev"
    return 0
  fi

  case "$(uname -s)" in
    Darwin)
      for pattern in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/tty.usbmodem* /dev/tty.usbserial*; do
        for candidate in $pattern; do
          if [ -e "$candidate" ]; then
            printf '%s\n' "$candidate"
            return 0
          fi
        done
      done
      ;;
  esac

  printf '%s\n' "$serial_dev"
}

resolve_serial_dev
