#!/usr/bin/env bash
# Guard independent C, C++, and Python calibration literals from drifting.

set -e

SOURCE_ROOT="${SOURCE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

require_match() {
  local relative_path="$1"
  local pattern="$2"
  local file_path="${SOURCE_ROOT}/${relative_path}"

  if ! grep -Eq "${pattern}" "${file_path}"; then
    echo "calibration constant guard failed: ${relative_path} must use EDGES_PER_WHEEL_REV = 448" >&2
    exit 1
  fi
}

require_match \
  "firmware/Core/Inc/encoder.h" \
  '^[[:space:]]*#define[[:space:]]+EDGES_PER_WHEEL_REV[[:space:]]+448[[:space:]]*$'
require_match \
  "ros2_ws/src/mcr_bringup/src/mcr_hardware_interface.cpp" \
  'EDGES_PER_WHEEL_REV[[:space:]]*=[[:space:]]*448(\.0)?[[:space:]]*;'
require_match \
  "tools/stm32_uart_sim.py" \
  '^[[:space:]]*EDGES_PER_WHEEL_REV[[:space:]]*=[[:space:]]*448[[:space:]]*$'

echo "calibration constant guard passed: EDGES_PER_WHEEL_REV = 448 in C, C++, and Python"
