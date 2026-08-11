#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! git config --global --get-all safe.directory 2>/dev/null | \
    grep --fixed-strings --line-regexp --quiet "${PROJECT_ROOT}"; then
  git config --global --add safe.directory "${PROJECT_ROOT}"
fi

make -C "${PROJECT_ROOT}/firmware" extract-sdk
bash "${SCRIPT_DIR}/seed-zsh-history.sh"

printf '\n%s\n%s\n%s\n%s\n' \
  'Mentor Pi development workspace is ready.' \
  'Firmware:       make -C firmware build' \
  'micro-ROS Agent: make -C micro_ros_agent build' \
  'ROS workspace:   make -C ros2_ws build'
