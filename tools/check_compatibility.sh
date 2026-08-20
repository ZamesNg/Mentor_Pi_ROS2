#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SDK_MANIFEST="${PROJECT_ROOT}/firmware/mentor_pi_mcu/sdk/humble/manifest.txt"

Fail() { echo "Component compatibility error: $*" >&2; exit 1; }
[[ -f "${SDK_MANIFEST}" ]] || Fail "firmware SDK manifest is missing"
"${PROJECT_ROOT}/firmware/tools/validate_sdk.sh" >/dev/null
grep -Fqx 'ros_distro=humble' "${PROJECT_ROOT}/micro_ros_agent/sources.lock" || \
  Fail "Agent is not locked to Humble"
grep -Fqx 'ros_distro=humble' "${SDK_MANIFEST}" || \
  Fail "firmware SDK is not Humble"
for package in mentor_pi_interfaces mentor_pi_bringup mentor_pi_hardwares; do
  [[ -f "${PROJECT_ROOT}/ros2_ws/src/${package}/package.xml" ]] || \
    Fail "ROS package is missing: ${package}"
done
for removed in mentor_pi_tracking mentor_pi_tracking_interfaces; do
  [[ ! -e "${PROJECT_ROOT}/ros2_ws/src/${removed}" ]] || \
    Fail "retired ROS package remains: ${removed}"
done
[[ ! -e "${PROJECT_ROOT}/mentor_pi_ros2" ]] || \
  Fail "legacy mentor_pi_ros2 workspace still exists"
echo "Component compatibility passed: Humble Agent, SDK, and ROS interfaces agree."
