#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
readonly SOURCE="${PROJECT_ROOT}/ros2_ws/src/mentor_pi_interfaces"
readonly GENERATOR="${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_library_generation.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

cp -a "${SOURCE}" "${TEST_ROOT}/mentor_pi_interfaces"
"${SCRIPT_DIR}/validate_sdk.sh" "${TEST_ROOT}/mentor_pi_interfaces" >/dev/null
printf '\n# deliberate stale-SDK test mutation\n' \
  >>"${TEST_ROOT}/mentor_pi_interfaces/msg/MotorCommand.msg"
if "${SCRIPT_DIR}/validate_sdk.sh" "${TEST_ROOT}/mentor_pi_interfaces" \
    >"${TEST_ROOT}/output" 2>&1; then
  echo "SDK validation accepted a changed interface" >&2
  exit 1
fi
grep -Fq 'SDK is stale relative to mentor_pi_interfaces' "${TEST_ROOT}/output"
grep -Fq 'LOCAL_MICROROS_TOOLS=' "${GENERATOR}"
grep -Fq 'cp -a "${INSTALLED_MICROROS_TOOLS}" "${LOCAL_MICROROS_TOOLS}"' \
  "${GENERATOR}"
if grep -F 'sed -i' -A2 "${GENERATOR}" | \
    grep -Fq 'INSTALLED_MICROROS_TOOLS'; then
  echo "SDK generator patches the installed micro-ROS overlay" >&2
  exit 1
fi
echo "Firmware SDK stale-interface rejection passed."
