#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly BUILD_ROOT="${MCU_ROOT}/build/host-tools"
readonly OUTPUT="${BUILD_ROOT}/ch9102_boot_control"

Fail() {
  echo "CH9102F boot-control build error: $*" >&2
  exit 1
}

[[ "$#" == 0 ]] || Fail "this helper takes no arguments"
"${SCRIPT_DIR}/check_environment.sh" >/dev/null
"${SCRIPT_DIR}/extract_microros_sdk.sh" >/dev/null
cmake -S "${MCU_ROOT}" -B "${BUILD_ROOT}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MICROROS_SDK_ROOT="${MCU_ROOT}/build/microros-sdk" \
  >/dev/null
cmake --build "${BUILD_ROOT}" --target mentor_pi_ch9102_boot_control \
  >/dev/null
[[ -x "${OUTPUT}" && ! -L "${OUTPUT}" ]] || \
  Fail "CMake/Ninja did not produce ${OUTPUT}"
printf '%s\n' "${OUTPUT}"
