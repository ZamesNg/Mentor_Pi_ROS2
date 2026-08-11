#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly BUILD_ROOT="${MCU_ROOT}/build/domain"
readonly JOBS="${1:-1}"
[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || { echo "invalid build job count" >&2; exit 2; }
cmake -S "${MCU_ROOT}" -B "${BUILD_ROOT}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DMENTOR_PI_MICROROS_SDK_ROOT="${MCU_ROOT}/build/microros-sdk"
cmake --build "${BUILD_ROOT}" --parallel "${JOBS}"
ctest --test-dir "${BUILD_ROOT}" --output-on-failure
