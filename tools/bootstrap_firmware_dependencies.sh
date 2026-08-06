#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DEPS_DIR="${PROJECT_ROOT}/firmware/mentor_pi_mcu/third_party"

readonly STM32CUBE_REPOSITORY="https://github.com/STMicroelectronics/STM32CubeF4.git"
readonly STM32CUBE_COMMIT="52757b5e33259a088509a777a9e3a5b971194c7d"

readonly MICROROS_REPOSITORY="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
readonly MICROROS_COMMIT="a5b2127495ae0ab53d7a1360beaf17822309a3cc"

CloneAndVerify() {
  local repository="$1"
  local expected_commit="$2"
  local destination="$3"

  if [[ ! -d "${destination}/.git" ]]; then
    if [[ -e "${destination}" ]]; then
      echo "Refusing to replace non-Git path: ${destination}" >&2
      return 1
    fi
    git init "${destination}"
    git -C "${destination}" remote add origin "${repository}"
    git -C "${destination}" fetch --depth 1 origin "${expected_commit}"
    git -C "${destination}" checkout --detach FETCH_HEAD
  fi

  local actual_commit
  actual_commit="$(git -C "${destination}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "Dependency revision mismatch at ${destination}" >&2
    echo "expected: ${expected_commit}" >&2
    echo "actual:   ${actual_commit}" >&2
    return 1
  fi
}

mkdir -p "${DEPS_DIR}"

CloneAndVerify "${STM32CUBE_REPOSITORY}" "${STM32CUBE_COMMIT}" \
  "${DEPS_DIR}/stm32cube_f4"
CloneAndVerify "${MICROROS_REPOSITORY}" "${MICROROS_COMMIT}" \
  "${DEPS_DIR}/micro_ros_stm32cubemx_utils"

echo "Firmware dependencies are present and match the pinned revisions."
