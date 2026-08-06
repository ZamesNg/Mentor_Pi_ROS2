#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DEPS_DIR="${PROJECT_ROOT}/firmware/mentor_pi_mcu/third_party"

readonly STM32CUBE_REPOSITORY="https://github.com/STMicroelectronics/STM32CubeF4.git"
readonly STM32CUBE_COMMIT="52757b5e33259a088509a777a9e3a5b971194c7d"

readonly MICROROS_REPOSITORY="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
readonly MICROROS_COMMIT="bd531b273c1bcd070b3143c5642128ec75a6f04e"

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

  VerifyCheckout "${repository}" "${expected_commit}" "${destination}"
}

VerifyCheckout() {
  local repository="$1"
  local expected_commit="$2"
  local destination="$3"

  if [[ ! -d "${destination}/.git" ]] ||
     [[ "$(git -C "${destination}" rev-parse --is-inside-work-tree 2>/dev/null)" != "true" ]]; then
    echo "Dependency is not a standalone Git checkout: ${destination}" >&2
    return 1
  fi

  local remote_count
  remote_count="$(git -C "${destination}" remote | awk 'END {print NR}')"
  local origin_url_count
  origin_url_count="$(git -C "${destination}" config --get-all remote.origin.url | \
    awk 'END {print NR}' || true)"
  local origin_push_url_count
  origin_push_url_count="$(git -C "${destination}" config --get-all remote.origin.pushurl | \
    awk 'END {print NR}' || true)"
  local actual_repository
  actual_repository="$(git -C "${destination}" config --get remote.origin.url 2>/dev/null || true)"
  if [[ "${remote_count}" != "1" ]] ||
     [[ "${origin_url_count}" != "1" ]] ||
     [[ "${origin_push_url_count}" != "0" ]] ||
     [[ "${actual_repository}" != "${repository}" ]]; then
    echo "Dependency origin mismatch at ${destination}" >&2
    echo "expected sole origin: ${repository}" >&2
    echo "actual origin:        ${actual_repository:-<missing>}" >&2
    return 1
  fi

  if git -C "${destination}" symbolic-ref -q HEAD >/dev/null 2>&1; then
    echo "Dependency must be checked out at a detached HEAD: ${destination}" >&2
    return 1
  fi

  local actual_commit
  actual_commit="$(git -C "${destination}" rev-parse --verify 'HEAD^{commit}' 2>/dev/null || true)"
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "Dependency revision mismatch at ${destination}" >&2
    echo "expected: ${expected_commit}" >&2
    echo "actual:   ${actual_commit}" >&2
    return 1
  fi

  local status
  status="$(git -C "${destination}" status --porcelain=v1 \
    --untracked-files=all --ignore-submodules=none)"
  if [[ -n "${status}" ]]; then
    echo "Dependency checkout is dirty at ${destination}" >&2
    printf '%s\n' "${status}" >&2
    return 1
  fi
}

if [[ "$#" -eq 4 && "$1" == "--verify-existing" ]]; then
  VerifyCheckout "$2" "$3" "$4"
  exit 0
elif [[ "$#" -ne 0 ]]; then
  echo "Usage: $0 [--verify-existing REPOSITORY COMMIT CHECKOUT]" >&2
  exit 2
fi

mkdir -p "${DEPS_DIR}"

CloneAndVerify "${STM32CUBE_REPOSITORY}" "${STM32CUBE_COMMIT}" \
  "${DEPS_DIR}/stm32cube_f4"
CloneAndVerify "${MICROROS_REPOSITORY}" "${MICROROS_COMMIT}" \
  "${DEPS_DIR}/micro_ros_stm32cubemx_utils"

echo "Firmware dependencies are present and match the pinned revisions."
