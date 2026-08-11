#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly DEPENDENCY_ROOT="${MCU_ROOT}/third_party"
readonly LOCAL_TOOLCHAIN="${MCU_ROOT}/.deps/arm-gnu-toolchain"
readonly TOOLCHAIN_VERSION="13.2.1"

verify_only=0
[[ "$#" -le 1 ]] || { echo "Usage: setup.sh [--verify]" >&2; exit 2; }
if [[ "${1:-}" == --verify ]]; then
  verify_only=1
elif [[ "$#" == 1 ]]; then
  echo "Usage: setup.sh [--verify]" >&2
  exit 2
fi

Fail() {
  echo "Firmware setup error: $*" >&2
  exit 1
}

"${SCRIPT_DIR}/check_environment.sh" >/dev/null

VerifyCheckout() {
  local repository="$1" commit="$2" checkout="$3"
  [[ -d "${checkout}/.git" ]] || Fail "missing dependency ${checkout}"
  [[ "$(git -C "${checkout}" remote get-url origin)" == "${repository}" ]] || \
    Fail "dependency origin mismatch at ${checkout}"
  [[ "$(git -C "${checkout}" rev-parse HEAD)" == "${commit}" ]] || \
    Fail "dependency revision mismatch at ${checkout}"
  [[ -z "$(git -C "${checkout}" status --porcelain=v1 \
    --untracked-files=all)" ]] || Fail "dependency checkout is dirty: ${checkout}"
}

FetchCheckout() {
  local repository="$1" commit="$2" checkout="$3"
  if [[ ! -d "${checkout}/.git" ]]; then
    [[ ! -e "${checkout}" ]] || Fail "refusing to replace ${checkout}"
    git init "${checkout}"
    git -C "${checkout}" remote add origin "${repository}"
    git -C "${checkout}" fetch --depth 1 origin "${commit}"
    git -C "${checkout}" checkout --detach FETCH_HEAD
  fi
  VerifyCheckout "${repository}" "${commit}" "${checkout}"
}

readonly CUBE_URL="https://github.com/STMicroelectronics/STM32CubeF4.git"
readonly CUBE_COMMIT="52757b5e33259a088509a777a9e3a5b971194c7d"
readonly MICROROS_URL="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
readonly MICROROS_COMMIT="bd531b273c1bcd070b3143c5642128ec75a6f04e"

if ((verify_only == 1)); then
  VerifyCheckout "${CUBE_URL}" "${CUBE_COMMIT}" \
    "${DEPENDENCY_ROOT}/stm32cube_f4"
  VerifyCheckout "${MICROROS_URL}" "${MICROROS_COMMIT}" \
    "${DEPENDENCY_ROOT}/micro_ros_stm32cubemx_utils"
else
  mkdir -p "${DEPENDENCY_ROOT}"
  FetchCheckout "${CUBE_URL}" "${CUBE_COMMIT}" \
    "${DEPENDENCY_ROOT}/stm32cube_f4"
  FetchCheckout "${MICROROS_URL}" "${MICROROS_COMMIT}" \
    "${DEPENDENCY_ROOT}/micro_ros_stm32cubemx_utils"
fi

FindToolchain() {
  local root
  for root in "${ARM_GNU_TOOLCHAIN_ROOT:-}" /opt/arm-gnu-toolchain \
      "${LOCAL_TOOLCHAIN}"; do
    [[ -n "${root}" && -x "${root}/bin/arm-none-eabi-gcc" ]] || continue
    if [[ "$("${root}/bin/arm-none-eabi-gcc" -dumpfullversion)" == \
        "${TOOLCHAIN_VERSION}" ]]; then
      printf '%s' "${root}"
      return 0
    fi
  done
  return 1
}

if toolchain="$(FindToolchain)"; then
  echo "Using verified Arm GNU ${TOOLCHAIN_VERSION}: ${toolchain}"
elif ((verify_only == 1)); then
  Fail "Arm GNU ${TOOLCHAIN_VERSION} is unavailable; run make setup"
else
  case "$(uname -m)" in
    x86_64 | amd64)
      host=x86_64
      expected=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb
      ;;
    aarch64 | arm64)
      host=aarch64
      expected=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a
      ;;
  esac
  archive="arm-gnu-toolchain-13.2.rel1-${host}-arm-none-eabi.tar.xz"
  download="${MCU_ROOT}/.deps/downloads/${archive}"
  mkdir -p "$(dirname "${download}")" "${LOCAL_TOOLCHAIN}"
  if [[ ! -f "${download}" ]]; then
    command -v curl >/dev/null 2>&1 || Fail "curl is required to fetch Arm GNU"
    curl -fL --retry 5 --retry-all-errors --continue-at - \
      --connect-timeout 30 --max-time 1800 \
      "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/13.2.rel1/binrel/${archive}" \
      -o "${download}"
  fi
  [[ "$("${SCRIPT_DIR}/sha256.sh" "${download}")" == "${expected}" ]] || \
    Fail "Arm GNU archive checksum mismatch"
  [[ -z "$(find "${LOCAL_TOOLCHAIN}" -mindepth 1 -print -quit)" ]] || \
    Fail "incomplete local toolchain directory exists: ${LOCAL_TOOLCHAIN}"
  tar -xJf "${download}" -C "${LOCAL_TOOLCHAIN}" --strip-components=1
  [[ "$("${LOCAL_TOOLCHAIN}/bin/arm-none-eabi-gcc" -dumpfullversion)" == \
    "${TOOLCHAIN_VERSION}" ]] || Fail "extracted toolchain version mismatch"
  printf '%s\n' "${expected}" >"${MCU_ROOT}/.deps/toolchain-archive.sha256"
  echo "Installed verified Arm GNU ${TOOLCHAIN_VERSION}: ${LOCAL_TOOLCHAIN}"
fi

"${SCRIPT_DIR}/extract_microros_sdk.sh"
echo "Firmware dependencies are ready."
