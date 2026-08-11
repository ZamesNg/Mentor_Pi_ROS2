#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly REPOSITORY_ROOT="$(cd "${FIRMWARE_ROOT}/.." && pwd)"
readonly INTERFACE_ROOT="${1:-${REPOSITORY_ROOT}/ros2_ws/src/mentor_pi_interfaces}"
readonly MANIFEST="${MCU_ROOT}/sdk/humble/manifest.txt"
readonly ARCHIVE="${MCU_ROOT}/sdk/humble/libmicroros.tar.xz"

Fail() { echo "Firmware SDK validation error: $*" >&2; exit 1; }
ReadValue() {
  local key="$1" value
  value="$(sed -n "s/^${key}=//p" "${MANIFEST}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "manifest lacks one ${key} value"
  printf '%s' "${value}"
}

[[ -f "${MANIFEST}" && -s "${ARCHIVE}" ]] || \
  Fail "checked SDK archive or manifest is missing"
[[ "$(ReadValue format)" == mentor-pi-firmware-sdk-v1 && \
   "$(ReadValue ros_distro)" == humble && \
   "$(ReadValue toolchain)" == arm-gnu-toolchain-13.2.rel1 ]] || \
  Fail "unsupported SDK identity"
[[ "$(ReadValue toolchain_amd64_sha256)" == \
     6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb && \
   "$(ReadValue toolchain_arm64_sha256)" == \
     8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a ]] || \
  Fail "toolchain archive hashes changed"
[[ "$(ReadValue archive_sha256)" == \
   "$("${SCRIPT_DIR}/sha256.sh" "${ARCHIVE}")" ]] || \
  Fail "SDK archive hash mismatch"
[[ "$(ReadValue source_lock_sha256)" == \
   "$("${SCRIPT_DIR}/sha256.sh" "${MCU_ROOT}/config/microros_sources.lock")" ]] || \
  Fail "micro-ROS source lock changed"
[[ "$(ReadValue interfaces_sha256)" == \
   "$("${SCRIPT_DIR}/interface_fingerprint.sh" "${INTERFACE_ROOT}")" ]] || \
  Fail "SDK is stale relative to mentor_pi_interfaces"

echo "Checked Humble firmware SDK matches ${INTERFACE_ROOT}."
