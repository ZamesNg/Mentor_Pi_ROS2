#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly BUILD_ROOT="${MCU_ROOT}/build/stm32"
readonly METADATA="${BUILD_ROOT}/rrclite-build-metadata.txt"
readonly SDK_MANIFEST="${MCU_ROOT}/sdk/humble/manifest.txt"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"

Fail() { echo "Firmware verification error: $*" >&2; exit 1; }
ReadValue() {
  local file="$1" key="$2" value
  value="$(sed -n "s/^${key}=//p" "${file}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "${file} must contain one ${key} value"
  printf '%s' "${value}"
}

[[ -f "${METADATA}" && ! -L "${METADATA}" ]] || Fail "build metadata is missing"
for expected in \
    'schema=mentor-pi-firmware-build-v3' \
    'target=STM32F407VET6' \
    'ros_distro=humble' \
    'builder_mode=native-pinned' \
    'motor_mode=ADRC' \
    'control_mode=CLOSED_LOOP' \
    'artifact_mode=NORMAL' \
    'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
    'release_qualified=0'; do
  grep -Fqx "${expected}" "${METADATA}" || Fail "metadata lacks ${expected}"
done
[[ "$(ReadValue "${METADATA}" source_sha256)" == \
   "$("${SCRIPT_DIR}/source_fingerprint.sh")" ]] || Fail "firmware sources changed"
interface_sha="$("${SCRIPT_DIR}/interface_fingerprint.sh")"
[[ "$(ReadValue "${METADATA}" interfaces_sha256)" == "${interface_sha}" && \
   "$(ReadValue "${SDK_MANIFEST}" interfaces_sha256)" == "${interface_sha}" ]] || \
  Fail "firmware SDK is stale relative to mentor_pi_interfaces"
[[ "$(ReadValue "${METADATA}" microros_sdk_archive_sha256)" == \
   "$("${SHA256}" "${MCU_ROOT}/sdk/humble/libmicroros.tar.xz")" ]] || \
  Fail "checked SDK archive changed after the build"
[[ "$(ReadValue "${METADATA}" microros_sdk_tree_sha256)" == \
   "$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${MCU_ROOT}/build/microros-sdk")" ]] || \
  Fail "extracted SDK changed after the build"
for extension in elf hex bin map; do
  artifact="${BUILD_ROOT}/mentor_pi_mcu.${extension}"
  [[ -s "${artifact}" && ! -L "${artifact}" ]] || Fail "missing ${artifact}"
  [[ "$(ReadValue "${METADATA}" "${extension}_sha256")" == \
     "$("${SHA256}" "${artifact}")" ]] || Fail "${extension} hash mismatch"
done
echo "Verified NORMAL_CLOSED_LOOP_DEFAULT firmware: ${BUILD_ROOT}/mentor_pi_mcu.elf"
