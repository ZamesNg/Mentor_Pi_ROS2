#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly BUILD_ROOT="${MCU_ROOT}/build/stm32"
readonly SDK_ROOT="${MCU_ROOT}/build/microros-sdk"
readonly SDK_MANIFEST="${MCU_ROOT}/sdk/humble/manifest.txt"
readonly JOBS="${RRCLITE_BUILD_JOBS:-1}"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"

Fail() {
  echo "Firmware build error: $*" >&2
  exit 1
}

if [[ "${1:-}" == --print-motor-profile && "$#" == 1 ]]; then
  printf '%s\n' \
    'mode=PID' \
    'control_mode=CLOSED_LOOP' \
    'maximum_accepted_rps=6.0' \
    'output_limit_permille=1000' \
    'release_qualified=0'
  exit 0
fi
[[ "$#" == 0 ]] || Fail "usage: build.sh [--print-motor-profile]"

[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || Fail "RRCLITE_BUILD_JOBS must be positive"
"${SCRIPT_DIR}/setup.sh" --verify >/dev/null
"${SCRIPT_DIR}/validate_sdk.sh" >/dev/null
"${SCRIPT_DIR}/extract_microros_sdk.sh" >/dev/null

for root in "${ARM_GNU_TOOLCHAIN_ROOT:-}" /opt/arm-gnu-toolchain \
    "${MCU_ROOT}/.deps/arm-gnu-toolchain"; do
  if [[ -n "${root}" && -x "${root}/bin/arm-none-eabi-gcc" && \
      "$("${root}/bin/arm-none-eabi-gcc" -dumpfullversion)" == 13.2.1 ]]; then
    toolchain_root="${root}"
    break
  fi
done
[[ -n "${toolchain_root:-}" ]] || Fail "verified Arm GNU 13.2.1 is unavailable"
readonly toolchain_root
export PATH="${toolchain_root}/bin:${PATH}"
export SOURCE_DATE_EPOCH=0

interface_sha="$("${SCRIPT_DIR}/interface_fingerprint.sh")"
sdk_interface_sha="$(sed -n 's/^interfaces_sha256=//p' "${SDK_MANIFEST}")"
[[ "${interface_sha}" == "${sdk_interface_sha}" ]] || \
  Fail "mentor_pi_interfaces changed; regenerate and commit the firmware SDK"
sdk_archive_sha="$("${SHA256}" "${MCU_ROOT}/sdk/humble/libmicroros.tar.xz")"
sdk_tree_sha="$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${SDK_ROOT}")"
source_sha_before="$("${SCRIPT_DIR}/source_fingerprint.sh")"

rm -rf -- "${BUILD_ROOT}"
cmake -S "${MCU_ROOT}/target/stm32" -B "${BUILD_ROOT}" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${MCU_ROOT}/target/stm32/arm-none-eabi-toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DRRCLITE_MICROROS_ROOT="${SDK_ROOT}"
cmake --build "${BUILD_ROOT}" --parallel "${JOBS}"

for extension in elf hex bin map; do
  [[ -s "${BUILD_ROOT}/mentor_pi_mcu.${extension}" ]] || \
    Fail "missing mentor_pi_mcu.${extension}"
done
[[ "${source_sha_before}" == "$("${SCRIPT_DIR}/source_fingerprint.sh")" ]] || \
  Fail "firmware sources changed during the build"

case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
esac
if [[ -f /.dockerenv ]]; then environment=devcontainer; else environment=native; fi
metadata="${BUILD_ROOT}/rrclite-build-metadata.txt"
printf '%s\n' \
  'schema=mentor-pi-firmware-build-v3' \
  'target=STM32F407VET6' \
  'ros_distro=humble' \
  'builder_mode=native-pinned' \
  "build_environment=${environment}" \
  'host_os=ubuntu-22.04' \
  "host_architecture=${architecture}" \
  'toolchain=arm-gnu-toolchain-13.2.rel1' \
  'motor_mode=PID' \
  'control_mode=CLOSED_LOOP' \
  'artifact_mode=NORMAL' \
  'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
  'release_qualified=0' \
  "source_sha256=${source_sha_before}" \
  "interfaces_sha256=${interface_sha}" \
  "microros_sdk_archive_sha256=${sdk_archive_sha}" \
  "microros_sdk_tree_sha256=${sdk_tree_sha}" \
  "elf_sha256=$("${SHA256}" "${BUILD_ROOT}/mentor_pi_mcu.elf")" \
  "hex_sha256=$("${SHA256}" "${BUILD_ROOT}/mentor_pi_mcu.hex")" \
  "bin_sha256=$("${SHA256}" "${BUILD_ROOT}/mentor_pi_mcu.bin")" \
  "map_sha256=$("${SHA256}" "${BUILD_ROOT}/mentor_pi_mcu.map")" \
  >"${metadata}"
"${SCRIPT_DIR}/verify.sh" >/dev/null
"${SCRIPT_DIR}/check_memory.sh"
echo "Firmware artifacts: ${BUILD_ROOT}"
