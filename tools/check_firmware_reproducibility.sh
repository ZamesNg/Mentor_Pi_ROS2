#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly FIRMWARE_IMAGE="mentor-pi/rrclite-firmware-builder:gcc-13.2.1"
readonly REPORT_ROOT="${RRCLITE_REPRO_REPORT_DIR:-${PROJECT_ROOT}/build/firmware-reproducibility}"
SNAPSHOT_ROOT="$(mktemp -d)"
readonly SNAPSHOT_ROOT

Cleanup() {
  cmake -E remove_directory "${SNAPSHOT_ROOT}"
}
trap Cleanup EXIT

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "Neither sha256sum nor shasum is installed." >&2
    return 1
  fi
}

MapSymbolUsedBytes() {
  local symbol="$1"
  local origin="$2"
  local address
  address="$(
    awk -v wanted_symbol="${symbol}" \
      '$2 == wanted_symbol && $3 == "=" { print $1; exit }' \
      "${BUILD_ROOT}/mentor_pi_mcu.map"
  )"
  [[ "${address}" =~ ^0x[0-9a-fA-F]+$ ]] || {
    echo "Could not find ${symbol} in the firmware map." >&2
    return 1
  }
  printf '%u' "$((address - origin))"
}

RequireWithinBudget() {
  local name="$1"
  local used_bytes="$2"
  local maximum_bytes="$3"
  if [[ ! "${used_bytes}" =~ ^[0-9]+$ ||
        ! "${maximum_bytes}" =~ ^[0-9]+$ ]]; then
    echo "${name} budget values must be nonempty decimal byte counts." >&2
    return 1
  fi
  if ((used_bytes > maximum_bytes)); then
    echo "${name} usage ${used_bytes} exceeds ${maximum_bytes}." >&2
    return 1
  fi
}

CleanBuild() {
  local build_number="$1"
  echo "Starting clean locked firmware build ${build_number}/2"
  cmake -E remove_directory "${BUILD_ROOT}"
  RRCLITE_MOTOR_COMMISSIONING=0 \
    RRCLITE_MOTOR_COMMISSIONING_ACK= \
    "${PROJECT_ROOT}/tools/build_firmware.sh"
}

GenerateLoadableImage() {
  local output_path="${BUILD_ROOT}/mentor_pi_mcu.loadable.bin"
  if command -v arm-none-eabi-objcopy >/dev/null 2>&1 && \
      [[ "$(arm-none-eabi-gcc -dumpfullversion 2>/dev/null || true)" == \
         "13.2.1" ]]; then
    arm-none-eabi-objcopy -O binary \
      "${BUILD_ROOT}/mentor_pi_mcu.elf" "${output_path}"
  else
    command -v docker >/dev/null 2>&1 || {
      echo "Docker or the pinned local Arm GNU 13.2.1 toolchain is required." >&2
      return 1
    }
    docker image inspect "${FIRMWARE_IMAGE}" >/dev/null 2>&1 || {
      echo "Pinned firmware builder image is unavailable after the build." >&2
      return 1
    }
    docker run --rm \
      --user "$(id -u):$(id -g)" \
      --volume "${PROJECT_ROOT}:/workspace" \
      --workdir /workspace \
      "${FIRMWARE_IMAGE}" \
      arm-none-eabi-objcopy -O binary \
      firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf \
      firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.loadable.bin
  fi
  cmp "${output_path}" "${BUILD_ROOT}/mentor_pi_mcu.bin"
}

if [[ "${RRCLITE_MOTOR_COMMISSIONING:-0}" != "0" ]]; then
  echo "Reproducibility qualification is restricted to the motor-locked image." >&2
  exit 2
fi

CleanBuild 1
GenerateLoadableImage
cmake -E copy "${BUILD_ROOT}/mentor_pi_mcu.loadable.bin" \
  "${SNAPSHOT_ROOT}/first.loadable.bin"
cmake -E copy "${BUILD_ROOT}/mentor_pi_mcu.hex" \
  "${SNAPSHOT_ROOT}/first.hex"

CleanBuild 2
GenerateLoadableImage

cmp "${SNAPSHOT_ROOT}/first.loadable.bin" \
  "${BUILD_ROOT}/mentor_pi_mcu.loadable.bin"
cmp "${SNAPSHOT_ROOT}/first.hex" "${BUILD_ROOT}/mentor_pi_mcu.hex"

LOADABLE_SHA256="$(
  Sha256 "${BUILD_ROOT}/mentor_pi_mcu.loadable.bin"
)"
BINARY_SHA256="$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.bin")"
HEX_SHA256="$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.hex")"
ELF_SHA256="$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.elf")"
MAP_SHA256="$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.map")"
readonly LOADABLE_SHA256 BINARY_SHA256 HEX_SHA256 ELF_SHA256 MAP_SHA256
for hash in "${LOADABLE_SHA256}" "${BINARY_SHA256}" "${HEX_SHA256}" \
  "${ELF_SHA256}" "${MAP_SHA256}"; do
  [[ "${hash}" =~ ^[0-9a-f]{64}$ ]] || {
    echo "A firmware artifact SHA-256 is malformed." >&2
    exit 1
  }
done
[[ "${LOADABLE_SHA256}" == "${BINARY_SHA256}" ]]
grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING:BOOL=OFF' \
  "${BUILD_ROOT}/CMakeCache.txt"

readonly FLASH_TOTAL_BYTES=524288
readonly SRAM_TOTAL_BYTES=131072
readonly CCM_TOTAL_BYTES=65536
readonly FLASH_MAXIMUM_BYTES=419430
readonly SRAM_MAXIMUM_BYTES=104857
readonly CCM_MAXIMUM_BYTES=52428
FLASH_USED_BYTES="$(MapSymbolUsedBytes __flash_image_end__ 0x08000000)"
SRAM_USED_BYTES="$(MapSymbolUsedBytes __ram_used_end__ 0x20000000)"
CCM_USED_BYTES="$(MapSymbolUsedBytes __ccm_end__ 0x10000000)"
readonly FLASH_USED_BYTES SRAM_USED_BYTES CCM_USED_BYTES
RequireWithinBudget flash "${FLASH_USED_BYTES}" "${FLASH_MAXIMUM_BYTES}"
RequireWithinBudget sram "${SRAM_USED_BYTES}" "${SRAM_MAXIMUM_BYTES}"
RequireWithinBudget ccm "${CCM_USED_BYTES}" "${CCM_MAXIMUM_BYTES}"

cmake -E make_directory "${REPORT_ROOT}"
printf '%s\n' \
  'target=STM32F407VET6' \
  'toolchain=arm-none-eabi-gcc-13.2.1' \
  'mode=motor-locked' \
  'builds=2' \
  'comparison=loadable-binary-and-ihex' \
  "elf_sha256=${ELF_SHA256}" \
  "binary_sha256=${BINARY_SHA256}" \
  "loadable_sha256=${LOADABLE_SHA256}" \
  "ihex_sha256=${HEX_SHA256}" \
  "map_sha256=${MAP_SHA256}" \
  "flash_used_bytes=${FLASH_USED_BYTES}" \
  "flash_total_bytes=${FLASH_TOTAL_BYTES}" \
  "flash_free_bytes=$((FLASH_TOTAL_BYTES - FLASH_USED_BYTES))" \
  "sram_used_bytes=${SRAM_USED_BYTES}" \
  "sram_total_bytes=${SRAM_TOTAL_BYTES}" \
  "sram_free_bytes=$((SRAM_TOTAL_BYTES - SRAM_USED_BYTES))" \
  "ccm_used_bytes=${CCM_USED_BYTES}" \
  "ccm_total_bytes=${CCM_TOTAL_BYTES}" \
  "ccm_free_bytes=$((CCM_TOTAL_BYTES - CCM_USED_BYTES))" \
  'resource_budget_result=pass' \
  'result=identical' \
  >"${REPORT_ROOT}/firmware-reproducibility.txt"

echo "Two clean firmware builds have identical loadable bytes and Intel HEX."
echo "Loadable SHA-256: ${LOADABLE_SHA256}"
printf 'Memory used: flash %s/%s, SRAM %s/%s, CCM %s/%s bytes\n' \
  "${FLASH_USED_BYTES}" "${FLASH_TOTAL_BYTES}" \
  "${SRAM_USED_BYTES}" "${SRAM_TOTAL_BYTES}" \
  "${CCM_USED_BYTES}" "${CCM_TOTAL_BYTES}"
echo "Report: ${REPORT_ROOT}/firmware-reproducibility.txt"
