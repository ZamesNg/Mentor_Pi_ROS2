#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi
readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly BUILD_IMAGE_PREPARER="${PROJECT_ROOT}/tools/prepare_build_images.sh"
readonly REPORT_ROOT="${RRCLITE_REPRO_REPORT_DIR:-${PROJECT_ROOT}/build/firmware-reproducibility}"
readonly TEMPORARY_PARENT="${TMPDIR:-/tmp}"
SNAPSHOT_ROOT="$(mktemp -d \
  "${TEMPORARY_PARENT%/}/rrclite-repro.XXXXXX")"
readonly SNAPSHOT_ROOT

Cleanup() {
  case "${SNAPSHOT_ROOT}" in
    "${TEMPORARY_PARENT%/}"/rrclite-repro.*)
      rm -rf -- "${SNAPSHOT_ROOT}"
      ;;
    *)
      echo "Refusing unsafe reproducibility-snapshot cleanup: ${SNAPSHOT_ROOT}" >&2
      return 1
      ;;
  esac
}
trap Cleanup EXIT

case "$(uname -m)" in
  x86_64 | amd64) readonly ARCHITECTURE=amd64 ;;
  aarch64 | arm64) readonly ARCHITECTURE=arm64 ;;
  *) echo "Unsupported host architecture." >&2; exit 1 ;;
esac
readonly FIRMWARE_IMAGE="$("${BUILD_IMAGE_PREPARER}" \
  --architecture "${ARCHITECTURE}" --print firmware)"

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
  echo "Starting clean PID firmware build ${build_number}/2"
  [[ "${BUILD_ROOT}" == \
      "${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32" && \
      "${BUILD_ROOT}" != "/" ]] || {
    echo "Refusing unsafe firmware-build cleanup: ${BUILD_ROOT}" >&2
    return 1
  }
  rm -rf -- "${BUILD_ROOT}"
  "${PROJECT_ROOT}/tools/build_firmware.sh"
}

GenerateLoadableImage() {
  local output_path="${BUILD_ROOT}/mentor_pi_mcu.loadable.bin"
  command -v docker >/dev/null 2>&1 || {
    echo "Docker is required." >&2
    return 1
  }
  docker image inspect "${FIRMWARE_IMAGE}" >/dev/null 2>&1 || {
    echo "Pinned firmware builder image is unavailable after the build." >&2
    return 1
  }
  docker run --rm \
    --platform "linux/${ARCHITECTURE}" \
    --user "$(id -u):$(id -g)" \
    --volume "${PROJECT_ROOT}:/workspace" \
    --workdir /workspace \
    "${FIRMWARE_IMAGE}" \
    arm-none-eabi-objcopy -O binary \
    firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf \
    firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.loadable.bin
  cmp "${output_path}" "${BUILD_ROOT}/mentor_pi_mcu.bin"
}

CleanBuild 1
GenerateLoadableImage
cp -- "${BUILD_ROOT}/mentor_pi_mcu.loadable.bin" \
  "${SNAPSHOT_ROOT}/first.loadable.bin"
cp -- "${BUILD_ROOT}/mentor_pi_mcu.hex" \
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
grep -Fqx 'CMAKE_BUILD_TYPE:STRING=MinSizeRel' \
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

mkdir -p -- "${REPORT_ROOT}"
printf '%s\n' \
  'target=STM32F407VET6' \
  'ros_distro=humble' \
  'toolchain=arm-none-eabi-gcc-13.2.1' \
  'mode=pid-default' \
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
