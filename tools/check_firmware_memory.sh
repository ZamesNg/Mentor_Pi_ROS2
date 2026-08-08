#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() {
  echo "Firmware memory check failed: $*" >&2
  exit 1
}

Usage() {
  echo "Usage: ./tools/check_firmware_memory.sh LOCKED|COMMISSIONING|COMMISSIONING_PID [PROJECT_ROOT]" >&2
}

[[ "$#" -ge 1 && "$#" -le 2 ]] || {
  Usage
  exit 2
}
readonly MODE="$1"
readonly PROJECT_ROOT="${2:-${DEFAULT_PROJECT_ROOT}}"
case "${MODE}" in
  LOCKED | COMMISSIONING | COMMISSIONING_PID) ;;
  *)
    Usage
    Fail "mode must identify the exact verified artifact"
    ;;
esac

readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly MAP_FILE="${BUILD_ROOT}/mentor_pi_mcu.map"
readonly ARTIFACT_VERIFIER="${PROJECT_ROOT}/tools/verify_firmware_artifact.sh"
[[ -x "${ARTIFACT_VERIFIER}" ]] || Fail "artifact verifier is unavailable"
[[ -s "${MAP_FILE}" ]] || Fail "firmware map is missing or empty"
"${ARTIFACT_VERIFIER}" "${MODE}" "${PROJECT_ROOT}" >/dev/null

MapSymbolUsedBytes() {
  local symbol="$1"
  local origin="$2"
  local address
  address="$(
    awk -v wanted_symbol="${symbol}" \
      '$2 == wanted_symbol && $3 == "=" { print $1; exit }' \
      "${MAP_FILE}"
  )"
  [[ "${address}" =~ ^0x[0-9a-fA-F]+$ ]] || \
    Fail "could not find ${symbol} in the firmware map"
  printf '%u' "$((address - origin))"
}

RequireWithinBudget() {
  local region="$1"
  local used="$2"
  local maximum="$3"
  ((used <= maximum)) || \
    Fail "${region} usage ${used} bytes exceeds the 80% gate of ${maximum} bytes"
}

readonly FLASH_TOTAL_BYTES=524288
readonly SRAM_TOTAL_BYTES=131072
readonly CCM_TOTAL_BYTES=65536
readonly FLASH_MAXIMUM_BYTES=419430
readonly SRAM_MAXIMUM_BYTES=104857
readonly CCM_MAXIMUM_BYTES=52428
readonly FLASH_USED_BYTES="$(MapSymbolUsedBytes __flash_image_end__ 0x08000000)"
readonly SRAM_USED_BYTES="$(MapSymbolUsedBytes __ram_used_end__ 0x20000000)"
readonly CCM_USED_BYTES="$(MapSymbolUsedBytes __ccm_end__ 0x10000000)"

RequireWithinBudget flash "${FLASH_USED_BYTES}" "${FLASH_MAXIMUM_BYTES}"
RequireWithinBudget sram "${SRAM_USED_BYTES}" "${SRAM_MAXIMUM_BYTES}"
RequireWithinBudget ccm "${CCM_USED_BYTES}" "${CCM_MAXIMUM_BYTES}"

printf '%s\n' \
  "Firmware memory headroom passed for ${MODE}." \
  "flash=${FLASH_USED_BYTES}/${FLASH_TOTAL_BYTES} free=$((FLASH_TOTAL_BYTES - FLASH_USED_BYTES))" \
  "sram=${SRAM_USED_BYTES}/${SRAM_TOTAL_BYTES} free=$((SRAM_TOTAL_BYTES - SRAM_USED_BYTES))" \
  "ccm=${CCM_USED_BYTES}/${CCM_TOTAL_BYTES} free=$((CCM_TOTAL_BYTES - CCM_USED_BYTES))"
