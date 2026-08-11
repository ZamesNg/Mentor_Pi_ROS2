#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly MCU_ROOT="$(cd "${SCRIPT_DIR}/../mentor_pi_mcu" && pwd)"
readonly MAP="${MCU_ROOT}/build/stm32/mentor_pi_mcu.map"
"${SCRIPT_DIR}/verify.sh" >/dev/null

Used() {
  local symbol="$1" origin="$2" address
  address="$(awk -v wanted="${symbol}" '$2 == wanted && $3 == "=" {print $1; exit}' "${MAP}")"
  [[ "${address}" =~ ^0x[0-9a-fA-F]+$ ]] || {
    echo "Missing memory symbol ${symbol}" >&2
    exit 1
  }
  printf '%u' "$((address - origin))"
}
flash="$(Used __flash_image_end__ 0x08000000)"
sram="$(Used __ram_used_end__ 0x20000000)"
ccm="$(Used __ccm_end__ 0x10000000)"
((flash <= 419430 && sram <= 104857 && ccm <= 52428)) || {
  echo "Firmware exceeds the 80% memory budget" >&2
  exit 1
}
printf 'Firmware memory passed: flash=%s/524288 sram=%s/131072 ccm=%s/65536\n' \
  "${flash}" "${sram}" "${ccm}"
