#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu/build/stm32"
requested="${1:-}"
if [[ -z "${requested}" ]]; then
  requested="${FIRMWARE_ROOT}/build/packages/$(date -u +%Y%m%dT%H%M%SZ)"
elif [[ "${requested}" != /* ]]; then
  requested="${FIRMWARE_ROOT}/${requested}"
fi
[[ ! -e "${requested}" && ! -L "${requested}" ]] || {
  echo "Package destination already exists: ${requested}" >&2
  exit 1
}
destination="${requested}/firmware-adrc-release"
mkdir -p "${destination}"
for extension in elf hex bin map; do
  cp "${BUILD_ROOT}/mentor_pi_mcu.${extension}" \
    "${destination}/mentor_pi_mcu-firmware-adrc-release.${extension}"
done
cp "${BUILD_ROOT}/rrclite-build-metadata.txt" \
  "${destination}/BUILD-METADATA.txt"
printf '%s\n' \
  'target=STM32F407VET6' \
  'motor_mode=ADRC' \
  'control_mode=CLOSED_LOOP' \
  'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
  >"${destination}/BUILD-MODE.txt"
(
  cd "${destination}"
  find . -type f ! -name SHA256SUMS -print | LC_ALL=C sort | \
    while IFS= read -r file; do
      printf '%s  %s\n' "$("${SCRIPT_DIR}/sha256.sh" "${file}")" "${file#./}"
    done >SHA256SUMS
)
echo "Firmware release package: ${destination}"
