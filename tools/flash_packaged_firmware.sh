#!/usr/bin/env bash

# Flash a verified board-handoff PID artifact without rebuilding it.
set -euo pipefail
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
Fail() { echo "Packaged firmware flash error: $*" >&2; exit 1; }
[[ "$#" == 2 ]] || { echo "Usage: flash_packaged_firmware.sh /dev/mentor_pi_mcu BOARD_HANDOFF_DIRECTORY" >&2; exit 2; }
readonly PORT="$1"
readonly HANDOFF="$2"
[[ "${HANDOFF}" == /* && -d "${HANDOFF}" && ! -L "${HANDOFF}" ]] || \
  Fail "board handoff must be an absolute non-symbolic directory"
[[ -f "${HANDOFF}/HANDOFF.txt" && -f "${HANDOFF}/SHA256SUMS" ]] || \
  Fail "board handoff manifest is missing"
(cd "${HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null) || Fail "board handoff checksum verification failed"
readonly RELEASE="${HANDOFF}/firmware-pid-release"
[[ -f "${RELEASE}/BUILD-MODE.txt" && -f "${RELEASE}/BUILD-METADATA.txt" ]] || \
  Fail "PID release metadata is missing"
grep -Fqx 'classification=NORMAL_CLOSED_LOOP_DEFAULT' "${RELEASE}/BUILD-MODE.txt" || \
  Fail "board handoff is not the normal closed-loop PID release"
(cd "${RELEASE}" && sha256sum --check SHA256SUMS >/dev/null) || Fail "PID release checksum verification failed"
exec env RRCLITE_FLASH_FIRMWARE_DIRECTORY="${RELEASE}" \
  "${SCRIPT_DIR}/guided_flash.sh" "${PORT}"
