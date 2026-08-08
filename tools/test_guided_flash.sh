#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d)"

Cleanup() {
  [[ -d "${TEST_ROOT}" ]] || return
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Guided flash test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output
  if output="$({ "$@"; } 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "expected '${expected}' in failure: ${output}"
}

mkdir -p "${TEST_ROOT}/tools" "${TEST_ROOT}/bin"
cp "${SCRIPT_DIR}/guided_flash.sh" "${TEST_ROOT}/tools/guided_flash.sh"
chmod +x "${TEST_ROOT}/tools/guided_flash.sh"

printf '%s\n' \
  '#!/usr/bin/env bash' \
  'printf "%s\n" "${FAKE_BOOT_CONTROL}"' \
  >"${TEST_ROOT}/tools/build_ch9102_boot_control.sh"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'printf "%s\t%s\t%s\n" "${RRCLITE_AUTOMATIC_BOOT_CONTROL:-0}" "$1" "$2" >>"${FAKE_FLASH_LOG}"' \
  'if [[ "${RRCLITE_AUTOMATIC_BOOT_CONTROL:-0}" == 1 ]]; then' \
  '  exit "${FAKE_AUTOMATIC_STATUS:-0}"' \
  'fi' \
  'exit "${FAKE_MANUAL_STATUS:-0}"' \
  >"${TEST_ROOT}/tools/flash_firmware.sh"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  '[[ "$1" == --device && "$3" == --mode && "$4" == application ]]' \
  'printf "%s\n" "$4" >>"${FAKE_BOOT_LOG}"' \
  >"${TEST_ROOT}/boot-control"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'exit "${FAKE_FUSER_STATUS:-1}"' \
  >"${TEST_ROOT}/bin/fuser"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'if [[ "${FAKE_SERIAL_GROUP_TEST:-0}" != 1 ]]; then exec /usr/bin/stat "$@"; fi' \
  '[[ "$*" == "-L -c %G -- /dev/null" ]] || exit 2' \
  'printf "%s\n" mentor-pi-serial' \
  >"${TEST_ROOT}/bin/stat"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'if [[ "${FAKE_SERIAL_GROUP_TEST:-0}" != 1 ]]; then exec /usr/bin/id "$@"; fi' \
  'case "$*" in' \
  '  -un) printf "%s\n" rrclite-test ;;' \
  '  -nG) if [[ "${FAKE_GROUP_ACTIVE:-0}" == 1 ]]; then printf "%s\n" "rrclite-test mentor-pi-serial"; else printf "%s\n" rrclite-test; fi ;;' \
  '  "-nG rrclite-test") printf "%s\n" "rrclite-test mentor-pi-serial" ;;' \
  '  *) exit 2 ;;' \
  'esac' \
  >"${TEST_ROOT}/bin/id"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  '[[ "$1" == mentor-pi-serial && "$2" == -c && -n "${3:-}" ]]' \
  'printf "%s\n" "$1" >>"${FAKE_SG_LOG}"' \
  'export FAKE_GROUP_ACTIVE=1' \
  'exec /bin/bash -c "$3"' \
  >"${TEST_ROOT}/bin/sg"
chmod +x "${TEST_ROOT}/tools/build_ch9102_boot_control.sh" \
  "${TEST_ROOT}/tools/flash_firmware.sh" "${TEST_ROOT}/boot-control" \
  "${TEST_ROOT}/bin/fuser" "${TEST_ROOT}/bin/stat" \
  "${TEST_ROOT}/bin/id" "${TEST_ROOT}/bin/sg"

Run() {
  env PATH="${TEST_ROOT}/bin:${PATH}" \
    FAKE_BOOT_CONTROL="${TEST_ROOT}/boot-control" \
    FAKE_FLASH_LOG="${TEST_ROOT}/flash.log" \
    FAKE_BOOT_LOG="${TEST_ROOT}/boot.log" \
    "$@" "${TEST_ROOT}/tools/guided_flash.sh" LOCKED /dev/null
}

Run RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
grep -Fqx $'1\tLOCKED\t/dev/null' "${TEST_ROOT}/flash.log" || \
  Fail "automatic success did not call the automatic flash path"
[[ ! -e "${TEST_ROOT}/boot.log" ]] || \
  Fail "guided wrapper redundantly reset after automatic success"

: >"${TEST_ROOT}/flash.log"
: >"${TEST_ROOT}/sg.log"
Run RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  FAKE_SERIAL_GROUP_TEST=1 FAKE_SG_LOG="${TEST_ROOT}/sg.log"
grep -Fqx mentor-pi-serial "${TEST_ROOT}/sg.log" || \
  Fail "inactive serial membership was not activated with sg"
grep -Fqx $'1\tLOCKED\t/dev/null' "${TEST_ROOT}/flash.log" || \
  Fail "group-activated flash did not reach the automatic flash path"

: >"${TEST_ROOT}/flash.log"
ExpectFailure "no application reset was attempted" Run \
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  FAKE_AUTOMATIC_STATUS=1
[[ "$(wc -l <"${TEST_ROOT}/flash.log")" == 1 ]] || \
  Fail "a programming failure incorrectly entered manual fallback"

: >"${TEST_ROOT}/flash.log"
ExpectFailure "manual fallback needs a terminal" Run \
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  FAKE_AUTOMATIC_STATUS=3
[[ "$(wc -l <"${TEST_ROOT}/flash.log")" == 1 ]] || \
  Fail "noninteractive activation failure retried programming"

command -v script >/dev/null 2>&1 || Fail "util-linux script is required"
: >"${TEST_ROOT}/flash.log"
rm -f "${TEST_ROOT}/boot.log"
printf '\n' | script -qfec \
  "env PATH='${TEST_ROOT}/bin:${PATH}' \
    FAKE_BOOT_CONTROL='${TEST_ROOT}/boot-control' \
    FAKE_FLASH_LOG='${TEST_ROOT}/flash.log' \
    FAKE_BOOT_LOG='${TEST_ROOT}/boot.log' \
    FAKE_AUTOMATIC_STATUS=3 \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    '${TEST_ROOT}/tools/guided_flash.sh' LOCKED /dev/null" \
  /dev/null >/dev/null
printf '%s\n' $'1\tLOCKED\t/dev/null' $'0\tLOCKED\t/dev/null' \
  >"${TEST_ROOT}/expected-flash.log"
cmp "${TEST_ROOT}/expected-flash.log" "${TEST_ROOT}/flash.log" || \
  Fail "manual fallback did not perform exactly one safe retry"
grep -Fqx application "${TEST_ROOT}/boot.log" || \
  Fail "verified manual fallback did not reset into the application"

: >"${TEST_ROOT}/flash.log"
rm -f "${TEST_ROOT}/boot.log"
if printf '\n' | script -qfec \
  "env PATH='${TEST_ROOT}/bin:${PATH}' \
    FAKE_BOOT_CONTROL='${TEST_ROOT}/boot-control' \
    FAKE_FLASH_LOG='${TEST_ROOT}/flash.log' \
    FAKE_BOOT_LOG='${TEST_ROOT}/boot.log' \
    FAKE_AUTOMATIC_STATUS=3 FAKE_MANUAL_STATUS=1 \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    '${TEST_ROOT}/tools/guided_flash.sh' LOCKED /dev/null" \
  /dev/null >/dev/null; then
  Fail "failed manual retry unexpectedly succeeded"
fi
[[ ! -e "${TEST_ROOT}/boot.log" ]] || \
  Fail "failed manual retry reset into the application"

ExpectFailure "another process owns" Run \
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  FAKE_FUSER_STATUS=0

echo "Guided automatic/manual flash tests passed."
