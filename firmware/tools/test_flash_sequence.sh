#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FLASH_SOURCE="${SCRIPT_DIR}/flash.sh"
readonly SHA_SOURCE="${SCRIPT_DIR}/sha256.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "Firmware automatic-flash test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output=""
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "failure did not contain '${expected}': ${output}"
}

CreateFixture() {
  local name="$1"
  local root="${TEST_ROOT}/${name}"
  local tools="${root}/firmware/tools"
  local build="${root}/firmware/mentor_pi_mcu/build/stm32"
  mkdir -p "${tools}" "${build}" "${root}/bin" "${root}/tmp"
  cp "${FLASH_SOURCE}" "${tools}/flash.sh"
  cp "${SHA_SOURCE}" "${tools}/sha256.sh"
  chmod 0755 "${tools}/flash.sh" "${tools}/sha256.sh"
  printf '%s\n' 'verified ADRC firmware fixture' \
    >"${build}/mentor_pi_mcu.elf"
  digest="$("${tools}/sha256.sh" "${build}/mentor_pi_mcu.elf")"
  printf 'elf_sha256=%s\n' "${digest}" \
    >"${build}/rrclite-build-metadata.txt"

  cat >"${root}/bin/fuser" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
  cat >"${root}/bin/lsof" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
  cat >"${root}/STM32_Programmer_CLI" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
: "${FAKE_PROGRAMMER_LOG:?}"
mode=preflight
for argument in "$@"; do
  [[ "${argument}" != -w ]] || mode=flash
done
printf '%s\n' "${mode}" >>"${FAKE_PROGRAMMER_LOG}"
if [[ "${mode}" == preflight ]]; then
  exit "${FAKE_PREFLIGHT_STATUS:-0}"
fi
exit "${FAKE_FLASH_STATUS:-0}"
EOF
  cat >"${root}/ch9102_boot_control" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
: "${FAKE_BOOT_LOG:?}"
[[ "$#" == 4 && "$1" == --device && "$3" == --mode ]]
printf '%s\n' "$4" >>"${FAKE_BOOT_LOG}"
[[ "${FAKE_BOOT_FAILURE_MODE:-}" != "$4" ]]
EOF
  chmod 0755 "${root}/bin/fuser" "${root}/bin/lsof" \
    "${root}/STM32_Programmer_CLI" "${root}/ch9102_boot_control"
  printf '%s' "${root}"
}

RunFlash() {
  local root="$1"
  shift
  env \
    PATH="${root}/bin:/usr/bin:/bin" \
    FAKE_PROGRAMMER_LOG="${root}/programmer.log" \
    FAKE_BOOT_LOG="${root}/boot.log" \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    RRCLITE_CH9102_BOOT_CONTROL="${root}/ch9102_boot_control" \
    STM32_CUBE_PROGRAMMER_CLI="${root}/STM32_Programmer_CLI" \
    TMPDIR="${root}/tmp" \
    "$@" "${root}/firmware/tools/flash.sh" /dev/null
}

if [[ -f /.dockerenv ]]; then
  echo "Firmware automatic-flash orchestration test skipped in the Dev Container."
  exit 0
fi

ack_root="$(CreateFixture acknowledgement)"
ExpectFailure 'set FLASH_ACK=' env \
  PATH="${ack_root}/bin:/usr/bin:/bin" \
  STM32_CUBE_PROGRAMMER_CLI="${ack_root}/STM32_Programmer_CLI" \
  "${ack_root}/firmware/tools/flash.sh" /dev/null
[[ ! -e "${ack_root}/boot.log" && ! -e "${ack_root}/programmer.log" ]] || \
  Fail "a command ran before the safety acknowledgement"

success_root="$(CreateFixture success)"
RunFlash "${success_root}" >/dev/null
printf '%s\n' bootloader application >"${success_root}/expected-boot.log"
printf '%s\n' preflight flash >"${success_root}/expected-programmer.log"
cmp "${success_root}/expected-boot.log" "${success_root}/boot.log" || \
  Fail "successful automatic boot sequence differs"
cmp "${success_root}/expected-programmer.log" \
  "${success_root}/programmer.log" || \
  Fail "successful programmer sequence differs"

preflight_root="$(CreateFixture preflight-failure)"
ExpectFailure 'MCU remains in bootloader mode' RunFlash "${preflight_root}" \
  FAKE_PREFLIGHT_STATUS=17
[[ "$(cat "${preflight_root}/boot.log")" == bootloader ]] || \
  Fail "preflight failure reset the application"
[[ "$(cat "${preflight_root}/programmer.log")" == preflight ]] || \
  Fail "programming ran after preflight failure"

flash_root="$(CreateFixture flash-failure)"
ExpectFailure 'programming or read-back verification failed' \
  RunFlash "${flash_root}" FAKE_FLASH_STATUS=18
[[ "$(cat "${flash_root}/boot.log")" == bootloader ]] || \
  Fail "program failure reset the application"

reset_root="$(CreateFixture reset-failure)"
ExpectFailure 'automatic normal-boot reset failed' RunFlash "${reset_root}" \
  FAKE_BOOT_FAILURE_MODE=application
printf '%s\n' bootloader application >"${reset_root}/expected-boot.log"
cmp "${reset_root}/expected-boot.log" "${reset_root}/boot.log" || \
  Fail "normal-reset failure sequence differs"

manual_root="$(CreateFixture manual)"
RunFlash "${manual_root}" RRCLITE_AUTOMATIC_BOOT_CONTROL=0 >/dev/null
[[ ! -e "${manual_root}/boot.log" ]] || \
  Fail "manual fallback invoked modem control"
[[ "$(cat "${manual_root}/programmer.log")" == flash ]] || \
  Fail "manual fallback did not run exactly one programming command"

echo "Firmware automatic CH9102F flash orchestration tests passed."
