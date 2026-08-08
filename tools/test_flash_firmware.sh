#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FLASH_SOURCE="${SCRIPT_DIR}/flash_firmware.sh"
readonly TEST_ROOT="$(mktemp -d)"

Cleanup() {
  [[ -n "${TEST_ROOT}" && -d "${TEST_ROOT}" ]] || return
  chmod -R u+rwX "${TEST_ROOT}"
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Direct firmware flash test failed: $*" >&2
  exit 1
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

ExpectFailure() {
  local expected_text="$1"
  shift
  local output
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected_text}"* ]] || \
    Fail "failure did not contain '${expected_text}': ${output}"
}

CreateFixture() {
  local name="$1"
  local mode="${2:-LOCKED}"
  local motor_mode="${mode}"
  local artifact_mode="NORMAL"
  local root="${TEST_ROOT}/${name}"
  local build_root="${root}/firmware/mentor_pi_mcu/build/stm32"
  mkdir -p "${root}/tools" "${build_root}" "${root}/tmp"
  cp "${FLASH_SOURCE}" "${root}/tools/flash_firmware.sh"
  chmod +x "${root}/tools/flash_firmware.sh"

  printf 'verified %s firmware\n' "${mode}" \
    >"${build_root}/mentor_pi_mcu.elf"
  local digest
  digest="$(Sha256 "${build_root}/mentor_pi_mcu.elf")"
  printf '%s\n' \
    "motor_mode=${motor_mode}" \
    "artifact_mode=${artifact_mode}" \
    "elf_sha256=${digest}" \
    >"${build_root}/rrclite-build-metadata.txt"

  # The artifact verifier has its own exhaustive fixture tests. This fake
  # proves that the flash wrapper invokes the fixed project-owned verifier
  # twice with the requested mode and project root, and aborts on either
  # failure without coupling these tests to a particular metadata schema.
  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    ': "${FAKE_VERIFIER_LOG:?}"' \
    ': "${FAKE_VERIFIER_COUNT:?}"' \
    'count=0' \
    '[[ ! -f "${FAKE_VERIFIER_COUNT}" ]] || count="$(cat "${FAKE_VERIFIER_COUNT}")"' \
    'count=$((count + 1))' \
    'printf "%s" "${count}" >"${FAKE_VERIFIER_COUNT}"' \
    'printf "%s\t%s\n" "$1" "$2" >>"${FAKE_VERIFIER_LOG}"' \
    'if [[ "${FAKE_VERIFIER_MUTATE_METADATA_CALL:-}" == "${count}" ]]; then' \
    '  printf "race=yes\n" >>"$2/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"' \
    'fi' \
    'if [[ "${FAKE_VERIFIER_FAIL_CALL:-}" == "${count}" ]]; then' \
    '  echo "deliberate verifier failure" >&2' \
    '  exit 23' \
    'fi' \
    >"${root}/tools/verify_firmware_artifact.sh"
  chmod +x "${root}/tools/verify_firmware_artifact.sh"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    ': "${FAKE_PROGRAMMER_LOG:?}"' \
    ': "${FAKE_PROGRAMMER_SNAPSHOT_LOG:?}"' \
    'printf "%s\n" "$@" >"${FAKE_PROGRAMMER_LOG}"' \
    'snapshot=""' \
    'while (($# > 0)); do' \
    '  if [[ "$1" == "-w" && $# -ge 2 ]]; then' \
    '    snapshot="$2"' \
    '    break' \
    '  fi' \
    '  shift' \
    'done' \
    'if [[ -z "${snapshot}" ]]; then' \
    '  sleep "${FAKE_PROGRAMMER_PREFLIGHT_DELAY_SEC:-0}"' \
    '  exit "${FAKE_PROGRAMMER_PREFLIGHT_EXIT_CODE:-0}"' \
    'fi' \
    'sleep "${FAKE_PROGRAMMER_FLASH_DELAY_SEC:-0}"' \
    '[[ -n "${snapshot}" && -f "${snapshot}" && ! -L "${snapshot}" ]]' \
    'if command -v sha256sum >/dev/null 2>&1; then' \
    '  read -r snapshot_hash _ < <(sha256sum "${snapshot}")' \
    'else' \
    '  read -r snapshot_hash _ < <(shasum -a 256 "${snapshot}")' \
    'fi' \
    'if stat -c %a "${snapshot}" >/dev/null 2>&1; then' \
    '  snapshot_mode="$(stat -c %a "${snapshot}")"' \
    'else' \
    '  snapshot_mode="$(stat -f %Lp "${snapshot}")"' \
    'fi' \
    'printf "path=%s\nsha256=%s\nmode=%s\n" "${snapshot}" "${snapshot_hash}" "${snapshot_mode}" >"${FAKE_PROGRAMMER_SNAPSHOT_LOG}"' \
    'exit "${FAKE_PROGRAMMER_EXIT_CODE:-0}"' \
    >"${root}/STM32_Programmer_CLI"
  chmod +x "${root}/STM32_Programmer_CLI"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    ': "${FAKE_BOOT_CONTROL_LOG:?}"' \
    '[[ "$#" == 4 && "$1" == "--device" && "$3" == "--mode" ]]' \
    'printf "%s\n" "$4" >>"${FAKE_BOOT_CONTROL_LOG}"' \
    'if [[ "${FAKE_BOOT_CONTROL_FAIL_MODE:-}" == "$4" ]]; then exit 19; fi' \
    >"${root}/ch9102_boot_control"
  chmod +x "${root}/ch9102_boot_control"

  printf '%s' "${root}"
}

RunFlash() {
  local root="$1"
  local mode="$2"
  local port="$3"
  shift 3
  env \
    FAKE_PROGRAMMER_LOG="${root}/programmer.log" \
    FAKE_PROGRAMMER_SNAPSHOT_LOG="${root}/programmer-snapshot.log" \
    FAKE_BOOT_CONTROL_LOG="${root}/boot-control.log" \
    FAKE_VERIFIER_LOG="${root}/verifier.log" \
    FAKE_VERIFIER_COUNT="${root}/verifier.count" \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    STM32_CUBE_PROGRAMMER_CLI="${root}/STM32_Programmer_CLI" \
    TMPDIR="${root}/tmp" \
    "$@" \
    "${root}/tools/flash_firmware.sh" "${mode}" "${port}"
}

usage_root="$(CreateFixture usage)"
ExpectFailure "Usage:" "${usage_root}/tools/flash_firmware.sh"
ExpectFailure "RRCLITE_UART_BOOTLOADER_ACK=" \
  env FAKE_PROGRAMMER_LOG="${usage_root}/programmer.log" \
    FAKE_VERIFIER_LOG="${usage_root}/verifier.log" \
    FAKE_VERIFIER_COUNT="${usage_root}/verifier.count" \
    STM32_CUBE_PROGRAMMER_CLI="${usage_root}/STM32_Programmer_CLI" \
    "${usage_root}/tools/flash_firmware.sh" LOCKED /dev/ttyUSB0
ExpectFailure "well-formed /dev path" \
  RunFlash "${usage_root}" LOCKED 'port=attacker'
ExpectFailure "well-formed /dev path" \
  RunFlash "${usage_root}" LOCKED /dev/../tmp/ttyUSB0
ExpectFailure "existing character device" \
  RunFlash "${usage_root}" LOCKED /dev/rrclite-port-that-does-not-exist

commissioning_root="$(CreateFixture commissioning COMMISSIONING)"
ExpectFailure "RRCLITE_COMMISSIONING_FLASH_ACK=" \
  RunFlash "${commissioning_root}" COMMISSIONING /dev/null

pid_root="$(CreateFixture commissioning-pid COMMISSIONING_PID)"
ExpectFailure "RRCLITE_COMMISSIONING_FLASH_ACK=" \
  RunFlash "${pid_root}" COMMISSIONING_PID /dev/null
RunFlash "${pid_root}" COMMISSIONING_PID /dev/null \
  RRCLITE_COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED \
  >/dev/null
printf '%s\t%s\n' \
  COMMISSIONING_PID "${pid_root}" \
  COMMISSIONING_PID "${pid_root}" \
  >"${pid_root}/expected-verifier.log"
cmp "${pid_root}/expected-verifier.log" "${pid_root}/verifier.log" || \
  Fail "PID flash did not verify the exact COMMISSIONING_PID artifact twice"

wrong_mode_root="$(CreateFixture wrong-mode LOCKED)"
ExpectFailure "metadata mode changed" \
  RunFlash "${wrong_mode_root}" COMMISSIONING /dev/null \
    RRCLITE_COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED

stale_root="$(CreateFixture stale)"
printf 'changed\n' \
  >>"${stale_root}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
ExpectFailure "ELF changed while the flash snapshot was copied" \
  RunFlash "${stale_root}" LOCKED /dev/null
[[ ! -e "${stale_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran for a stale firmware ELF"

first_verifier_root="$(CreateFixture first-verifier)"
ExpectFailure "artifact verification failed" \
  RunFlash "${first_verifier_root}" LOCKED /dev/null \
    FAKE_VERIFIER_FAIL_CALL=1
[[ ! -e "${first_verifier_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after initial artifact verification failed"

second_verifier_root="$(CreateFixture second-verifier)"
ExpectFailure "firmware changed while the flash snapshot was prepared" \
  RunFlash "${second_verifier_root}" LOCKED /dev/null \
    FAKE_VERIFIER_FAIL_CALL=2
[[ ! -e "${second_verifier_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after post-snapshot verification failed"

metadata_race_root="$(CreateFixture metadata-race)"
ExpectFailure "metadata changed while the flash snapshot was prepared" \
  RunFlash "${metadata_race_root}" LOCKED /dev/null \
    FAKE_VERIFIER_MUTATE_METADATA_CALL=2
[[ ! -e "${metadata_race_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after concurrent metadata replacement"

programmer_failure_root="$(CreateFixture programmer-failure)"
ExpectFailure "programming or read-back verification failed" \
  RunFlash "${programmer_failure_root}" LOCKED /dev/null \
    FAKE_PROGRAMMER_EXIT_CODE=17
failed_snapshot="$(sed -n 's/^path=//p' \
  "${programmer_failure_root}/programmer-snapshot.log")"
[[ -n "${failed_snapshot}" && ! -e "${failed_snapshot}" ]] || \
  Fail "the temporary snapshot survived a CubeProgrammer failure"

success_root="$(CreateFixture success)"
RunFlash "${success_root}" LOCKED /dev/null >/dev/null
success_build="${success_root}/firmware/mentor_pi_mcu/build/stm32"
success_hash="$(Sha256 "${success_build}/mentor_pi_mcu.elf")"
success_snapshot="$(sed -n 's/^path=//p' \
  "${success_root}/programmer-snapshot.log")"
[[ "${success_snapshot}" == \
  "${success_root}/tmp/rrclite-flash."*/"mentor_pi_mcu-${success_hash}.elf" ]] || \
  Fail "CubeProgrammer did not receive the expected hash-named snapshot"
grep -Fqx "sha256=${success_hash}" \
  "${success_root}/programmer-snapshot.log" || \
  Fail "the snapshot hash changed before CubeProgrammer ran"
grep -Fqx 'mode=444' "${success_root}/programmer-snapshot.log" || \
  Fail "the verified snapshot was not read-only"
[[ ! -e "${success_snapshot}" && ! -e "$(dirname "${success_snapshot}")" ]] || \
  Fail "the temporary snapshot survived successful programming"
[[ "$(cat "${success_root}/verifier.count")" == "2" ]] || \
  Fail "the artifact verifier did not run before and after snapshotting"
readonly EXPECTED_VERIFIER_LOG="${success_root}/expected-verifier.log"
printf '%s\t%s\n' \
  LOCKED "${success_root}" \
  LOCKED "${success_root}" \
  >"${EXPECTED_VERIFIER_LOG}"
cmp "${EXPECTED_VERIFIER_LOG}" "${success_root}/verifier.log" || \
  Fail "artifact verifier arguments differ from the reviewed contract"
readonly EXPECTED_PROGRAMMER_LOG="${success_root}/expected-programmer.log"
printf '%s\n' \
  -c \
  port=/dev/null \
  br=115200 \
  P=EVEN \
  db=8 \
  sb=1 \
  fc=OFF \
  rts=low \
  dtr=low \
  -w \
  "${success_snapshot}" \
  -v \
  >"${EXPECTED_PROGRAMMER_LOG}"
cmp "${EXPECTED_PROGRAMMER_LOG}" "${success_root}/programmer.log" || \
  Fail "CubeProgrammer arguments differ from the reviewed UART command"

automatic_root="$(CreateFixture automatic)"
RunFlash "${automatic_root}" LOCKED /dev/null \
  RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
  RRCLITE_CH9102_BOOT_CONTROL="${automatic_root}/ch9102_boot_control" \
  >/dev/null
printf '%s\n' bootloader application >"${automatic_root}/expected-boot.log"
cmp "${automatic_root}/expected-boot.log" \
  "${automatic_root}/boot-control.log" || \
  Fail "automatic flash did not enter bootloader then reset the application"

activation_failure_root="$(CreateFixture activation-failure)"
ExpectFailure "automatic bootloader activation failed" \
  RunFlash "${activation_failure_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${activation_failure_root}/ch9102_boot_control" \
    FAKE_PROGRAMMER_PREFLIGHT_EXIT_CODE=18
grep -Fqx bootloader "${activation_failure_root}/boot-control.log" || \
  Fail "activation failure did not preserve bootloader-only state"
[[ "$(wc -l <"${activation_failure_root}/boot-control.log")" == "1" ]] || \
  Fail "activation failure reset into the application"

automatic_program_failure_root="$(CreateFixture automatic-program-failure)"
ExpectFailure "programming or read-back verification failed" \
  RunFlash "${automatic_program_failure_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${automatic_program_failure_root}/ch9102_boot_control" \
    FAKE_PROGRAMMER_EXIT_CODE=17
grep -Fqx bootloader \
  "${automatic_program_failure_root}/boot-control.log" || \
  Fail "program failure did not preserve bootloader-only state"
[[ "$(wc -l <"${automatic_program_failure_root}/boot-control.log")" == "1" ]] || \
  Fail "program failure reset into the application"

automatic_interrupt_root="$(CreateFixture automatic-interrupt)"
ExpectFailure "programming or read-back verification failed" \
  RunFlash "${automatic_interrupt_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${automatic_interrupt_root}/ch9102_boot_control" \
    FAKE_PROGRAMMER_EXIT_CODE=130
[[ "$(cat "${automatic_interrupt_root}/boot-control.log")" == bootloader ]] || \
  Fail "interrupted programming reset into the application"

preflight_timeout_root="$(CreateFixture preflight-timeout)"
ExpectFailure "automatic bootloader activation failed" \
  RunFlash "${preflight_timeout_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${preflight_timeout_root}/ch9102_boot_control" \
    RRCLITE_PROGRAMMER_PREFLIGHT_TIMEOUT_SEC=1 \
    FAKE_PROGRAMMER_PREFLIGHT_DELAY_SEC=2
[[ "$(cat "${preflight_timeout_root}/boot-control.log")" == bootloader ]] || \
  Fail "preflight timeout reset into the application"

flash_timeout_root="$(CreateFixture flash-timeout)"
ExpectFailure "programming or read-back verification failed" \
  RunFlash "${flash_timeout_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${flash_timeout_root}/ch9102_boot_control" \
    RRCLITE_PROGRAMMER_FLASH_TIMEOUT_SEC=1 \
    FAKE_PROGRAMMER_FLASH_DELAY_SEC=2
[[ "$(cat "${flash_timeout_root}/boot-control.log")" == bootloader ]] || \
  Fail "programming timeout reset into the application"

application_reset_failure_root="$(CreateFixture application-reset-failure)"
ExpectFailure "automatic normal-boot reset failed" \
  RunFlash "${application_reset_failure_root}" LOCKED /dev/null \
    RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
    RRCLITE_CH9102_BOOT_CONTROL="${application_reset_failure_root}/ch9102_boot_control" \
    FAKE_BOOT_CONTROL_FAIL_MODE=application
printf '%s\n' bootloader application \
  >"${application_reset_failure_root}/expected-boot.log"
cmp "${application_reset_failure_root}/expected-boot.log" \
  "${application_reset_failure_root}/boot-control.log" || \
  Fail "application reset failure sequence differs"

printf 'later authoritative replacement\n' \
  >"${success_build}/mentor_pi_mcu.elf"
grep -Fqx "sha256=${success_hash}" \
  "${success_root}/programmer-snapshot.log" || \
  Fail "the recorded flashed hash changed with the authoritative source"

echo "Direct CubeProgrammer firmware flash tests passed."
