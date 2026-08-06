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

FileMode() {
  if stat -c '%a' "$1" >/dev/null 2>&1; then
    stat -c '%a' "$1"
  else
    stat -f '%Lp' "$1"
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
  local root="${TEST_ROOT}/${name}"
  local build_root="${root}/firmware/mentor_pi_mcu/build/stm32"
  mkdir -p "${root}/tools" "${build_root}"
  cp "${FLASH_SOURCE}" "${root}/tools/flash_firmware.sh"
  chmod +x "${root}/tools/flash_firmware.sh"

  printf 'verified %s firmware\n' "${mode}" \
    >"${build_root}/mentor_pi_mcu.elf"
  local digest
  digest="$(Sha256 "${build_root}/mentor_pi_mcu.elf")"
  printf '%s\n' \
    "motor_mode=${mode}" \
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
    'printf "%s\n" "$@" >"${FAKE_PROGRAMMER_LOG}"' \
    'exit "${FAKE_PROGRAMMER_EXIT_CODE:-0}"' \
    >"${root}/STM32_Programmer_CLI"
  chmod +x "${root}/STM32_Programmer_CLI"

  printf '%s' "${root}"
}

RunFlash() {
  local root="$1"
  local mode="$2"
  local port="$3"
  shift 3
  env \
    FAKE_PROGRAMMER_LOG="${root}/programmer.log" \
    FAKE_VERIFIER_LOG="${root}/verifier.log" \
    FAKE_VERIFIER_COUNT="${root}/verifier.count" \
    RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    STM32_CUBE_PROGRAMMER_CLI="${root}/STM32_Programmer_CLI" \
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

commissioning_root="$(CreateFixture commissioning COMMISSIONING)"
ExpectFailure "RRCLITE_COMMISSIONING_FLASH_ACK=" \
  RunFlash "${commissioning_root}" COMMISSIONING /dev/ttyUSB0

wrong_mode_root="$(CreateFixture wrong-mode LOCKED)"
ExpectFailure "metadata mode changed" \
  RunFlash "${wrong_mode_root}" COMMISSIONING /dev/ttyUSB0 \
    RRCLITE_COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED

stale_root="$(CreateFixture stale)"
printf 'changed\n' \
  >>"${stale_root}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
ExpectFailure "ELF changed while the flash snapshot was copied" \
  RunFlash "${stale_root}" LOCKED /dev/ttyUSB0
[[ ! -e "${stale_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran for a stale firmware ELF"

first_verifier_root="$(CreateFixture first-verifier)"
ExpectFailure "artifact verification failed" \
  RunFlash "${first_verifier_root}" LOCKED /dev/ttyUSB0 \
    FAKE_VERIFIER_FAIL_CALL=1
[[ ! -e "${first_verifier_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after initial artifact verification failed"

second_verifier_root="$(CreateFixture second-verifier)"
ExpectFailure "firmware changed while the flash snapshot was prepared" \
  RunFlash "${second_verifier_root}" LOCKED /dev/ttyUSB0 \
    FAKE_VERIFIER_FAIL_CALL=2
[[ ! -e "${second_verifier_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after post-snapshot verification failed"

metadata_race_root="$(CreateFixture metadata-race)"
ExpectFailure "metadata changed while the flash snapshot was prepared" \
  RunFlash "${metadata_race_root}" LOCKED /dev/ttyUSB0 \
    FAKE_VERIFIER_MUTATE_METADATA_CALL=2
[[ ! -e "${metadata_race_root}/programmer.log" ]] || \
  Fail "CubeProgrammer ran after concurrent metadata replacement"

conflict_root="$(CreateFixture conflicting-snapshot)"
conflict_build="${conflict_root}/firmware/mentor_pi_mcu/build/stm32"
conflict_hash="$(Sha256 "${conflict_build}/mentor_pi_mcu.elf")"
mkdir -p "${conflict_build}/verified-artifacts"
printf 'conflicting bytes\n' \
  >"${conflict_build}/verified-artifacts/mentor_pi_mcu-${conflict_hash}.elf"
ExpectFailure "existing verified firmware snapshot hash mismatch" \
  RunFlash "${conflict_root}" LOCKED /dev/ttyUSB0
[[ "$(cat "${conflict_build}/verified-artifacts/mentor_pi_mcu-${conflict_hash}.elf")" == \
  "conflicting bytes" ]] || Fail "a conflicting snapshot was overwritten"

programmer_failure_root="$(CreateFixture programmer-failure)"
ExpectFailure "programming or read-back verification failed" \
  RunFlash "${programmer_failure_root}" LOCKED /dev/ttyUSB0 \
    FAKE_PROGRAMMER_EXIT_CODE=17

success_root="$(CreateFixture success)"
RunFlash "${success_root}" LOCKED /dev/serial/by-id/rrclite-test >/dev/null
success_build="${success_root}/firmware/mentor_pi_mcu/build/stm32"
success_hash="$(Sha256 "${success_build}/mentor_pi_mcu.elf")"
success_snapshot="${success_build}/verified-artifacts/mentor_pi_mcu-${success_hash}.elf"
[[ -f "${success_snapshot}" && ! -L "${success_snapshot}" ]] || \
  Fail "the digest-named verified snapshot was not published"
[[ "$(FileMode "${success_snapshot}")" == "444" ]] || \
  Fail "the verified snapshot is not read-only"
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
  port=/dev/serial/by-id/rrclite-test \
  br=115200 \
  -w \
  "${success_snapshot}" \
  -v \
  >"${EXPECTED_PROGRAMMER_LOG}"
cmp "${EXPECTED_PROGRAMMER_LOG}" "${success_root}/programmer.log" || \
  Fail "CubeProgrammer arguments differ from the reviewed UART command"

original_snapshot_hash="$(Sha256 "${success_snapshot}")"
printf 'later authoritative replacement\n' \
  >"${success_build}/mentor_pi_mcu.elf"
[[ "$(Sha256 "${success_snapshot}")" == "${original_snapshot_hash}" ]] || \
  Fail "the verified snapshot changed with the authoritative source"

echo "Direct CubeProgrammer firmware flash tests passed."
