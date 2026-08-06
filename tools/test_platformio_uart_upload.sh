#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly UPLOADER="${SCRIPT_DIR}/platformio_uart_upload.sh"
readonly TEST_ROOT="$(mktemp -d)"

Cleanup() {
  [[ -n "${TEST_ROOT}" && -d "${TEST_ROOT}" ]] || return
  chmod -R u+rwX "${TEST_ROOT}"
  rm -rf "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "PlatformIO UART upload test failed: $*" >&2
  exit 1
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

readonly LOG="${TEST_ROOT}/arguments.txt"
readonly FAKE_PROGRAMMER="${TEST_ROOT}/STM32_Programmer_CLI"
readonly SNAPSHOT_DIRECTORY="${TEST_ROOT}/verified-artifacts"
mkdir -p "${SNAPSHOT_DIRECTORY}"
readonly FIRMWARE_CONTENT="${TEST_ROOT}/firmware-content"
printf 'verified elf\n' >"${FIRMWARE_CONTENT}"
if command -v sha256sum >/dev/null 2>&1; then
  firmware_sha256="$(sha256sum "${FIRMWARE_CONTENT}" | awk '{print $1}')"
else
  firmware_sha256="$(shasum -a 256 "${FIRMWARE_CONTENT}" | awk '{print $1}')"
fi
readonly firmware_sha256
readonly FIRMWARE="${SNAPSHOT_DIRECTORY}/mentor_pi_mcu-${firmware_sha256}.elf"
cp "${FIRMWARE_CONTENT}" "${FIRMWARE}"

apply_fake_programmer() {
  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    ': "${FAKE_PROGRAMMER_LOG:?}"' \
    'printf "%s\n" "$@" >"${FAKE_PROGRAMMER_LOG}"' \
    'exit "${FAKE_PROGRAMMER_EXIT_CODE:-0}"' \
    >"${FAKE_PROGRAMMER}"
  chmod +x "${FAKE_PROGRAMMER}"
}
apply_fake_programmer

ExpectFailure "RRCLITE_UART_BOOTLOADER_ACK=" \
  env STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" /dev/cu.rrclite-test "${FIRMWARE}"

ExpectFailure "upload port is empty or malformed" \
  env RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" --wrong "${FIRMWARE}"

readonly WRONG_EXTENSION="${TEST_ROOT}/unverified.hex"
printf 'hex\n' >"${WRONG_EXTENSION}"
ExpectFailure "hash-named verified ELF snapshot" \
  env RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" /dev/cu.rrclite-test "${WRONG_EXTENSION}"

printf 'changed\n' >>"${FIRMWARE}"
ExpectFailure "changed after preparation" \
  env RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" /dev/cu.rrclite-test "${FIRMWARE}"
cp "${FIRMWARE_CONTENT}" "${FIRMWARE}"

ExpectFailure "programming or read-back verification failed" \
  env FAKE_PROGRAMMER_LOG="${LOG}" FAKE_PROGRAMMER_EXIT_CODE=17 \
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" /dev/cu.rrclite-test "${FIRMWARE}"

FAKE_PROGRAMMER_LOG="${LOG}" \
RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
STM32_CUBE_PROGRAMMER_CLI="${FAKE_PROGRAMMER}" \
  "${UPLOADER}" /dev/cu.rrclite-test "${FIRMWARE}" >/dev/null

readonly EXPECTED_ARGUMENTS="${TEST_ROOT}/expected-arguments.txt"
printf '%s\n' \
  -c \
  port=/dev/cu.rrclite-test \
  br=115200 \
  -w \
  "${FIRMWARE}" \
  -v \
  >"${EXPECTED_ARGUMENTS}"
cmp "${EXPECTED_ARGUMENTS}" "${LOG}" || \
  Fail "CubeProgrammer arguments differ from the reviewed UART command"

echo "PlatformIO UART upload helper tests passed."
