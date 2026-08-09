#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ARTIFACT_VERIFIER="${SCRIPT_DIR}/verify_firmware_artifact.sh"
readonly REQUIRED_UART_ACK="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"
readonly MODE="PID"
readonly TEMPORARY_PARENT="${TMPDIR:-/tmp}"
readonly AUTOMATIC_BOOT_CONTROL="${RRCLITE_AUTOMATIC_BOOT_CONTROL:-0}"
readonly PREFLIGHT_TIMEOUT_SEC="${RRCLITE_PROGRAMMER_PREFLIGHT_TIMEOUT_SEC:-15}"
readonly FLASH_TIMEOUT_SEC="${RRCLITE_PROGRAMMER_FLASH_TIMEOUT_SEC:-300}"

temporary_directory=""

Cleanup() {
  [[ -n "${temporary_directory}" ]] || return
  case "${temporary_directory}" in
    "${TEMPORARY_PARENT%/}"/rrclite-flash.*)
      rm -rf -- "${temporary_directory}"
      ;;
    *)
      echo "Refusing unsafe flash-snapshot cleanup: ${temporary_directory}" >&2
      ;;
  esac
}
trap Cleanup EXIT

Fail() {
  echo "RRCLite firmware flash failed: $*" >&2
  exit 1
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    Fail "neither sha256sum nor shasum is installed"
  fi
}

ReadMetadata() {
  local metadata="$1"
  local key="$2"
  local count
  local line
  count="$(grep -Ec "^${key}=" "${metadata}" || true)"
  [[ "${count}" == "1" ]] || \
    Fail "build metadata must contain exactly one ${key} entry"
  line="$(grep -E "^${key}=" "${metadata}")"
  printf '%s' "${line#*=}"
}

Usage() {
  cat <<'EOF'
Usage: ./tools/flash_firmware.sh /dev/SERIAL_PORT

Before flashing:
  1. Disconnect motor and servo power.
  2. Connect the USB-C port labelled UART1 / USB serial 1 (not 5V5A OUT).
  3. Enter the ROM bootloader automatically or with BOOT/RST.
  4. Set the exact acknowledgement only after confirming actuator power is off:

       RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED

The factory ROM bootloader uses 115200 baud, 8E1, and no flow control. The
running RRCLite application returns to its separately configured 1000000-baud
8N1 transport after BOOT is released and the board is reset.

Set RRCLITE_AUTOMATIC_BOOT_CONTROL=1 and RRCLITE_CH9102_BOOT_CONTROL to the
reviewed helper executable to enter and leave the bootloader automatically.
EOF
}

[[ "$#" -eq 1 ]] || {
  Usage >&2
  exit 2
}
readonly SERIAL_PORT="$1"

[[ "${AUTOMATIC_BOOT_CONTROL}" == "0" || \
  "${AUTOMATIC_BOOT_CONTROL}" == "1" ]] || \
  Fail "RRCLITE_AUTOMATIC_BOOT_CONTROL must be 0 or 1"
[[ "${PREFLIGHT_TIMEOUT_SEC}" =~ ^[1-9][0-9]{0,2}$ && \
  "${FLASH_TIMEOUT_SEC}" =~ ^[1-9][0-9]{0,3}$ ]] || \
  Fail "CubeProgrammer timeouts must be positive integer seconds"

[[ "${RRCLITE_UART_BOOTLOADER_ACK:-}" == "${REQUIRED_UART_ACK}" ]] || {
  Usage >&2
  Fail "set RRCLITE_UART_BOOTLOADER_ACK=${REQUIRED_UART_ACK} only after entering the ROM bootloader with all actuators disconnected"
}
[[ "${SERIAL_PORT}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && \
  "${SERIAL_PORT}" != *"/../"* && "${SERIAL_PORT}" != */.. && \
  "${SERIAL_PORT}" != *"/./"* && "${SERIAL_PORT}" != */. ]] || \
  Fail "serial port must be an explicit, well-formed /dev path"
[[ -c "${SERIAL_PORT}" ]] || \
  Fail "serial port does not resolve to an existing character device: ${SERIAL_PORT}"
[[ -r "${SERIAL_PORT}" && -w "${SERIAL_PORT}" ]] || \
  Fail "serial port is not readable and writable: ${SERIAL_PORT}"
[[ -x "${ARTIFACT_VERIFIER}" ]] || \
  Fail "firmware artifact verifier is missing or not executable"

readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly AUTHORITATIVE_ELF="${BUILD_ROOT}/mentor_pi_mcu.elf"
readonly METADATA="${BUILD_ROOT}/rrclite-build-metadata.txt"

if ! "${ARTIFACT_VERIFIER}" "${MODE}" "${PROJECT_ROOT}" >/dev/null; then
  Fail "firmware artifact verification failed; rebuild before flashing"
fi
[[ -s "${AUTHORITATIVE_ELF}" && ! -L "${AUTHORITATIVE_ELF}" ]] || \
  Fail "authoritative firmware ELF is missing, empty, or a symbolic link"
[[ -f "${METADATA}" && ! -L "${METADATA}" ]] || \
  Fail "firmware build metadata is missing or a symbolic link"

temporary_directory="$(mktemp -d \
  "${TEMPORARY_PARENT%/}/rrclite-flash.XXXXXX")"
readonly SNAPSHOT_DIRECTORY="${temporary_directory}"
readonly SNAPSHOT_METADATA="${SNAPSHOT_DIRECTORY}/rrclite-build-metadata.txt"
readonly SNAPSHOT_TEMP="${SNAPSHOT_DIRECTORY}/mentor_pi_mcu.elf.tmp"

cp "${METADATA}" "${SNAPSHOT_METADATA}"
cp "${AUTHORITATIVE_ELF}" "${SNAPSHOT_TEMP}"

readonly SNAPSHOT_MODE="$(ReadMetadata "${SNAPSHOT_METADATA}" motor_mode)"
readonly SNAPSHOT_ARTIFACT_MODE="$(
  ReadMetadata "${SNAPSHOT_METADATA}" artifact_mode
)"
[[ "${SNAPSHOT_MODE}" == "${MODE}" && \
  "${SNAPSHOT_ARTIFACT_MODE}" == "NORMAL" ]] || \
  Fail "firmware metadata mode changed while the flash snapshot was prepared"
readonly EXPECTED_ELF_SHA256="$(
  ReadMetadata "${SNAPSHOT_METADATA}" elf_sha256
)"
[[ "${EXPECTED_ELF_SHA256}" =~ ^[0-9a-f]{64}$ ]] || \
  Fail "firmware metadata contains a malformed ELF SHA-256"
[[ "$(Sha256 "${SNAPSHOT_TEMP}")" == "${EXPECTED_ELF_SHA256}" ]] || \
  Fail "firmware ELF changed while the flash snapshot was copied"

if ! "${ARTIFACT_VERIFIER}" "${MODE}" "${PROJECT_ROOT}" >/dev/null; then
  Fail "firmware changed while the flash snapshot was prepared"
fi
cmp "${SNAPSHOT_METADATA}" "${METADATA}" >/dev/null || \
  Fail "firmware metadata changed while the flash snapshot was prepared"
[[ "$(Sha256 "${AUTHORITATIVE_ELF}")" == "${EXPECTED_ELF_SHA256}" ]] || \
  Fail "authoritative firmware ELF changed while the flash snapshot was prepared"

readonly SNAPSHOT="${SNAPSHOT_DIRECTORY}/mentor_pi_mcu-${EXPECTED_ELF_SHA256}.elf"
mv "${SNAPSHOT_TEMP}" "${SNAPSHOT}"
chmod 0444 "${SNAPSHOT}"
[[ "$(Sha256 "${SNAPSHOT}")" == "${EXPECTED_ELF_SHA256}" ]] || \
  Fail "verified firmware snapshot hash mismatch"

programmer="${STM32_CUBE_PROGRAMMER_CLI:-}"
if [[ -n "${programmer}" ]]; then
  [[ -x "${programmer}" ]] || \
    Fail "STM32_CUBE_PROGRAMMER_CLI is not executable: ${programmer}"
elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  programmer="$(command -v STM32_Programmer_CLI)"
else
  readonly -a PROGRAMMER_CANDIDATES=(
    "${HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
    "/opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI"
  )
  for candidate in "${PROGRAMMER_CANDIDATES[@]}"; do
    if [[ -x "${candidate}" ]]; then
      programmer="${candidate}"
      break
    fi
  done
  if [[ -z "${programmer}" ]]; then
    Fail "STM32_Programmer_CLI was not found; install STM32CubeProgrammer or set STM32_CUBE_PROGRAMMER_CLI to its executable"
  fi
fi
readonly programmer

boot_control="${RRCLITE_CH9102_BOOT_CONTROL:-}"
if [[ "${AUTOMATIC_BOOT_CONTROL}" == "1" ]]; then
  [[ -x "${boot_control}" ]] || \
    Fail "automatic boot control helper is missing or not executable"
  echo "Entering the STM32 ROM bootloader through CH9102F DTR/RTS."
  if ! "${boot_control}" --device "${SERIAL_PORT}" --mode bootloader; then
    Fail "automatic CH9102F ROM-bootloader entry failed"
  fi
  echo "Probing the STM32 ROM bootloader before programming."
  if ! timeout --foreground "${PREFLIGHT_TIMEOUT_SEC}" "${programmer}" \
    -c "port=${SERIAL_PORT}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
    rts=low dtr=low; then
    echo "RRCLite automatic bootloader activation failed before programming." >&2
    exit 3
  fi
fi

echo "Programming verified ${MODE} firmware over ${SERIAL_PORT}."
echo "Verified ELF SHA-256: ${EXPECTED_ELF_SHA256}"
echo "CubeProgrammer UART settings: 115200 baud, 8E1, flow control off, DTR/RTS low."
if ! timeout --foreground "${FLASH_TIMEOUT_SEC}" "${programmer}" \
  -c "port=${SERIAL_PORT}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
  rts=low dtr=low \
  -w "${SNAPSHOT}" -v; then
  Fail "CubeProgrammer programming or read-back verification failed"
fi

echo "Firmware programming and read-back verification succeeded."
if [[ "${AUTOMATIC_BOOT_CONTROL}" == "1" ]]; then
  echo "Resetting into the verified application through CH9102F DTR/RTS."
  if ! "${boot_control}" --device "${SERIAL_PORT}" --mode application; then
    Fail "firmware is verified, but the automatic normal-boot reset failed"
  fi
  echo "Verified application reset completed; no BOOT or RST press is needed."
else
  echo "Release BOOT and briefly press RST to run the verified image."
fi
