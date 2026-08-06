#!/usr/bin/env bash

set -euo pipefail

readonly REQUIRED_ACKNOWLEDGEMENT="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"

Fail() {
  echo "RRCLite UART upload failed: $*" >&2
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

Usage() {
  cat <<'EOF'
This internal helper is supported only when invoked by PlatformIO after it has
verified and snapshotted the authoritative firmware ELF. Do not run it
directly.

Before upload:
  1. Disconnect motor and servo power.
  2. Connect the USB-C port labelled UART1 / USB serial 1 (not 5V5A OUT).
  3. Hold BOOT, press and release RST, then release BOOT.
  4. Set RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED.
  5. Run PlatformIO with --upload-port set to the exact CH9102F device.

Example:
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    pio run -e rrclite_uart -t upload \
      --upload-port /dev/cu.wchusbserial-REPLACE_ME
EOF
}

[[ "$#" -eq 2 ]] || {
  Usage >&2
  exit 2
}

readonly SERIAL_PORT="$1"
readonly FIRMWARE_ELF="$2"

[[ "${RRCLITE_UART_BOOTLOADER_ACK:-}" == \
  "${REQUIRED_ACKNOWLEDGEMENT}" ]] || {
  Usage >&2
  Fail "set RRCLITE_UART_BOOTLOADER_ACK=${REQUIRED_ACKNOWLEDGEMENT} only after entering the ROM bootloader with all actuators disconnected"
}

[[ -n "${SERIAL_PORT}" && "${SERIAL_PORT}" != -* && \
  "${SERIAL_PORT}" != *$'\n'* && "${SERIAL_PORT}" != *$'\r'* ]] || \
  Fail "the upload port is empty or malformed; pass --upload-port explicitly"
[[ -f "${FIRMWARE_ELF}" && -s "${FIRMWARE_ELF}" ]] || \
  Fail "verified PlatformIO firmware snapshot is missing or empty: ${FIRMWARE_ELF}"
[[ ! -L "${FIRMWARE_ELF}" ]] || \
  Fail "verified PlatformIO firmware snapshot must not be a symbolic link"
readonly SNAPSHOT_BASENAME="$(basename "${FIRMWARE_ELF}")"
readonly SNAPSHOT_DIRECTORY="$(basename "$(dirname "${FIRMWARE_ELF}")")"
[[ "${SNAPSHOT_DIRECTORY}" == "verified-artifacts" && \
  "${SNAPSHOT_BASENAME}" =~ ^mentor_pi_mcu-([0-9a-f]{64})\.elf$ ]] || \
  Fail "PlatformIO must provide its hash-named verified ELF snapshot, not an arbitrary image"
readonly EXPECTED_ELF_SHA256="${BASH_REMATCH[1]}"
readonly ACTUAL_ELF_SHA256="$(Sha256 "${FIRMWARE_ELF}")"
[[ "${ACTUAL_ELF_SHA256}" == "${EXPECTED_ELF_SHA256}" ]] || \
  Fail "verified PlatformIO firmware snapshot changed after preparation"

programmer="${STM32_CUBE_PROGRAMMER_CLI:-}"
if [[ -n "${programmer}" ]]; then
  [[ -x "${programmer}" ]] || \
    Fail "STM32_CUBE_PROGRAMMER_CLI is not executable: ${programmer}"
elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then
  programmer="$(command -v STM32_Programmer_CLI)"
else
  readonly MACOS_PROGRAMMER="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
  if [[ -x "${MACOS_PROGRAMMER}" ]]; then
    programmer="${MACOS_PROGRAMMER}"
  else
    Fail "STM32_Programmer_CLI was not found; install STM32CubeProgrammer or set STM32_CUBE_PROGRAMMER_CLI to its executable"
  fi
fi
readonly programmer

echo "Programming the verified motor-profile-bound ELF over ${SERIAL_PORT}."
echo "Verified ELF SHA-256: ${ACTUAL_ELF_SHA256}"
echo "CubeProgrammer UART bootloader settings: 115200 baud, 8 data bits, even parity, 1 stop bit, flow control off."
if ! "${programmer}" \
  -c "port=${SERIAL_PORT}" br=115200 \
  -w "${FIRMWARE_ELF}" -v; then
  Fail "CubeProgrammer programming or read-back verification failed"
fi

echo "UART programming and read-back verification succeeded."
echo "Release BOOT and press RST normally to run the image from internal flash."
