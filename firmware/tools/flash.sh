#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly MCU_ROOT="$(cd "${SCRIPT_DIR}/../mentor_pi_mcu" && pwd)"
readonly REQUIRED_ACK="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"
readonly PORT="${1:-}"
readonly AUTOMATIC_BOOT_CONTROL="${RRCLITE_AUTOMATIC_BOOT_CONTROL:-1}"
readonly BOOT_CONTROL_BUILDER="${SCRIPT_DIR}/build_boot_control.sh"
readonly PREFLIGHT_TIMEOUT_SEC="${RRCLITE_PROGRAMMER_PREFLIGHT_TIMEOUT_SEC:-15}"
readonly FLASH_TIMEOUT_SEC="${RRCLITE_PROGRAMMER_FLASH_TIMEOUT_SEC:-300}"

Fail() { echo "Firmware flash error: $*" >&2; exit 1; }
RunWithTimeout() {
  local duration="$1"
  shift
  if command -v timeout >/dev/null 2>&1; then
    timeout --foreground "${duration}" "$@"
  else
    "$@"
  fi
}

[[ "$#" == 1 ]] || Fail "usage: flash.sh /dev/SERIAL_PORT"
[[ "${RRCLITE_UART_BOOTLOADER_ACK:-}" == "${REQUIRED_ACK}" ]] || \
  Fail "set FLASH_ACK=${REQUIRED_ACK} only with actuator power disconnected"
[[ ! -f /.dockerenv ]] || \
  Fail "flashing requires the physical host, not the VS Code Dev Container"
[[ "${AUTOMATIC_BOOT_CONTROL}" == 0 || \
   "${AUTOMATIC_BOOT_CONTROL}" == 1 ]] || \
  Fail "AUTOMATIC_BOOT_CONTROL must be 0 or 1"
[[ "${PREFLIGHT_TIMEOUT_SEC}" =~ ^[1-9][0-9]{0,2}$ && \
   "${FLASH_TIMEOUT_SEC}" =~ ^[1-9][0-9]{0,3}$ ]] || \
  Fail "CubeProgrammer timeouts must be positive integer seconds"
[[ "${PORT}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && -c "${PORT}" ]] || \
  Fail "serial port must be an existing explicit /dev path"
[[ -r "${PORT}" && -w "${PORT}" ]] || Fail "serial port is not readable/writable"
if command -v fuser >/dev/null 2>&1 && fuser "${PORT}" >/dev/null 2>&1; then
  Fail "another process owns ${PORT}; stop mentor-pi-agent.service first"
fi
if command -v lsof >/dev/null 2>&1 && lsof "${PORT}" >/dev/null 2>&1; then
  Fail "another process owns ${PORT}; stop the Agent first"
fi

programmer="${STM32_CUBE_PROGRAMMER_CLI:-}"
if [[ -z "${programmer}" ]]; then
  for candidate in \
      "$(command -v STM32_Programmer_CLI 2>/dev/null || true)" \
      /opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI \
      /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI \
      /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI; do
    if [[ -n "${candidate}" && -x "${candidate}" ]]; then programmer="${candidate}"; break; fi
  done
fi
[[ -x "${programmer}" ]] || Fail "STM32_Programmer_CLI was not found"

boot_control="${RRCLITE_CH9102_BOOT_CONTROL:-}"
if [[ "${AUTOMATIC_BOOT_CONTROL}" == 1 ]]; then
  [[ "$(uname -s)" == Linux ]] || \
    Fail "automatic CH9102F boot control requires Linux; use AUTOMATIC_BOOT_CONTROL=0 only for the documented manual BOOT/RST fallback"
  if [[ -z "${boot_control}" ]]; then
    [[ -x "${BOOT_CONTROL_BUILDER}" ]] || \
      Fail "CH9102F boot-control builder is unavailable"
    boot_control="$("${BOOT_CONTROL_BUILDER}")" || \
      Fail "CH9102F boot-control helper could not be built"
  fi
  [[ -x "${boot_control}" && ! -L "${boot_control}" ]] || \
    Fail "CH9102F boot-control helper is missing, symbolic, or not executable"
fi
readonly boot_control

elf="${MCU_ROOT}/build/stm32/mentor_pi_mcu.elf"
expected="$(sed -n 's/^elf_sha256=//p' \
  "${MCU_ROOT}/build/stm32/rrclite-build-metadata.txt")"
snapshot_dir="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-flash.XXXXXX")"
trap 'rm -rf -- "${snapshot_dir}"' EXIT
cp "${elf}" "${snapshot_dir}/mentor_pi_mcu.elf"
[[ "$("${SCRIPT_DIR}/sha256.sh" "${snapshot_dir}/mentor_pi_mcu.elf")" == \
   "${expected}" ]] || Fail "firmware changed during flash snapshot"

echo "Programming verified PID firmware over ${PORT}; actuators must remain disconnected."
if [[ "${AUTOMATIC_BOOT_CONTROL}" == 1 ]]; then
  echo "Entering the STM32 ROM bootloader through separate CH9102F RTS/DTR set/clear operations."
  "${boot_control}" --device "${PORT}" --mode bootloader || \
    Fail "automatic CH9102F ROM-bootloader entry failed"
  echo "Probing the STM32 ROM bootloader before programming."
  if ! RunWithTimeout "${PREFLIGHT_TIMEOUT_SEC}" "${programmer}" \
      -c "port=${PORT}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
      rts=low dtr=low; then
    Fail "automatic bootloader activation failed before programming; the MCU remains in bootloader mode"
  fi
else
  echo "Automatic boot control is disabled; the operator must already have used the documented BOOT/RST sequence."
fi

RunWithTimeout "${FLASH_TIMEOUT_SEC}" "${programmer}" \
  -c "port=${PORT}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
  rts=low dtr=low -w "${snapshot_dir}/mentor_pi_mcu.elf" -v || \
  Fail "programming or read-back verification failed"
if [[ "${AUTOMATIC_BOOT_CONTROL}" == 1 ]]; then
  echo "Resetting into the verified application through separate CH9102F RTS/DTR set/clear operations."
  "${boot_control}" --device "${PORT}" --mode application || \
    Fail "firmware is verified, but the automatic normal-boot reset failed"
  echo "Flash verified and the application reset completed."
else
  echo "Flash verified. Release BOOT and reset the board into the application."
fi
