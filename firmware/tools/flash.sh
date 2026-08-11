#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly MCU_ROOT="$(cd "${SCRIPT_DIR}/../mentor_pi_mcu" && pwd)"
readonly REQUIRED_ACK="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"
readonly PORT="${1:-}"

Fail() { echo "Firmware flash error: $*" >&2; exit 1; }
[[ "$#" == 1 ]] || Fail "usage: flash.sh /dev/SERIAL_PORT"
[[ "${RRCLITE_UART_BOOTLOADER_ACK:-}" == "${REQUIRED_ACK}" ]] || \
  Fail "set FLASH_ACK=${REQUIRED_ACK} only with actuator power disconnected"
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

elf="${MCU_ROOT}/build/stm32/mentor_pi_mcu.elf"
expected="$(sed -n 's/^elf_sha256=//p' \
  "${MCU_ROOT}/build/stm32/rrclite-build-metadata.txt")"
snapshot_dir="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-flash.XXXXXX")"
trap 'rm -rf -- "${snapshot_dir}"' EXIT
cp "${elf}" "${snapshot_dir}/mentor_pi_mcu.elf"
[[ "$("${SCRIPT_DIR}/sha256.sh" "${snapshot_dir}/mentor_pi_mcu.elf")" == \
   "${expected}" ]] || Fail "firmware changed during flash snapshot"

echo "Programming verified PID firmware over ${PORT}; actuators must remain disconnected."
"${programmer}" -c "port=${PORT}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
  rts=low dtr=low -w "${snapshot_dir}/mentor_pi_mcu.elf" -v || \
  Fail "programming or read-back verification failed"
echo "Flash verified. Release BOOT and reset the board into the application."
