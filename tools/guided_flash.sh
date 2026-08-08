#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly REQUIRED_FLASH_ACK="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"
readonly REQUIRED_COMMISSIONING_ACK="MOTORS_RAISED_CURRENT_LIMITED"
readonly SERIAL_GROUP="mentor-pi-serial"

Fail() {
  echo "Guided firmware flash failed: $*" >&2
  exit 1
}

PromptExact() {
  local variable_name="$1"
  local required="$2"
  local prompt="$3"
  local current=""
  case "${variable_name}" in
    RRCLITE_UART_BOOTLOADER_ACK)
      current="${RRCLITE_UART_BOOTLOADER_ACK:-}"
      ;;
    RRCLITE_COMMISSIONING_FLASH_ACK)
      current="${RRCLITE_COMMISSIONING_FLASH_ACK:-}"
      ;;
    *) Fail "unsupported acknowledgement variable" ;;
  esac
  if [[ -z "${current}" ]]; then
    [[ -t 0 ]] || \
      Fail "noninteractive use requires ${variable_name}=${required}"
    printf '%s\nType exactly: %s\n> ' "${prompt}" "${required}" >&2
    IFS= read -r current
  fi
  [[ "${current}" == "${required}" ]] || \
    Fail "acknowledgement did not match ${required}"
  case "${variable_name}" in
    RRCLITE_UART_BOOTLOADER_ACK)
      export RRCLITE_UART_BOOTLOADER_ACK="${current}"
      ;;
    RRCLITE_COMMISSIONING_FLASH_ACK)
      export RRCLITE_COMMISSIONING_FLASH_ACK="${current}"
      ;;
  esac
}

ActivateSerialGroupIfNeeded() {
  [[ -c "${PORT}" ]] || \
    Fail "serial port does not resolve to an existing character device: ${PORT}"

  local port_group=""
  port_group="$(stat -L -c %G -- "${PORT}")" || \
    Fail "could not inspect the group owning ${PORT}"
  [[ "${port_group}" == "${SERIAL_GROUP}" ]] || return 0

  if id -nG | tr ' ' '\n' | grep -Fqx "${SERIAL_GROUP}"; then
    return 0
  fi

  local current_user=""
  current_user="$(id -un)"
  if ! id -nG "${current_user}" | tr ' ' '\n' | \
      grep -Fqx "${SERIAL_GROUP}"; then
    Fail "${current_user} is not a member of ${SERIAL_GROUP}; run make serial-setup"
  fi
  command -v sg >/dev/null 2>&1 || \
    Fail "the ${SERIAL_GROUP} membership is inactive and the sg command is unavailable"

  local command_line=""
  printf -v command_line 'exec %q %q %q' \
    "${SCRIPT_DIR}/guided_flash.sh" "${MODE}" "${PORT}"
  echo "Activating the existing ${SERIAL_GROUP} membership for this flash."
  exec sg "${SERIAL_GROUP}" -c "${command_line}"
}

[[ "$#" == 2 ]] || {
  echo "Usage: guided_flash.sh LOCKED|COMMISSIONING|COMMISSIONING_PID /dev/mentor_pi_mcu" >&2
  exit 2
}
readonly MODE="$1"
readonly PORT="$2"
case "${MODE}" in
  LOCKED | COMMISSIONING | COMMISSIONING_PID) ;;
  *) Fail "mode must be LOCKED, COMMISSIONING, or COMMISSIONING_PID" ;;
esac
[[ "${PORT}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && \
  "${PORT}" != *"/../"* && "${PORT}" != */.. && \
  "${PORT}" != *"/./"* && "${PORT}" != */. ]] || \
  Fail "serial port must be an explicit, well-formed /dev path"

ActivateSerialGroupIfNeeded
[[ -r "${PORT}" && -w "${PORT}" ]] || \
  Fail "serial port is not readable and writable after activating ${SERIAL_GROUP}: ${PORT}"

PromptExact RRCLITE_UART_BOOTLOADER_ACK "${REQUIRED_FLASH_ACK}" \
  "Disconnect motor power, PWM servos, and bus servos before flashing."
if [[ "${MODE}" == "COMMISSIONING" || "${MODE}" == "COMMISSIONING_PID" ]]; then
  PromptExact RRCLITE_COMMISSIONING_FLASH_ACK \
    "${REQUIRED_COMMISSIONING_ACK}" \
    "Confirm every wheel is raised and the motor supply is current-limited."
fi

if command -v fuser >/dev/null 2>&1 && fuser "${PORT}" >/dev/null 2>&1; then
  fuser -v "${PORT}" >&2 || true
  Fail "another process owns ${PORT}; stop make start or the Agent first"
fi

readonly BOOT_CONTROL="$(${SCRIPT_DIR}/build_ch9102_boot_control.sh)"
set +e
RRCLITE_AUTOMATIC_BOOT_CONTROL=1 \
RRCLITE_CH9102_BOOT_CONTROL="${BOOT_CONTROL}" \
  "${SCRIPT_DIR}/flash_firmware.sh" "${MODE}" "${PORT}"
status=$?
set -e

if ((status == 0)); then
  exit 0
fi
if ((status != 3)); then
  Fail "automatic flash failed; no application reset was attempted"
fi

[[ -t 0 ]] || \
  Fail "automatic bootloader activation failed and manual fallback needs a terminal"
printf '%s\n' \
  "Automatic ROM-bootloader activation failed before programming." \
  "Manual fallback: hold BOOT, tap RST, release BOOT, then press Enter." >&2
IFS= read -r _

RRCLITE_AUTOMATIC_BOOT_CONTROL=0 \
  "${SCRIPT_DIR}/flash_firmware.sh" "${MODE}" "${PORT}"

echo "Manual flash verified. Resetting the verified image into normal boot."
"${BOOT_CONTROL}" --device "${PORT}" --mode application
