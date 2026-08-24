#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly MCU_ROOT="$(cd "${SCRIPT_DIR}/../mentor_pi_mcu" && pwd)"
readonly REQUIRED_ACK="ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED"
readonly AUTOMATIC_BOOT_CONTROL="${RRCLITE_AUTOMATIC_BOOT_CONTROL:-1}"
readonly BOOT_CONTROL_BUILDER="${SCRIPT_DIR}/build_boot_control.sh"
readonly PACKAGE_VERIFIER="${SCRIPT_DIR}/verify_package.sh"
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

package=""
expected_namespace=""
expected_manifest_sha256=""
case "$#" in
  1)
    port="$1"
    ;;
  7)
    [[ "$1" == --package && "$3" == --expected-namespace && \
       "$5" == --expected-manifest-sha256 ]] || \
      Fail "usage: flash.sh [--package DIRECTORY --expected-namespace /ROBOT --expected-manifest-sha256 SHA256] /dev/SERIAL_PORT"
    package="$2"
    expected_namespace="$4"
    expected_manifest_sha256="$6"
    port="$7"
    ;;
  *)
    Fail "usage: flash.sh [--package DIRECTORY --expected-namespace /ROBOT --expected-manifest-sha256 SHA256] /dev/SERIAL_PORT"
    ;;
esac
readonly package expected_namespace expected_manifest_sha256 port
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
[[ "${port}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && -c "${port}" ]] || \
  Fail "serial port must be an existing explicit /dev path"
[[ -r "${port}" && -w "${port}" ]] || Fail "serial port is not readable/writable"
if command -v fuser >/dev/null 2>&1 && fuser "${port}" >/dev/null 2>&1; then
  Fail "another process owns ${port}; stop mentor-pi-agent.service first"
fi
if command -v lsof >/dev/null 2>&1 && lsof "${port}" >/dev/null 2>&1; then
  Fail "another process owns ${port}; stop the Agent first"
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

snapshot_dir="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-flash.XXXXXX")"
CleanupSnapshot() {
  chmod -R u+w "${snapshot_dir}" 2>/dev/null || true
  rm -rf -- "${snapshot_dir}"
}
trap CleanupSnapshot EXIT
if [[ -n "${package}" ]]; then
  "${PACKAGE_VERIFIER}" \
    --expected-namespace "${expected_namespace}" \
    --expected-manifest-sha256 "${expected_manifest_sha256}" \
    "${package}" >/dev/null
  verified_package="$(cd "${package}" && pwd -P)"
  snapshot_package="${snapshot_dir}/firmware-adrc-release"
  mkdir -p "${snapshot_package}"
  for file in \
      BUILD-METADATA.txt \
      BUILD-MODE.txt \
      SHA256SUMS \
      mentor_pi_mcu-firmware-adrc-release.bin \
      mentor_pi_mcu-firmware-adrc-release.elf \
      mentor_pi_mcu-firmware-adrc-release.hex \
      mentor_pi_mcu-firmware-adrc-release.map; do
    cp "${verified_package}/${file}" "${snapshot_package}/${file}"
  done
  "${PACKAGE_VERIFIER}" \
    --expected-namespace "${expected_namespace}" \
    --expected-manifest-sha256 "${expected_manifest_sha256}" \
    "${snapshot_package}" >/dev/null
  elf="${snapshot_package}/mentor_pi_mcu-firmware-adrc-release.elf"
  metadata="${snapshot_package}/BUILD-METADATA.txt"
else
  source_elf="${MCU_ROOT}/build/stm32/mentor_pi_mcu.elf"
  metadata="${MCU_ROOT}/build/stm32/rrclite-build-metadata.txt"
  elf="${snapshot_dir}/mentor_pi_mcu.elf"
  cp "${source_elf}" "${elf}"
fi
expected="$(sed -n 's/^elf_sha256=//p' "${metadata}")"
[[ "${expected}" =~ ^[0-9a-f]{64}$ ]] || \
  Fail "firmware metadata must contain one valid elf_sha256 value"
[[ "$("${SCRIPT_DIR}/sha256.sh" "${elf}")" == "${expected}" ]] || \
  Fail "firmware changed during flash snapshot"
elf_header="$(od -An -tx1 -N20 "${elf}" | tr -d '[:space:]')"
[[ "${elf_header}" =~ ^7f454c46010101[0-9a-f]{18}02002800$ ]] || \
  Fail "firmware ELF is not a 32-bit little-endian ARM executable"
chmod -R a-w "${snapshot_dir}"

echo "Programming verified ADRC firmware over ${port}; actuators must remain disconnected."
if [[ "${AUTOMATIC_BOOT_CONTROL}" == 1 ]]; then
  echo "Entering the STM32 ROM bootloader through separate CH9102F RTS/DTR set/clear operations."
  "${boot_control}" --device "${port}" --mode bootloader || \
    Fail "automatic CH9102F ROM-bootloader entry failed"
  echo "Probing the STM32 ROM bootloader before programming."
  if ! RunWithTimeout "${PREFLIGHT_TIMEOUT_SEC}" "${programmer}" \
      -c "port=${port}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
      rts=low dtr=low; then
    Fail "automatic bootloader activation failed before programming; the MCU remains in bootloader mode"
  fi
else
  echo "Automatic boot control is disabled; the operator must already have used the documented BOOT/RST sequence."
fi

RunWithTimeout "${FLASH_TIMEOUT_SEC}" "${programmer}" \
  -c "port=${port}" br=115200 P=EVEN db=8 sb=1 fc=OFF \
  rts=low dtr=low -w "${elf}" -v || \
  Fail "programming or read-back verification failed"
if [[ "${AUTOMATIC_BOOT_CONTROL}" == 1 ]]; then
  echo "Resetting into the verified application through separate CH9102F RTS/DTR set/clear operations."
  "${boot_control}" --device "${port}" --mode application || \
    Fail "firmware is verified, but the automatic normal-boot reset failed"
  echo "Flash verified and the application reset completed."
else
  echo "Flash verified. Release BOOT and reset the board into the application."
fi
