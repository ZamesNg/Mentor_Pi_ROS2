#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly ARCHIVE="${MCU_ROOT}/sdk/humble/libmicroros.tar.xz"
readonly MANIFEST="${MCU_ROOT}/sdk/humble/manifest.txt"
readonly DESTINATION="${MCU_ROOT}/build/microros-sdk"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"

Fail() {
  echo "Firmware SDK extraction error: $*" >&2
  exit 1
}

ReadValue() {
  local key="$1"
  local value
  value="$(sed -n "s/^${key}=//p" "${MANIFEST}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "manifest must contain one ${key} value"
  printf '%s' "${value}"
}

[[ -s "${ARCHIVE}" && -f "${MANIFEST}" ]] || \
  Fail "checked Humble SDK archive or manifest is missing"
[[ "$(ReadValue format)" == mentor-pi-firmware-sdk-v1 && \
   "$(ReadValue ros_distro)" == humble ]] || Fail "unsupported SDK manifest"
[[ "$("${SHA256}" "${ARCHIVE}")" == "$(ReadValue archive_sha256)" ]] || \
  Fail "checked SDK archive hash does not match its manifest"

if [[ -d "${DESTINATION}" && ! -L "${DESTINATION}" ]]; then
  current_tree="$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" \
    "${DESTINATION}" 2>/dev/null || true)"
  if [[ "${current_tree}" == "$(ReadValue tree_sha256)" ]]; then
    echo "Reusing verified Humble firmware SDK: ${DESTINATION}"
    exit 0
  fi
fi
[[ ! -L "${DESTINATION}" ]] || Fail "SDK destination must not be symbolic"
temporary="${MCU_ROOT}/build/.microros-sdk.tmp.$$"
trap 'rm -rf -- "${temporary}"' EXIT
mkdir -p "${temporary}"
tar -xJf "${ARCHIVE}" -C "${temporary}"
[[ "$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${temporary}")" == \
   "$(ReadValue tree_sha256)" ]] || Fail "extracted SDK tree hash differs"
rm -rf -- "${DESTINATION}"
mv "${temporary}" "${DESTINATION}"
trap - EXIT
echo "Extracted verified Humble firmware SDK: ${DESTINATION}"
