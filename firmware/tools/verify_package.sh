#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

requested_package=""
expected_namespace=""
expected_manifest_sha256=""

Fail() {
  echo "Firmware package verification error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --expected-namespace)
      (($# >= 2)) || Fail "--expected-namespace requires a value"
      expected_namespace="$2"
      shift 2
      ;;
    --expected-manifest-sha256)
      (($# >= 2)) || Fail "--expected-manifest-sha256 requires a value"
      expected_manifest_sha256="$2"
      shift 2
      ;;
    --*)
      Fail "unknown option: $1"
      ;;
    *)
      [[ -z "${requested_package}" ]] || Fail "more than one package directory was provided"
      requested_package="$1"
      shift
      ;;
  esac
done
[[ -n "${requested_package}" && -d "${requested_package}" && \
   ! -L "${requested_package}" ]] || \
  Fail "usage: verify_package.sh [--expected-namespace /ROBOT] [--expected-manifest-sha256 SHA256] FIRMWARE_ADRC_RELEASE_DIRECTORY"
[[ -z "${expected_namespace}" || \
   "${expected_namespace}" =~ ^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$ ]] || \
  Fail "expected ROS namespace is invalid"
[[ -z "${expected_manifest_sha256}" || \
   "${expected_manifest_sha256}" =~ ^[0-9a-f]{64}$ ]] || \
  Fail "expected manifest SHA-256 is invalid"
readonly PACKAGE="$(cd "${requested_package}" && pwd -P)"
readonly METADATA="${PACKAGE}/BUILD-METADATA.txt"
readonly MODE="${PACKAGE}/BUILD-MODE.txt"
readonly CHECKSUMS="${PACKAGE}/SHA256SUMS"

readonly -a PAYLOAD_FILES=(
  BUILD-METADATA.txt
  BUILD-MODE.txt
  mentor_pi_mcu-firmware-adrc-release.bin
  mentor_pi_mcu-firmware-adrc-release.elf
  mentor_pi_mcu-firmware-adrc-release.hex
  mentor_pi_mcu-firmware-adrc-release.map
)
readonly -a PACKAGE_FILES=("${PAYLOAD_FILES[@]}" SHA256SUMS)
readonly -a METADATA_KEYS=(
  schema
  target
  ros_distro
  builder_mode
  build_environment
  host_os
  host_architecture
  toolchain
  motor_mode
  control_mode
  artifact_mode
  classification
  release_qualified
  ros_namespace
  source_sha256
  interfaces_sha256
  microros_sdk_archive_sha256
  microros_sdk_tree_sha256
  elf_sha256
  hex_sha256
  bin_sha256
  map_sha256
)

temporary="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-package-verify.XXXXXX")"
trap 'rm -rf -- "${temporary}"' EXIT
expected_listing="${temporary}/expected-listing"
actual_listing="${temporary}/actual-listing"
actual_checksums="${temporary}/actual-SHA256SUMS"
expected_mode="${temporary}/expected-BUILD-MODE.txt"

printf '%s\n' "${PACKAGE_FILES[@]}" | LC_ALL=C sort >"${expected_listing}"
find "${PACKAGE}" ! -path "${PACKAGE}" -print | \
  while IFS= read -r entry; do
    printf '%s\n' "${entry#"${PACKAGE}/"}"
  done | LC_ALL=C sort >"${actual_listing}"
cmp -s "${expected_listing}" "${actual_listing}" || \
  Fail "release package contents differ from the exact seven-file contract"

for file in "${PACKAGE_FILES[@]}"; do
  [[ -f "${PACKAGE}/${file}" && -s "${PACKAGE}/${file}" && \
     ! -L "${PACKAGE}/${file}" ]] || \
    Fail "release package entry is missing, empty, or symbolic: ${file}"
done

if [[ -n "${expected_manifest_sha256}" ]]; then
  [[ "$("${SCRIPT_DIR}/sha256.sh" "${CHECKSUMS}")" == \
     "${expected_manifest_sha256}" ]] || \
    Fail "SHA256SUMS does not match the trusted out-of-band digest"
fi

"${SCRIPT_DIR}/sha256_manifest.sh" "${PACKAGE}" \
  "${PAYLOAD_FILES[@]}" >"${actual_checksums}"
cmp -s "${actual_checksums}" "${CHECKSUMS}" || \
  Fail "SHA256SUMS differs from the exact package payload"

printf '%s\n' \
  'target=STM32F407VET6' \
  'motor_mode=ADRC' \
  'control_mode=CLOSED_LOOP' \
  'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
  >"${expected_mode}"
cmp -s "${expected_mode}" "${MODE}" || \
  Fail "BUILD-MODE.txt is not the NORMAL_CLOSED_LOOP_DEFAULT contract"

ReadValue() {
  local key="$1" value
  value="$(sed -n "s/^${key}=//p" "${METADATA}")"
  [[ -n "${value}" && "${value}" != *$'\n'* ]] || \
    Fail "BUILD-METADATA.txt must contain one ${key} value"
  printf '%s' "${value}"
}

[[ "$(wc -l <"${METADATA}" | tr -d '[:space:]')" == \
   "${#METADATA_KEYS[@]}" ]] || \
  Fail "BUILD-METADATA.txt does not contain the exact v3 field set"
for key in "${METADATA_KEYS[@]}"; do
  grep -Eq "^${key}=.+$" "${METADATA}" || \
    Fail "BUILD-METADATA.txt lacks ${key}"
done
[[ "$(ReadValue schema)" == mentor-pi-firmware-build-v3 && \
   "$(ReadValue target)" == STM32F407VET6 && \
   "$(ReadValue ros_distro)" == humble && \
   "$(ReadValue builder_mode)" == native-pinned && \
   "$(ReadValue host_os)" == ubuntu-22.04 && \
   "$(ReadValue toolchain)" == arm-gnu-toolchain-13.2.rel1 && \
   "$(ReadValue motor_mode)" == ADRC && \
   "$(ReadValue control_mode)" == CLOSED_LOOP && \
   "$(ReadValue artifact_mode)" == NORMAL && \
   "$(ReadValue classification)" == NORMAL_CLOSED_LOOP_DEFAULT && \
   "$(ReadValue release_qualified)" == 0 ]] || \
  Fail "BUILD-METADATA.txt has an unsupported identity or motor mode"
[[ "$(ReadValue build_environment)" == native || \
   "$(ReadValue build_environment)" == devcontainer ]] || \
  Fail "BUILD-METADATA.txt has an unsupported build environment"
[[ "$(ReadValue host_architecture)" == amd64 || \
   "$(ReadValue host_architecture)" == arm64 ]] || \
  Fail "BUILD-METADATA.txt has an unsupported host architecture"
[[ "$(ReadValue ros_namespace)" =~ ^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$ ]] || \
  Fail "BUILD-METADATA.txt has an invalid ROS namespace"
[[ -z "${expected_namespace}" || \
   "$(ReadValue ros_namespace)" == "${expected_namespace}" ]] || \
  Fail "package ROS namespace does not match the expected robot namespace"

for key in source_sha256 interfaces_sha256 microros_sdk_archive_sha256 \
    microros_sdk_tree_sha256 elf_sha256 hex_sha256 bin_sha256 map_sha256; do
  [[ "$(ReadValue "${key}")" =~ ^[0-9a-f]{64}$ ]] || \
    Fail "BUILD-METADATA.txt has an invalid ${key}"
done

for extension in elf hex bin map; do
  packaged_name="mentor_pi_mcu-firmware-adrc-release.${extension}"
  packaged_sha="$(sed -n "s/  ${packaged_name}$//p" "${CHECKSUMS}")"
  [[ "${packaged_sha}" =~ ^[0-9a-f]{64}$ && \
     "${packaged_sha}" == "$(ReadValue "${extension}_sha256")" ]] || \
    Fail "BUILD-METADATA.txt ${extension} digest differs from the package"
done


elf_header="$(od -An -tx1 -N20 \
  "${PACKAGE}/mentor_pi_mcu-firmware-adrc-release.elf" | tr -d '[:space:]')"
[[ "${elf_header}" =~ ^7f454c46010101[0-9a-f]{18}02002800$ ]] || \
  Fail "release ELF is not a 32-bit little-endian ARM executable"

UsedMemory() {
  local symbol="$1" origin="$2" address numeric_address
  address="$(awk -v wanted="${symbol}" \
    '$2 == wanted && $3 == "=" {print $1; exit}' \
    "${PACKAGE}/mentor_pi_mcu-firmware-adrc-release.map")"
  [[ "${address}" =~ ^0x[0-9a-fA-F]{1,8}$ ]] || \
    Fail "release map lacks memory symbol ${symbol}"
  numeric_address="$((address))"
  ((numeric_address >= origin)) || \
    Fail "release map memory symbol ${symbol} precedes its region"
  printf '%u' "$((numeric_address - origin))"
}
flash_bytes="$(UsedMemory __flash_image_end__ 0x08000000)"
sram_bytes="$(UsedMemory __ram_used_end__ 0x20000000)"
ccm_bytes="$(UsedMemory __ccm_end__ 0x10000000)"
((flash_bytes <= 419430 && sram_bytes <= 104857 && ccm_bytes <= 52428)) || \
  Fail "release package exceeds the 80% firmware memory budget"

echo "Verified NORMAL_CLOSED_LOOP_DEFAULT firmware package: ${PACKAGE}"
