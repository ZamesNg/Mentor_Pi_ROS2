#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() {
  echo "Firmware artifact verification failed: $*" >&2
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
  local key="$1"
  local line
  line="$(grep -E "^${key}=" "${METADATA}" || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] || \
    Fail "metadata must contain exactly one ${key} entry"
  printf '%s' "${line#*=}"
}

Usage() {
  cat <<'EOF'
Usage: ./tools/verify_firmware_artifact.sh LOCKED|COMMISSIONING [PROJECT_ROOT]

Verify that the authoritative ELF, build profile, micro-ROS interface library,
and project-owned sources all match the successful-build metadata. This is a
read-only gate used before firmware flash operations.
EOF
}

[[ "$#" -ge 1 && "$#" -le 2 ]] || {
  Usage >&2
  exit 2
}
readonly EXPECTED_MODE="$1"
readonly PROJECT_ROOT="${2:-${DEFAULT_PROJECT_ROOT}}"
case "${EXPECTED_MODE}" in
  LOCKED | COMMISSIONING)
    ;;
  *)
    Usage >&2
    Fail "expected mode must be LOCKED or COMMISSIONING"
    ;;
esac

readonly BUILD_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32"
readonly CACHE="${BUILD_ROOT}/CMakeCache.txt"
readonly ELF="${BUILD_ROOT}/mentor_pi_mcu.elf"
readonly METADATA="${BUILD_ROOT}/rrclite-build-metadata.txt"
readonly MICROROS_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros"
readonly MICROROS_ARCHIVE="${MICROROS_ROOT}/libmicroros.a"
readonly MICROROS_ROS_DISTRO="${MICROROS_ROOT}/ros_distro"
readonly MICROROS_INTERFACE_FINGERPRINT="${MICROROS_ROOT}/mentor_pi_interfaces.source.sha256"
readonly FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/firmware_source_fingerprint.sh"
readonly MICROROS_FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/microros_artifact_fingerprint.sh"
readonly PINNED_MICROROS_ARCHIVE_HASH="${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_artifact.sha256"
readonly PINNED_MICROROS_TREE_HASH="${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_artifact_tree.sha256"

[[ -f "${CACHE}" ]] || Fail "missing CMake cache; rebuild firmware"
[[ -s "${ELF}" ]] || Fail "missing or empty ELF; rebuild firmware"
[[ -f "${METADATA}" ]] || Fail "missing build metadata; rebuild firmware"
[[ -s "${MICROROS_ARCHIVE}" ]] || Fail "missing micro-ROS archive"
[[ -f "${MICROROS_ROS_DISTRO}" ]] || Fail "missing micro-ROS ROS identity"
[[ "$(tr -d '[:space:]' <"${MICROROS_ROS_DISTRO}")" == "humble" ]] ||
  Fail "micro-ROS library targets a different ROS distribution"
[[ -f "${MICROROS_INTERFACE_FINGERPRINT}" ]] || \
  Fail "missing micro-ROS interface fingerprint; regenerate the library"
[[ -x "${FINGERPRINT_TOOL}" ]] || Fail "fingerprint tool is not executable"
[[ -x "${MICROROS_FINGERPRINT_TOOL}" ]] || \
  Fail "micro-ROS artifact fingerprint tool is not executable"
[[ -f "${PINNED_MICROROS_ARCHIVE_HASH}" ]] || \
  Fail "pinned micro-ROS archive hash is missing"
[[ -f "${PINNED_MICROROS_TREE_HASH}" ]] || \
  Fail "pinned micro-ROS tree hash is missing"

[[ "$(ReadMetadata schema)" == "rrclite-firmware-build-v2" ]] || \
  Fail "unsupported or missing build metadata schema"
[[ "$(ReadMetadata target)" == "STM32F407VET6" ]] || \
  Fail "build metadata targets a different MCU"
[[ "$(ReadMetadata ros_distro)" == "humble" ]] || \
  Fail "build metadata targets a different ROS distribution"
[[ "$(ReadMetadata release_qualified)" == "0" ]] || \
  Fail "build metadata must classify the artifact as non-release"

readonly RECORDED_MODE="$(ReadMetadata motor_mode)"
[[ "${RECORDED_MODE}" == "${EXPECTED_MODE}" ]] || \
  Fail "artifact is ${RECORDED_MODE}, but ${EXPECTED_MODE} was requested"
readonly RECORDED_COMMISSIONING_ACK="$(ReadMetadata commissioning_ack)"

if [[ "${EXPECTED_MODE}" == "LOCKED" ]]; then
  [[ -z "${RECORDED_COMMISSIONING_ACK}" ]] || \
    Fail "locked build metadata contains a commissioning acknowledgement"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING:BOOL=OFF' "${CACHE}" || \
    Fail "CMake cache is not motor-locked"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=' "${CACHE}" || \
    Fail "locked build contains a commissioning acknowledgement"
else
  [[ "${RECORDED_COMMISSIONING_ACK}" == "MOTORS_RAISED" ]] || \
    Fail "commissioning build metadata acknowledgement is missing or invalid"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING:BOOL=ON' "${CACHE}" || \
    Fail "CMake cache is not a commissioning build"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=MOTORS_RAISED' \
    "${CACHE}" || Fail "commissioning acknowledgement is missing"
fi

readonly CURRENT_SOURCE_SHA256="$(
  "${FINGERPRINT_TOOL}" firmware "${PROJECT_ROOT}"
)"
readonly RECORDED_SOURCE_SHA256="$(ReadMetadata source_sha256)"
[[ "${CURRENT_SOURCE_SHA256}" == "${RECORDED_SOURCE_SHA256}" ]] || \
  Fail "project-owned firmware inputs changed after the build; rebuild"

readonly CURRENT_INTERFACE_SHA256="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"
readonly LIBRARY_INTERFACE_SHA256="$(tr -d '[:space:]' \
  <"${MICROROS_INTERFACE_FINGERPRINT}")"
readonly RECORDED_INTERFACE_SHA256="$(ReadMetadata interfaces_sha256)"
[[ "${CURRENT_INTERFACE_SHA256}" == "${LIBRARY_INTERFACE_SHA256}" ]] || \
  Fail "micro-ROS library was generated from different interface sources"
[[ "${CURRENT_INTERFACE_SHA256}" == "${RECORDED_INTERFACE_SHA256}" ]] || \
  Fail "firmware metadata references different interface sources"

readonly CURRENT_MICROROS_SHA256="$(Sha256 "${MICROROS_ARCHIVE}")"
readonly RECORDED_MICROROS_SHA256="$(ReadMetadata microros_archive_sha256)"
readonly EXPECTED_MICROROS_SHA256="$(tr -d '[:space:]' \
  <"${PINNED_MICROROS_ARCHIVE_HASH}")"
[[ "${CURRENT_MICROROS_SHA256}" == "${EXPECTED_MICROROS_SHA256}" ]] || \
  Fail "micro-ROS archive differs from the pinned reviewed artifact"
[[ "${CURRENT_MICROROS_SHA256}" == "${RECORDED_MICROROS_SHA256}" ]] || \
  Fail "micro-ROS archive changed after the build"

readonly CURRENT_MICROROS_TREE_SHA256="$(
  "${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}"
)"
readonly RECORDED_MICROROS_TREE_SHA256="$(
  ReadMetadata microros_tree_sha256
)"
readonly EXPECTED_MICROROS_TREE_SHA256="$(tr -d '[:space:]' \
  <"${PINNED_MICROROS_TREE_HASH}")"
[[ "${CURRENT_MICROROS_TREE_SHA256}" == \
    "${EXPECTED_MICROROS_TREE_SHA256}" ]] || \
  Fail "micro-ROS generated header/archive tree differs from the pinned artifact"
[[ "${CURRENT_MICROROS_TREE_SHA256}" == \
    "${RECORDED_MICROROS_TREE_SHA256}" ]] || \
  Fail "micro-ROS generated header/archive tree changed after the build"

readonly CURRENT_ELF_SHA256="$(Sha256 "${ELF}")"
readonly RECORDED_ELF_SHA256="$(ReadMetadata elf_sha256)"
[[ "${CURRENT_ELF_SHA256}" == "${RECORDED_ELF_SHA256}" ]] || \
  Fail "ELF changed after its successful-build metadata was written"

echo "Verified ${EXPECTED_MODE} firmware artifact: ${ELF}"
