#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "${TEST_ROOT}"' EXIT

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

ExpectFailure() {
  local expected_text="$1"
  shift
  local output
  if output="$("$@" 2>&1)"; then
    echo "Expected command to fail: $*" >&2
    exit 1
  fi
  [[ "${output}" == *"${expected_text}"* ]] || {
    echo "Failure did not contain '${expected_text}': ${output}" >&2
    exit 1
  }
}

MakeBaseFixture() {
  local root="$1"
  mkdir -p \
    "${root}/mentor_pi_ros2/src/mentor_pi_interfaces/include/mentor_pi_interfaces" \
    "${root}/mentor_pi_ros2/src/mentor_pi_interfaces/msg" \
    "${root}/mentor_pi_ros2/src/mentor_pi_interfaces/srv" \
    "${root}/firmware/mentor_pi_mcu/build/stm32" \
    "${root}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros" \
    "${root}/tools/docker"
  local directory
  for directory in app config drivers include linker platform src target/stm32; do
    mkdir -p "${root}/firmware/mentor_pi_mcu/${directory}"
    printf 'fixture %s\n' "${directory}" \
      >"${root}/firmware/mentor_pi_mcu/${directory}/input.txt"
  done
  printf 'cmake\n' >"${root}/mentor_pi_ros2/src/mentor_pi_interfaces/CMakeLists.txt"
  printf 'package\n' >"${root}/mentor_pi_ros2/src/mentor_pi_interfaces/package.xml"
  printf 'profile contract\n' \
    >"${root}/mentor_pi_ros2/src/mentor_pi_interfaces/include/mentor_pi_interfaces/motor_profile_contract.hpp"
  printf 'float32 target\n' \
    >"${root}/mentor_pi_ros2/src/mentor_pi_interfaces/msg/MotorCommand.msg"
  printf 'uint8 result\n' \
    >"${root}/mentor_pi_ros2/src/mentor_pi_interfaces/srv/SetMotorModel.srv"
  printf 'firmware cmake\n' \
    >"${root}/firmware/mentor_pi_mcu/CMakeLists.txt"
  printf 'build script\n' >"${root}/tools/build_firmware.sh"
  printf 'memory checker\n' >"${root}/tools/check_firmware_memory.sh"
  printf 'micro-ROS source-lock script\n' \
    >"${root}/tools/apply_microros_source_lock.sh"
  printf 'bootstrap script\n' \
    >"${root}/tools/bootstrap_firmware_dependencies.sh"
  printf 'native toolchain bootstrap script\n' \
    >"${root}/tools/bootstrap_native_arm_toolchain.sh"
  printf 'micro-ROS build script\n' \
    >"${root}/tools/build_microros_library.sh"
  printf 'dockerfile\n' >"${root}/tools/docker/firmware-builder.Dockerfile"
  printf 'micro-ROS dockerfile\n' \
    >"${root}/tools/docker/microros-builder.Dockerfile"
  cp "${SCRIPT_DIR}/firmware_source_fingerprint.sh" "${root}/tools/"
  chmod +x "${root}/tools/firmware_source_fingerprint.sh"

  local library_root="${root}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros"
  mkdir -p "${library_root}/include/mentor_pi_interfaces/msg"
  printf 'archive\n' >"${library_root}/libmicroros.a"
  printf 'header\n' \
    >"${library_root}/include/mentor_pi_interfaces/msg/motor_command.h"
  printf 'types\n' >"${library_root}/available_ros2_types"
  printf 'packages\n' >"${library_root}/built_packages"
  printf 'humble\n' >"${library_root}/ros_distro"
  cp "${SCRIPT_DIR}/microros_artifact_fingerprint.sh" "${root}/tools/"
  chmod +x "${root}/tools/microros_artifact_fingerprint.sh"
  printf '%s\n' "$(Sha256 "${library_root}/libmicroros.a")" \
    >"${root}/firmware/mentor_pi_mcu/config/microros_artifact.sha256"
  local microros_tree_sha256
  microros_tree_sha256="$(
    "${root}/tools/microros_artifact_fingerprint.sh" "${root}"
  )"
  printf '%s\n' "${microros_tree_sha256}" \
    >"${root}/firmware/mentor_pi_mcu/config/microros_artifact_tree.sha256"
  "${root}/tools/firmware_source_fingerprint.sh" interfaces "${root}" \
    >"${library_root}/mentor_pi_interfaces.source.sha256"

  local build_root="${root}/firmware/mentor_pi_mcu/build/stm32"
  printf 'elf\n' >"${build_root}/mentor_pi_mcu.elf"

  local source_sha256
  source_sha256="$(
    "${root}/tools/firmware_source_fingerprint.sh" firmware "${root}"
  )"
  local interfaces_sha256
  interfaces_sha256="$(
    "${root}/tools/firmware_source_fingerprint.sh" interfaces "${root}"
  )"
  printf '%s\n' \
    'schema=rrclite-firmware-build-v2' \
    'target=STM32F407VET6' \
    'ros_distro=humble' \
    'builder_mode=docker-pinned' \
    'artifact_mode=NORMAL' \
    'release_qualified=0' \
    "source_sha256=${source_sha256}" \
    "interfaces_sha256=${interfaces_sha256}" \
    "microros_archive_sha256=$(Sha256 "${library_root}/libmicroros.a")" \
    "microros_tree_sha256=${microros_tree_sha256}" \
    "elf_sha256=$(Sha256 "${build_root}/mentor_pi_mcu.elf")" \
    >"${build_root}/metadata-common.txt"
}

WriteCache() {
  local root="$1"
  local build_root="${root}/firmware/mentor_pi_mcu/build/stm32"
  printf '%s\n' \
    "CMAKE_BUILD_TYPE:STRING=MinSizeRel" \
    >"${build_root}/CMakeCache.txt"
}

WriteMetadata() {
  local root="$1"
  local motor_mode="$2"
  local build_root="${root}/firmware/mentor_pi_mcu/build/stm32"
  {
    cat "${build_root}/metadata-common.txt"
    printf '%s\n' \
      "motor_mode=${motor_mode}" \
      "control_mode=CLOSED_LOOP"
  } >"${build_root}/rrclite-build-metadata.txt"
}

MakePidFixture() {
  local root="$1"
  MakeBaseFixture "${root}"
  WriteCache "${root}"
  WriteMetadata "${root}" PID
}

MakeLockedMetadataFixture() {
  local root="$1"
  MakeBaseFixture "${root}"
  WriteCache "${root}"
  WriteMetadata "${root}" LOCKED
}

readonly PID_VALID="${TEST_ROOT}/pid-valid"
MakePidFixture "${PID_VALID}"
"${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${PID_VALID}" >/dev/null

readonly WRONG_MODE_LOCKED="${TEST_ROOT}/wrong-mode-locked"
MakeLockedMetadataFixture "${WRONG_MODE_LOCKED}"
ExpectFailure 'artifact motor mode is LOCKED' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${WRONG_MODE_LOCKED}"

readonly FALSE_QUALIFICATION="${TEST_ROOT}/false-qualification"
cp -R "${PID_VALID}" "${FALSE_QUALIFICATION}"
sed -i 's/^release_qualified=0$/release_qualified=1/' \
  "${FALSE_QUALIFICATION}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'release qualification pending HIL evidence' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${FALSE_QUALIFICATION}"

readonly INVALID_BUILDER_MODE="${TEST_ROOT}/invalid-builder-mode"
cp -R "${PID_VALID}" "${INVALID_BUILDER_MODE}"
sed -i 's/^builder_mode=docker-pinned$/builder_mode=native-explicit/' \
  "${INVALID_BUILDER_MODE}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'unsupported builder mode' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${INVALID_BUILDER_MODE}"

readonly STALE_ELF="${TEST_ROOT}/stale-elf"
cp -R "${PID_VALID}" "${STALE_ELF}"
printf 'changed\n' \
  >>"${STALE_ELF}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
ExpectFailure 'ELF changed' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${STALE_ELF}"

readonly MISSING_VERIFIER="${TEST_ROOT}/missing-verifier"
cp -R "${PID_VALID}" "${MISSING_VERIFIER}"
chmod -x "${MISSING_VERIFIER}/tools/firmware_source_fingerprint.sh"
ExpectFailure 'fingerprint tool is not executable' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${MISSING_VERIFIER}"

echo "Firmware artifact verification tests passed."
