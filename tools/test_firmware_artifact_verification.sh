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

MakeFixture() {
  local root="$1"
  mkdir -p \
    "${root}/src/mentor_pi_interfaces/include/mentor_pi_interfaces" \
    "${root}/src/mentor_pi_interfaces/msg" \
    "${root}/src/mentor_pi_interfaces/srv" \
    "${root}/src/ros_package_schema" \
    "${root}/firmware/mentor_pi_mcu/build/stm32" \
    "${root}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros" \
    "${root}/tools/docker"
  local directory
  for directory in app config drivers include linker platform src target/stm32; do
    mkdir -p "${root}/firmware/mentor_pi_mcu/${directory}"
    printf 'fixture %s\n' "${directory}" \
      >"${root}/firmware/mentor_pi_mcu/${directory}/input.txt"
  done
  printf 'cmake\n' >"${root}/src/mentor_pi_interfaces/CMakeLists.txt"
  printf 'package\n' >"${root}/src/mentor_pi_interfaces/package.xml"
  printf 'schema\n' >"${root}/src/ros_package_schema/package_format3.xsd"
  printf 'profile contract\n' \
    >"${root}/src/mentor_pi_interfaces/include/mentor_pi_interfaces/motor_profile_contract.hpp"
  printf 'float32 target\n' \
    >"${root}/src/mentor_pi_interfaces/msg/MotorCommand.msg"
  printf 'uint8 result\n' \
    >"${root}/src/mentor_pi_interfaces/srv/SetMotorModel.srv"
  printf 'firmware cmake\n' \
    >"${root}/firmware/mentor_pi_mcu/CMakeLists.txt"
  printf 'build script\n' >"${root}/tools/build_firmware.sh"
  printf 'micro-ROS source-lock script\n' \
    >"${root}/tools/apply_microros_source_lock.sh"
  printf 'bootstrap script\n' \
    >"${root}/tools/bootstrap_firmware_dependencies.sh"
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
  printf '%s\n' \
    'RRCLITE_MOTOR_COMMISSIONING:BOOL=OFF' \
    'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=' \
    >"${build_root}/CMakeCache.txt"
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
    'motor_mode=LOCKED' \
    'commissioning_ack=' \
    'release_qualified=0' \
    "source_sha256=${source_sha256}" \
    "interfaces_sha256=${interfaces_sha256}" \
    "microros_archive_sha256=$(Sha256 "${library_root}/libmicroros.a")" \
    "microros_tree_sha256=${microros_tree_sha256}" \
    "elf_sha256=$(Sha256 "${build_root}/mentor_pi_mcu.elf")" \
    >"${build_root}/rrclite-build-metadata.txt"
}

readonly BASE="${TEST_ROOT}/base"
MakeFixture "${BASE}"
"${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${BASE}" >/dev/null
ExpectFailure 'artifact is LOCKED' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" COMMISSIONING "${BASE}"

readonly LEGACY_SCHEMA="${TEST_ROOT}/legacy-schema"
cp -R "${BASE}" "${LEGACY_SCHEMA}"
sed 's/^schema=.*/schema=rrclite-firmware-build-v1/' \
  "${LEGACY_SCHEMA}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${LEGACY_SCHEMA}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv "${LEGACY_SCHEMA}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${LEGACY_SCHEMA}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'unsupported or missing build metadata schema' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${LEGACY_SCHEMA}"

readonly WRONG_ROS_DISTRO="${TEST_ROOT}/wrong-ros-distro"
cp -R "${BASE}" "${WRONG_ROS_DISTRO}"
sed 's/^ros_distro=.*/ros_distro=rolling/' \
  "${WRONG_ROS_DISTRO}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${WRONG_ROS_DISTRO}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv "${WRONG_ROS_DISTRO}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${WRONG_ROS_DISTRO}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'different ROS distribution' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${WRONG_ROS_DISTRO}"

readonly WRONG_RELEASE_CLASS="${TEST_ROOT}/wrong-release-class"
cp -R "${BASE}" "${WRONG_RELEASE_CLASS}"
sed 's/^release_qualified=.*/release_qualified=1/' \
  "${WRONG_RELEASE_CLASS}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${WRONG_RELEASE_CLASS}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv "${WRONG_RELEASE_CLASS}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${WRONG_RELEASE_CLASS}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'classify the artifact as non-release' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED \
  "${WRONG_RELEASE_CLASS}"

readonly LOCKED_METADATA_ACK="${TEST_ROOT}/locked-metadata-ack"
cp -R "${BASE}" "${LOCKED_METADATA_ACK}"
sed 's/^commissioning_ack=$/commissioning_ack=MOTORS_RAISED/' \
  "${LOCKED_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${LOCKED_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv "${LOCKED_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${LOCKED_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'locked build metadata contains a commissioning acknowledgement' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED \
  "${LOCKED_METADATA_ACK}"

readonly COMMISSIONING_BASE="${TEST_ROOT}/commissioning-base"
cp -R "${BASE}" "${COMMISSIONING_BASE}"
printf '%s\n' \
  'RRCLITE_MOTOR_COMMISSIONING:BOOL=ON' \
  'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=MOTORS_RAISED' \
  >"${COMMISSIONING_BASE}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
sed \
  -e 's/^motor_mode=LOCKED$/motor_mode=COMMISSIONING/' \
  -e 's/^commissioning_ack=$/commissioning_ack=MOTORS_RAISED/' \
  "${COMMISSIONING_BASE}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${COMMISSIONING_BASE}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv \
  "${COMMISSIONING_BASE}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${COMMISSIONING_BASE}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
"${SCRIPT_DIR}/verify_firmware_artifact.sh" COMMISSIONING \
  "${COMMISSIONING_BASE}" >/dev/null

readonly COMMISSIONING_METADATA_ACK="${TEST_ROOT}/commissioning-metadata-ack"
cp -R "${COMMISSIONING_BASE}" "${COMMISSIONING_METADATA_ACK}"
sed 's/^commissioning_ack=.*/commissioning_ack=WRONG/' \
  "${COMMISSIONING_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt" \
  >"${COMMISSIONING_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp"
mv "${COMMISSIONING_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.tmp" \
  "${COMMISSIONING_METADATA_ACK}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
ExpectFailure 'metadata acknowledgement is missing or invalid' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" COMMISSIONING \
  "${COMMISSIONING_METADATA_ACK}"

readonly COMMISSIONING_MISSING_ACK="${TEST_ROOT}/commissioning-missing-ack"
cp -R "${COMMISSIONING_BASE}" "${COMMISSIONING_MISSING_ACK}"
printf '%s\n' 'RRCLITE_MOTOR_COMMISSIONING:BOOL=ON' \
  >"${COMMISSIONING_MISSING_ACK}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
ExpectFailure 'commissioning acknowledgement is missing' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" COMMISSIONING \
  "${COMMISSIONING_MISSING_ACK}"

readonly COMMISSIONING_WRONG_ACK="${TEST_ROOT}/commissioning-wrong-ack"
cp -R "${COMMISSIONING_BASE}" "${COMMISSIONING_WRONG_ACK}"
printf '%s\n' \
  'RRCLITE_MOTOR_COMMISSIONING:BOOL=ON' \
  'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=WRONG' \
  >"${COMMISSIONING_WRONG_ACK}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
ExpectFailure 'commissioning acknowledgement is missing' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" COMMISSIONING \
  "${COMMISSIONING_WRONG_ACK}"

readonly STALE_SOURCE="${TEST_ROOT}/stale-source"
cp -R "${BASE}" "${STALE_SOURCE}"
printf 'changed\n' \
  >>"${STALE_SOURCE}/firmware/mentor_pi_mcu/app/input.txt"
ExpectFailure 'firmware inputs changed after the build' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${STALE_SOURCE}"

readonly PROVENANCE_INPUTS=(
  tools/apply_microros_source_lock.sh
  tools/bootstrap_firmware_dependencies.sh
  tools/build_microros_library.sh
  tools/docker/microros-builder.Dockerfile
  tools/microros_artifact_fingerprint.sh
)
for provenance_input in "${PROVENANCE_INPUTS[@]}"; do
  fixture_name="${provenance_input//\//-}"
  stale_provenance="${TEST_ROOT}/stale-${fixture_name}"
  cp -R "${BASE}" "${stale_provenance}"
  printf '\nchanged provenance input\n' \
    >>"${stale_provenance}/${provenance_input}"
  ExpectFailure 'firmware inputs changed after the build' \
    "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED \
    "${stale_provenance}"
done

readonly STALE_ELF="${TEST_ROOT}/stale-elf"
cp -R "${BASE}" "${STALE_ELF}"
printf 'changed\n' \
  >>"${STALE_ELF}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
ExpectFailure 'ELF changed' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${STALE_ELF}"

readonly STALE_LIBRARY="${TEST_ROOT}/stale-library"
cp -R "${BASE}" "${STALE_LIBRARY}"
printf 'changed\n' \
  >>"${STALE_LIBRARY}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a"
ExpectFailure 'differs from the pinned reviewed artifact' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${STALE_LIBRARY}"

readonly STALE_HEADER="${TEST_ROOT}/stale-header"
cp -R "${BASE}" "${STALE_HEADER}"
printf 'changed\n' \
  >>"${STALE_HEADER}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/mentor_pi_interfaces/msg/motor_command.h"
ExpectFailure 'header/archive tree differs from the pinned artifact' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${STALE_HEADER}"

readonly WRONG_CACHE="${TEST_ROOT}/wrong-cache"
cp -R "${BASE}" "${WRONG_CACHE}"
printf '%s\n' \
  'RRCLITE_MOTOR_COMMISSIONING:BOOL=ON' \
  'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=MOTORS_RAISED' \
  >"${WRONG_CACHE}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
ExpectFailure 'CMake cache is not motor-locked' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${WRONG_CACHE}"

readonly SYMLINK_SOURCE="${TEST_ROOT}/symlink-source"
cp -R "${BASE}" "${SYMLINK_SOURCE}"
ln -s input.txt \
  "${SYMLINK_SOURCE}/firmware/mentor_pi_mcu/app/linked-input.txt"
ExpectFailure 'source symlink is unsupported' \
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${SYMLINK_SOURCE}"

echo "Firmware artifact verification tests passed."
