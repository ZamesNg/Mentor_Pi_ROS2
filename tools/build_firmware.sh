#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FIRMWARE_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu"
readonly TARGET_ROOT="${FIRMWARE_ROOT}/target/stm32"
readonly BUILD_ROOT="${FIRMWARE_ROOT}/build/stm32"
readonly CUBE_ROOT="${FIRMWARE_ROOT}/third_party/stm32cube_f4"
readonly MICROROS_ROOT="${FIRMWARE_ROOT}/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros"
readonly MICROROS_INTERFACE_FINGERPRINT="${MICROROS_ROOT}/mentor_pi_interfaces.source.sha256"
readonly MICROROS_ARCHIVE_HASH="${FIRMWARE_ROOT}/config/microros_artifact.sha256"
readonly MICROROS_TREE_HASH="${FIRMWARE_ROOT}/config/microros_artifact_tree.sha256"
readonly TOOLCHAIN_FILE="${TARGET_ROOT}/arm-none-eabi-toolchain.cmake"
readonly DOCKERFILE="${PROJECT_ROOT}/tools/docker/firmware-builder.Dockerfile"
readonly FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/firmware_source_fingerprint.sh"
readonly MICROROS_FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/microros_artifact_fingerprint.sh"
readonly ARTIFACT_VERIFIER="${PROJECT_ROOT}/tools/verify_firmware_artifact.sh"
readonly IMAGE="mentor-pi/rrclite-firmware-builder:gcc-13.2.1"
readonly EXPECTED_CUBE_COMMIT="52757b5e33259a088509a777a9e3a5b971194c7d"
readonly EXPECTED_MICROROS_COMMIT="a5b2127495ae0ab53d7a1360beaf17822309a3cc"
readonly COMMISSIONING_MAXIMUM_RPS="0.25"
readonly COMMISSIONING_OUTPUT_LIMIT_PERMILLE="300"

Fail() {
  echo "Firmware build error: $*" >&2
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

RequireFile() {
  local required_file="$1"
  local recovery_hint="$2"
  [[ -f "${required_file}" ]] || Fail "${required_file} is missing. ${recovery_hint}"
}

VerifyCommit() {
  local repository="$1"
  local expected_commit="$2"
  command -v git >/dev/null 2>&1 || Fail "git is not installed"
  [[ -d "${repository}" ]] || \
    Fail "dependency is missing at ${repository}; run ./tools/bootstrap_firmware_dependencies.sh"
  local actual_commit
  actual_commit="$(git -C "${repository}" rev-parse --verify HEAD 2>/dev/null)" || \
    Fail "dependency at ${repository} is not a Git worktree"
  [[ "${actual_commit}" == "${expected_commit}" ]] || \
    Fail "dependency mismatch at ${repository}; expected ${expected_commit}, got ${actual_commit}"
  local worktree_status
  worktree_status="$(
    git -C "${repository}" status --porcelain=v1 --untracked-files=all
  )"
  [[ -z "${worktree_status}" ]] || \
    Fail "dependency worktree is dirty at ${repository}; restore or re-bootstrap it before building"
}

if [[ "$#" -gt 1 || ("$#" -eq 1 && "$1" != "--print-motor-profile") ]]; then
  Fail "usage: ./tools/build_firmware.sh [--print-motor-profile]"
fi

case "${RRCLITE_MOTOR_COMMISSIONING:-0}" in
  0)
    readonly MOTOR_COMMISSIONING_OPTION="OFF"
    readonly MOTOR_COMMISSIONING_ACK=""
    ;;
  1)
    [[ "${RRCLITE_MOTOR_COMMISSIONING_ACK:-}" == "MOTORS_RAISED" ]] || \
      Fail "commissioning requires RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED after raising all wheels and using a current-limited supply"
    readonly MOTOR_COMMISSIONING_OPTION="ON"
    readonly MOTOR_COMMISSIONING_ACK="MOTORS_RAISED"
    ;;
  *)
    Fail "RRCLITE_MOTOR_COMMISSIONING must be 0 or 1"
    ;;
esac

if [[ "${1:-}" == "--print-motor-profile" ]]; then
  if [[ "${MOTOR_COMMISSIONING_OPTION}" == "ON" ]]; then
    printf '%s\n' \
      'mode=COMMISSIONING' \
      'closed_loop_enabled=1' \
      "maximum_accepted_rps=${COMMISSIONING_MAXIMUM_RPS}" \
      "output_limit_permille=${COMMISSIONING_OUTPUT_LIMIT_PERMILLE}" \
      'release_qualified=0'
  else
    printf '%s\n' \
      'mode=LOCKED' \
      'closed_loop_enabled=0' \
      'maximum_accepted_rps=0.0' \
      'output_limit_permille=0' \
      'release_qualified=0'
  fi
  exit 0
fi

VerifyCommit "${CUBE_ROOT}" "${EXPECTED_CUBE_COMMIT}"
VerifyCommit "${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils" \
  "${EXPECTED_MICROROS_COMMIT}"
RequireFile "${FINGERPRINT_TOOL}" "Restore the firmware fingerprint tool."
RequireFile "${MICROROS_FINGERPRINT_TOOL}" \
  "Restore the micro-ROS artifact fingerprint tool."
RequireFile "${ARTIFACT_VERIFIER}" "Restore the artifact verifier."
RequireFile "${MICROROS_ARCHIVE_HASH}" \
  "Restore the pinned micro-ROS archive hash."
RequireFile "${MICROROS_TREE_HASH}" \
  "Restore the pinned micro-ROS archive/header-tree hash."
RequireFile "${MICROROS_ROOT}/libmicroros.a" \
  "Run ./tools/build_microros_library.sh."
RequireFile "${MICROROS_INTERFACE_FINGERPRINT}" \
  "Run ./tools/build_microros_library.sh to bind it to current interfaces."
RequireFile \
  "${MICROROS_ROOT}/include/mentor_pi_interfaces/msg/motor_command.h" \
  "Regenerate libmicroros with mentor_pi_interfaces."
RequireFile "${TARGET_ROOT}/main.cc" \
  "Restore the STM32 target entry point."
RequireFile "${DOCKERFILE}" "Restore the reproducible builder definition."

readonly EXPECTED_MICROROS_ARCHIVE_FINGERPRINT="$(tr -d '[:space:]' \
  <"${MICROROS_ARCHIVE_HASH}")"
readonly MICROROS_ARCHIVE_FINGERPRINT_BEFORE="$(
  Sha256 "${MICROROS_ROOT}/libmicroros.a"
)"
[[ "${MICROROS_ARCHIVE_FINGERPRINT_BEFORE}" == \
    "${EXPECTED_MICROROS_ARCHIVE_FINGERPRINT}" ]] || \
  Fail "libmicroros.a differs from the pinned reviewed artifact; run ./tools/build_microros_library.sh"
readonly EXPECTED_MICROROS_TREE_FINGERPRINT="$(tr -d '[:space:]' \
  <"${MICROROS_TREE_HASH}")"
readonly MICROROS_TREE_FINGERPRINT_BEFORE="$(
  "${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}"
)"
[[ "${MICROROS_TREE_FINGERPRINT_BEFORE}" == \
    "${EXPECTED_MICROROS_TREE_FINGERPRINT}" ]] || \
  Fail "generated micro-ROS headers/archive differ from the pinned reviewed tree; run ./tools/build_microros_library.sh"

readonly INTERFACE_FINGERPRINT="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"
readonly LIBRARY_INTERFACE_FINGERPRINT="$(tr -d '[:space:]' \
  <"${MICROROS_INTERFACE_FINGERPRINT}")"
[[ "${INTERFACE_FINGERPRINT}" == "${LIBRARY_INTERFACE_FINGERPRINT}" ]] || \
  Fail "libmicroros was generated from different mentor_pi_interfaces; run ./tools/build_microros_library.sh"
readonly SOURCE_FINGERPRINT_BEFORE="$(
  "${FINGERPRINT_TOOL}" firmware "${PROJECT_ROOT}"
)"

if [[ "${RRCLITE_BUILD_LOCAL:-0}" == "1" ]]; then
  command -v cmake >/dev/null || Fail "cmake is not installed"
  command -v ninja >/dev/null || Fail "ninja is not installed"
  command -v arm-none-eabi-gcc >/dev/null || \
    Fail "arm-none-eabi-gcc is not installed"
  command -v arm-none-eabi-g++ >/dev/null || \
    Fail "arm-none-eabi-g++ is not installed"
  [[ "$(arm-none-eabi-gcc -dumpfullversion)" == "13.2.1" ]] || \
    Fail "local arm-none-eabi-gcc must be the pinned 13.2.1 release"
  [[ "$(arm-none-eabi-g++ -dumpfullversion)" == "13.2.1" ]] || \
    Fail "local arm-none-eabi-g++ must be the pinned 13.2.1 release"
  cmake -E remove_directory "${BUILD_ROOT}"
  cmake -S "${TARGET_ROOT}" -B "${BUILD_ROOT}" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DRRCLITE_MOTOR_COMMISSIONING="${MOTOR_COMMISSIONING_OPTION}" \
    -DRRCLITE_MOTOR_COMMISSIONING_ACK="${MOTOR_COMMISSIONING_ACK}"
  cmake --build "${BUILD_ROOT}" --parallel
else
  command -v docker >/dev/null || Fail "Docker is not installed"
  docker info >/dev/null 2>&1 || Fail "Docker Desktop/Engine is not running"
  docker build --file "${DOCKERFILE}" --tag "${IMAGE}" \
    "${PROJECT_ROOT}/tools/docker"
  cmake -E remove_directory "${BUILD_ROOT}"
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    --env SOURCE_DATE_EPOCH=0 \
    --env RRCLITE_CMAKE_MOTOR_OPTION="${MOTOR_COMMISSIONING_OPTION}" \
    --env RRCLITE_CMAKE_MOTOR_ACK="${MOTOR_COMMISSIONING_ACK}" \
    --volume "${PROJECT_ROOT}:/workspace" \
    --workdir /workspace \
    "${IMAGE}" \
    bash -euc '
      cmake -S firmware/mentor_pi_mcu/target/stm32 \
        -B firmware/mentor_pi_mcu/build/stm32 \
        -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/workspace/firmware/mentor_pi_mcu/target/stm32/arm-none-eabi-toolchain.cmake \
        -DCMAKE_BUILD_TYPE=MinSizeRel \
        -DRRCLITE_MOTOR_COMMISSIONING="${RRCLITE_CMAKE_MOTOR_OPTION}" \
        -DRRCLITE_MOTOR_COMMISSIONING_ACK="${RRCLITE_CMAKE_MOTOR_ACK}"
      cmake --build firmware/mentor_pi_mcu/build/stm32 --parallel
    '
fi

for artifact in mentor_pi_mcu.elf mentor_pi_mcu.hex mentor_pi_mcu.bin \
    mentor_pi_mcu.map; do
  RequireFile "${BUILD_ROOT}/${artifact}" "The target build did not complete."
done

VerifyCommit "${CUBE_ROOT}" "${EXPECTED_CUBE_COMMIT}"
VerifyCommit "${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils" \
  "${EXPECTED_MICROROS_COMMIT}"
readonly MICROROS_ARCHIVE_FINGERPRINT_AFTER="$(
  Sha256 "${MICROROS_ROOT}/libmicroros.a"
)"
readonly MICROROS_TREE_FINGERPRINT_AFTER="$(
  "${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}"
)"
if [[ "${MICROROS_ARCHIVE_FINGERPRINT_BEFORE}" != \
      "${MICROROS_ARCHIVE_FINGERPRINT_AFTER}" || \
    "${MICROROS_TREE_FINGERPRINT_BEFORE}" != \
      "${MICROROS_TREE_FINGERPRINT_AFTER}" ]]; then
  cmake -E remove_directory "${BUILD_ROOT}"
  Fail "micro-ROS generated inputs changed during the firmware build"
fi

readonly SOURCE_FINGERPRINT_AFTER="$(
  "${FINGERPRINT_TOOL}" firmware "${PROJECT_ROOT}"
)"
if [[ "${SOURCE_FINGERPRINT_BEFORE}" != "${SOURCE_FINGERPRINT_AFTER}" ]]; then
  cmake -E remove_directory "${BUILD_ROOT}"
  Fail "project-owned firmware inputs changed during the build"
fi

if [[ "${MOTOR_COMMISSIONING_OPTION}" == "ON" ]]; then
  readonly MOTOR_MODE="COMMISSIONING"
else
  readonly MOTOR_MODE="LOCKED"
fi
readonly METADATA="${BUILD_ROOT}/rrclite-build-metadata.txt"
readonly METADATA_TEMP="${METADATA}.tmp"
printf '%s\n' \
  'schema=rrclite-firmware-build-v1' \
  'target=STM32F407VET6' \
  "motor_mode=${MOTOR_MODE}" \
  "commissioning_ack=${MOTOR_COMMISSIONING_ACK}" \
  'release_qualified=0' \
  "source_sha256=${SOURCE_FINGERPRINT_AFTER}" \
  "interfaces_sha256=${INTERFACE_FINGERPRINT}" \
  "microros_archive_sha256=${MICROROS_ARCHIVE_FINGERPRINT_AFTER}" \
  "microros_tree_sha256=${MICROROS_TREE_FINGERPRINT_AFTER}" \
  "elf_sha256=$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.elf")" \
  "hex_sha256=$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.hex")" \
  "bin_sha256=$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.bin")" \
  "map_sha256=$(Sha256 "${BUILD_ROOT}/mentor_pi_mcu.map")" \
  >"${METADATA_TEMP}"
mv "${METADATA_TEMP}" "${METADATA}"

"${ARTIFACT_VERIFIER}" "${MOTOR_MODE}" "${PROJECT_ROOT}" >/dev/null

echo "Firmware artifacts: ${BUILD_ROOT}"
