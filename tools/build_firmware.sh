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
readonly MEMORY_CHECKER="${PROJECT_ROOT}/tools/check_firmware_memory.sh"
readonly DEPENDENCY_BOOTSTRAP="${PROJECT_ROOT}/tools/bootstrap_firmware_dependencies.sh"
readonly BUILD_IMAGE_PREPARER="${PROJECT_ROOT}/tools/prepare_build_images.sh"
readonly JOB_SELECTOR="${PROJECT_ROOT}/tools/select_build_jobs.sh"
readonly CUBE_REPOSITORY="https://github.com/STMicroelectronics/STM32CubeF4.git"
readonly MICROROS_REPOSITORY="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
readonly EXPECTED_CUBE_COMMIT="52757b5e33259a088509a777a9e3a5b971194c7d"
readonly EXPECTED_MICROROS_COMMIT="bd531b273c1bcd070b3143c5642128ec75a6f04e"
readonly BUILD_LOCK="${PROJECT_ROOT}/tools/run_with_build_lock.sh"

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

RemoveBuildRoot() {
  [[ "${BUILD_ROOT}" == "${FIRMWARE_ROOT}/build/stm32" &&
     "${BUILD_ROOT}" != "/" ]] ||
    Fail "refusing to remove unexpected firmware build root: ${BUILD_ROOT}"
  rm -rf -- "${BUILD_ROOT}"
}

VerifyDependency() {
  local repository="$1"
  local expected_origin="$2"
  local expected_commit="$3"
  [[ -d "${repository}" ]] || \
    Fail "dependency is missing at ${repository}; run ./tools/bootstrap_firmware_dependencies.sh"
  [[ -x "${DEPENDENCY_BOOTSTRAP}" ]] || \
    Fail "firmware dependency verifier is missing: ${DEPENDENCY_BOOTSTRAP}"
  "${DEPENDENCY_BOOTSTRAP}" --verify-existing \
    "${expected_origin}" "${expected_commit}" "${repository}" || \
    Fail "dependency provenance verification failed at ${repository}"
}

if [[ "$#" -gt 1 || ("$#" -eq 1 && "$1" != "--print-motor-profile") ]]; then
  Fail "usage: ./tools/build_firmware.sh [--print-motor-profile]"
fi

if [[ "${1:-}" == "--print-motor-profile" ]]; then
  printf '%s\n' \
    'mode=PID' \
    'control_mode=CLOSED_LOOP' \
    'maximum_accepted_rps=6.0' \
    'output_limit_permille=1000' \
    'release_qualified=0'
  exit 0
fi

if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi

VerifyDependency "${CUBE_ROOT}" "${CUBE_REPOSITORY}" \
  "${EXPECTED_CUBE_COMMIT}"
VerifyDependency "${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils" \
  "${MICROROS_REPOSITORY}" "${EXPECTED_MICROROS_COMMIT}"
RequireFile "${FINGERPRINT_TOOL}" "Restore the firmware fingerprint tool."
RequireFile "${MICROROS_FINGERPRINT_TOOL}" \
  "Restore the micro-ROS artifact fingerprint tool."
RequireFile "${ARTIFACT_VERIFIER}" "Restore the artifact verifier."
RequireFile "${MEMORY_CHECKER}" "Restore the firmware memory checker."
RequireFile "${MICROROS_ARCHIVE_HASH}" \
  "Restore the pinned micro-ROS archive hash."
RequireFile "${MICROROS_TREE_HASH}" \
  "Restore the pinned micro-ROS archive/header-tree hash."
RequireFile "${MICROROS_ROOT}/libmicroros.a" \
  "Run ./tools/build_microros_library.sh."
RequireFile "${MICROROS_ROOT}/ros_distro" \
  "Run ./tools/build_microros_library.sh for ROS 2 Humble."
[[ "$(tr -d '[:space:]' <"${MICROROS_ROOT}/ros_distro")" == "humble" ]] ||
  Fail "generated micro-ROS library targets a different ROS distribution"
RequireFile "${MICROROS_INTERFACE_FINGERPRINT}" \
  "Run ./tools/build_microros_library.sh to bind it to current interfaces."
RequireFile \
  "${MICROROS_ROOT}/include/mentor_pi_interfaces/msg/motor_command.h" \
  "Regenerate libmicroros with mentor_pi_interfaces."
RequireFile "${TARGET_ROOT}/main.cc" \
  "Restore the STM32 target entry point."
RequireFile "${DOCKERFILE}" "Restore the reproducible builder definition."
[[ -x "${BUILD_IMAGE_PREPARER}" && -x "${JOB_SELECTOR}" ]] || \
  Fail "Docker build-image or job-selection tooling is unavailable"

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

case "$(uname -m)" in
  x86_64 | amd64) readonly architecture=amd64 ;;
  aarch64 | arm64) readonly architecture=arm64 ;;
  *) Fail "the host architecture must be amd64 or arm64" ;;
esac
readonly IMAGE="$("${BUILD_IMAGE_PREPARER}" --architecture "${architecture}" --print firmware)"
readonly IMAGE_ID="$(docker image inspect "${IMAGE}" --format '{{.Id}}' \
  2>/dev/null || true)"
[[ "${IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "the firmware builder image identity is invalid; run make setup"
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"
command -v docker >/dev/null || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker Engine is not running"
"${BUILD_IMAGE_PREPARER}" --architecture "${architecture}"
RemoveBuildRoot
docker run --rm \
  --platform "linux/${architecture}" \
  --user "$(id -u):$(id -g)" \
  --env SOURCE_DATE_EPOCH=0 \
  --env "CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}" \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  bash -euc '
      cmake -S firmware/mentor_pi_mcu/target/stm32 \
        -B firmware/mentor_pi_mcu/build/stm32 \
        -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/workspace/firmware/mentor_pi_mcu/target/stm32/arm-none-eabi-toolchain.cmake \
        -DCMAKE_BUILD_TYPE=MinSizeRel
      cmake --build firmware/mentor_pi_mcu/build/stm32 \
        --parallel "${RRCLITE_BUILD_JOBS}"
    '

for artifact in mentor_pi_mcu.elf mentor_pi_mcu.hex mentor_pi_mcu.bin \
    mentor_pi_mcu.map; do
  RequireFile "${BUILD_ROOT}/${artifact}" "The target build did not complete."
done

VerifyDependency "${CUBE_ROOT}" "${CUBE_REPOSITORY}" \
  "${EXPECTED_CUBE_COMMIT}"
VerifyDependency "${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils" \
  "${MICROROS_REPOSITORY}" "${EXPECTED_MICROROS_COMMIT}"
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
  RemoveBuildRoot
  Fail "micro-ROS generated inputs changed during the firmware build"
fi

readonly SOURCE_FINGERPRINT_AFTER="$(
  "${FINGERPRINT_TOOL}" firmware "${PROJECT_ROOT}"
)"
if [[ "${SOURCE_FINGERPRINT_BEFORE}" != "${SOURCE_FINGERPRINT_AFTER}" ]]; then
  RemoveBuildRoot
  Fail "project-owned firmware inputs changed during the build"
fi

readonly MOTOR_MODE="PID"
readonly CONTROL_MODE="CLOSED_LOOP"
readonly ARTIFACT_MODE="NORMAL"
readonly VERIFICATION_MODE="PID"
readonly METADATA="${BUILD_ROOT}/rrclite-build-metadata.txt"
readonly METADATA_TEMP="${METADATA}.tmp"
printf '%s\n' \
  'schema=rrclite-firmware-build-v2' \
  'target=STM32F407VET6' \
  'ros_distro=humble' \
  'builder_mode=docker-pinned' \
  "builder_image_id=${IMAGE_ID}" \
  "motor_mode=${MOTOR_MODE}" \
  "control_mode=${CONTROL_MODE}" \
  "artifact_mode=${ARTIFACT_MODE}" \
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

"${ARTIFACT_VERIFIER}" "${VERIFICATION_MODE}" "${PROJECT_ROOT}" >/dev/null
"${MEMORY_CHECKER}" "${VERIFICATION_MODE}" "${PROJECT_ROOT}"

echo "Firmware artifacts: ${BUILD_ROOT}"
