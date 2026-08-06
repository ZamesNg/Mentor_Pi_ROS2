#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FIRMWARE_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu"
readonly INTERFACE_ROOT="${PROJECT_ROOT}/src/mentor_pi_interfaces"
readonly SOURCE_UTILS="${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils"
readonly BUILD_ROOT="${FIRMWARE_ROOT}/build/microros"
readonly BUILD_UTILS="${BUILD_ROOT}/micro_ros_stm32cubemx_utils"
readonly IMAGE="mentor-pi/micro-ros-static-library-builder:jazzy-arm64"
readonly DOCKERFILE="${PROJECT_ROOT}/tools/docker/microros-builder.Dockerfile"
readonly SOURCE_LOCK="${FIRMWARE_ROOT}/config/microros_sources.lock"
readonly ARTIFACT_HASH="${FIRMWARE_ROOT}/config/microros_artifact.sha256"
readonly ARTIFACT_TREE_HASH="${FIRMWARE_ROOT}/config/microros_artifact_tree.sha256"
readonly FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/firmware_source_fingerprint.sh"
readonly MICROROS_FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/microros_artifact_fingerprint.sh"
readonly GEOMETRY2_COMMIT="62335b1e1506785a283ba121f451bb962e9b6db3"

Sha256() {
  if command -v sha256sum >/dev/null; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "Neither sha256sum nor shasum is installed." >&2
    return 1
  fi
}

if [[ ! -d "${SOURCE_UTILS}" ]]; then
  echo "Run tools/bootstrap_firmware_dependencies.sh first." >&2
  exit 1
fi
if [[ ! -f "${INTERFACE_ROOT}/package.xml" ]]; then
  echo "mentor_pi_interfaces is missing: ${INTERFACE_ROOT}" >&2
  exit 1
fi
if [[ ! -f "${SOURCE_LOCK}" || ! -f "${ARTIFACT_HASH}" || \
    ! -f "${ARTIFACT_TREE_HASH}" ]]; then
  echo "micro-ROS source/artifact lock is missing under config/." >&2
  exit 1
fi
if [[ ! -x "${FINGERPRINT_TOOL}" || \
    ! -x "${MICROROS_FINGERPRINT_TOOL}" ]]; then
  echo "Firmware artifact fingerprint tooling is missing or not executable." >&2
  exit 1
fi
readonly INTERFACE_FINGERPRINT_BEFORE="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"

cmake -E remove_directory "${BUILD_ROOT}"
cmake -E make_directory "${BUILD_ROOT}"
cmake -E copy_directory "${SOURCE_UTILS}" "${BUILD_UTILS}"

# The upstream generator resolves custom packages from BASE_PATH/../../, where
# BASE_PATH is BUILD_UTILS/microros_static_library_ide.  Therefore the
# microros_component directory must be a sibling of BUILD_UTILS, not a child.
readonly EXTRA_PACKAGES="${BUILD_ROOT}/microros_component/extra_packages"
cmake -E make_directory "${EXTRA_PACKAGES}"
cmake -E copy_directory "${INTERFACE_ROOT}" \
  "${EXTRA_PACKAGES}/mentor_pi_interfaces"
cmake -E copy \
  "${FIRMWARE_ROOT}/config/microros_colcon.meta" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/colcon.meta"
cmake -E copy \
  "${FIRMWARE_ROOT}/config/microros_toolchain.cmake" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/toolchain.cmake"
cmake -E copy \
  "${FIRMWARE_ROOT}/config/microros_library_generation.sh" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/library_generation.sh"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker build --file "${DOCKERFILE}" --tag "${IMAGE}" \
    "${PROJECT_ROOT}/tools/docker"
fi

# Jazzy's generic C dispatcher otherwise retains every installed backend,
# including desktop introspection tables. Select the sole MCU backend at
# generation time. The project-owned toolchain also enables
# rosidl_generator_c's supported description-codegen reduction globally: it
# preserves type hashes and emits explicit empty description callbacks without
# allocating the source/field tables that the MCU never serves.
docker run --rm \
  --volume "${FIRMWARE_ROOT}:/project" \
  --volume "${PROJECT_ROOT}/tools:/rrclite_tools:ro" \
  --env MICROROS_LIBRARY_FOLDER=build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide \
  --env STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_microxrcedds_c \
  --env MICROROS_GEOMETRY2_COMMIT="${GEOMETRY2_COMMIT}" \
  --env PYTHONHASHSEED=0 \
  "${IMAGE}"

readonly LIBRARY_DIR="${BUILD_UTILS}/microros_static_library_ide/libmicroros"
test -f "${LIBRARY_DIR}/libmicroros.a"
test -d "${LIBRARY_DIR}/include"
test -f \
  "${LIBRARY_DIR}/include/mentor_pi_interfaces/msg/motor_command.h"
test -f \
  "${LIBRARY_DIR}/include/mentor_pi_interfaces/srv/set_motor_model.h"
grep -q '^mentor_pi_interfaces/MotorCommand.msg$' \
  "${LIBRARY_DIR}/available_ros2_types"
grep -q '^mentor_pi_interfaces/SetMotorModel.srv$' \
  "${LIBRARY_DIR}/available_ros2_types"
readonly NORMALIZED_EXPECTED="$(mktemp)"
readonly NORMALIZED_ACTUAL="$(mktemp)"
trap 'rm -f "${NORMALIZED_EXPECTED}" "${NORMALIZED_ACTUAL}"' EXIT
sed '/^[[:space:]]*#/d; /^[[:space:]]*$/d' "${SOURCE_LOCK}" | sort \
  >"${NORMALIZED_EXPECTED}"
sed '/^[[:space:]]*$/d; s#\.git # #; s#/$##' \
  "${LIBRARY_DIR}/built_packages" | sort >"${NORMALIZED_ACTUAL}"
if ! diff -u "${NORMALIZED_EXPECTED}" "${NORMALIZED_ACTUAL}"; then
  echo "Generated micro-ROS package revisions differ from the lock." >&2
  exit 1
fi
readonly EXPECTED_ARCHIVE_HASH="$(tr -d '[:space:]' <"${ARTIFACT_HASH}")"
readonly ACTUAL_ARCHIVE_HASH="$(Sha256 "${LIBRARY_DIR}/libmicroros.a")"
if [[ "${ACTUAL_ARCHIVE_HASH}" != "${EXPECTED_ARCHIVE_HASH}" ]]; then
  echo "Generated libmicroros.a hash differs from the reviewed artifact." >&2
  echo "Expected: ${EXPECTED_ARCHIVE_HASH}" >&2
  echo "Actual:   ${ACTUAL_ARCHIVE_HASH}" >&2
  exit 1
fi
readonly EXPECTED_TREE_HASH="$(tr -d '[:space:]' \
  <"${ARTIFACT_TREE_HASH}")"
readonly ACTUAL_TREE_HASH="$(
  "${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}"
)"
if [[ "${ACTUAL_TREE_HASH}" != "${EXPECTED_TREE_HASH}" ]]; then
  echo "Generated micro-ROS archive/header tree differs from the reviewed artifact." >&2
  echo "Expected: ${EXPECTED_TREE_HASH}" >&2
  echo "Actual:   ${ACTUAL_TREE_HASH}" >&2
  exit 1
fi
readonly INTERFACE_FINGERPRINT_AFTER="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"
if [[ "${INTERFACE_FINGERPRINT_BEFORE}" != \
    "${INTERFACE_FINGERPRINT_AFTER}" ]]; then
  echo "mentor_pi_interfaces changed while the library was generated." >&2
  exit 1
fi
printf '%s\n' "${INTERFACE_FINGERPRINT_AFTER}" \
  >"${LIBRARY_DIR}/mentor_pi_interfaces.source.sha256"
echo "Generated Jazzy micro-ROS library: ${LIBRARY_DIR}"
