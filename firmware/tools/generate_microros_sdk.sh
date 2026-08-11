#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly MCU_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu"
readonly REPOSITORY_ROOT="$(cd "${FIRMWARE_ROOT}/.." && pwd)"
readonly INTERFACE_ROOT="${REPOSITORY_ROOT}/ros2_ws/src/mentor_pi_interfaces"
readonly SOURCE_UTILS="${MCU_ROOT}/third_party/micro_ros_stm32cubemx_utils"
readonly BUILD_ROOT="${MCU_ROOT}/build/microros-generate"
readonly BUILD_UTILS="${BUILD_ROOT}/micro_ros_stm32cubemx_utils"
readonly LIBRARY_ROOT="${BUILD_UTILS}/microros_static_library_ide/libmicroros"
readonly ARCHIVE="${MCU_ROOT}/sdk/humble/libmicroros.tar.xz"
readonly MANIFEST="${MCU_ROOT}/sdk/humble/manifest.txt"
readonly JOBS="${RRCLITE_BUILD_JOBS:-1}"
readonly GEOMETRY2_COMMIT="65c620c308920f558c7b5d3fb852941bd6d8fced"
readonly LIBYAML_REPOSITORY="https://github.com/yaml/libyaml"
readonly LIBYAML_COMMIT="2c891fc7a770e8ba2fec34fc6b545c672beb37e6"

Fail() { echo "micro-ROS SDK generation error: $*" >&2; exit 1; }
[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || Fail "build job count must be positive"
((EUID != 0)) || Fail "run this target as the normal developer user, not with sudo"
"${SCRIPT_DIR}/check_environment.sh" >/dev/null
"${SCRIPT_DIR}/setup.sh" --verify >/dev/null
[[ -r /opt/ros/humble/setup.bash ]] || \
  Fail "ROS 2 Humble is required; use the VS Code Dev Container"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# micro_ros_setup is supplied by the official static-library builder as an
# overlay rather than an apt package.
if [[ -r /uros_ws/install/local_setup.bash ]]; then
  # shellcheck disable=SC1091
  source /uros_ws/install/local_setup.bash
fi
set -u
export ROS_DISTRO=humble
ros2 pkg prefix micro_ros_setup >/dev/null 2>&1 || \
  Fail "a Humble micro_ros_setup overlay is required; use the VS Code Dev Container"
command -v vcs >/dev/null 2>&1 || Fail "python3-vcstool is required"
command -v rosdep >/dev/null 2>&1 || Fail "python3-rosdep is required"
rosdep db >/dev/null 2>&1 || \
  Fail "the rosdep cache is unavailable; rebuild the Dev Container or run rosdep update"

for root in "${ARM_GNU_TOOLCHAIN_ROOT:-}" /opt/arm-gnu-toolchain \
    "${MCU_ROOT}/.deps/arm-gnu-toolchain"; do
  if [[ -n "${root}" && -x "${root}/bin/arm-none-eabi-gcc" && \
      "$("${root}/bin/arm-none-eabi-gcc" -dumpfullversion)" == 13.2.1 ]]; then
    toolchain_root="${root}"
    break
  fi
done
[[ -n "${toolchain_root:-}" ]] || Fail "Arm GNU 13.2.1 is unavailable"
readonly toolchain_root
export PATH="${toolchain_root}/bin:${PATH}"

if [[ -d "${BUILD_ROOT}" ]] && \
    find "${BUILD_ROOT}" -type d ! -writable -print -quit | grep -q .; then
  Fail "the failed generation tree is not writable; remove it once with: sudo rm -rf -- ${BUILD_ROOT}"
fi
rm -rf -- "${BUILD_ROOT}"
mkdir -p "${BUILD_UTILS}"
cp -a "${SOURCE_UTILS}/." "${BUILD_UTILS}/"
rm -rf -- "${BUILD_UTILS}/.git"
extra_packages="${BUILD_ROOT}/microros_component/extra_packages"
mkdir -p "${extra_packages}/mentor_pi_interfaces"
cp -a "${INTERFACE_ROOT}/." "${extra_packages}/mentor_pi_interfaces/"
cp "${MCU_ROOT}/config/microros_colcon.meta" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/colcon.meta"
cp "${MCU_ROOT}/config/microros_toolchain.cmake" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/toolchain.cmake"
cp "${MCU_ROOT}/config/microros_library_generation.sh" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/library_generation.sh"

export COLCON_DEFAULTS_FILE="${BUILD_ROOT}/colcon-defaults.yaml"
printf 'build:\n  executor: parallel\n  parallel-workers: %s\n' "${JOBS}" \
  >"${COLCON_DEFAULTS_FILE}"
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
export MICROROS_LIBRARY_FOLDER=build/microros-generate/micro_ros_stm32cubemx_utils/microros_static_library_ide
export STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_microxrcedds_c
export MICROROS_GEOMETRY2_COMMIT="${GEOMETRY2_COMMIT}"
export MICROROS_LIBYAML_REPOSITORY="${LIBYAML_REPOSITORY}"
export MICROROS_LIBYAML_COMMIT="${LIBYAML_COMMIT}"
export MICROROS_CAPTURE_SOURCE_LOCK=0
export MICROROS_CALLER_UID="$(id -u)"
export MICROROS_CALLER_GID="$(id -g)"
export MICROROS_PROJECT_ROOT="${MCU_ROOT}"
export MICROROS_GENERATOR_WORKSPACE="${BUILD_ROOT}/workspace"
export MICROROS_TOOLCHAIN_ROOT="${toolchain_root}/bin"
export MICROROS_TOOLS_ROOT="${FIRMWARE_ROOT}/tools"
export MICROROS_RESTORE_OWNERSHIP=0
export MICROROS_SETUP_OVERLAY=""
export PYTHONHASHSEED=0

bash "${BUILD_UTILS}/microros_static_library_ide/library_generation/library_generation.sh"
[[ -s "${LIBRARY_ROOT}/libmicroros.a" && \
   -f "${LIBRARY_ROOT}/include/mentor_pi_interfaces/motor_profile_contract.hpp" && \
   "$(tr -d '[:space:]' <"${LIBRARY_ROOT}/ros_distro")" == humble ]] || \
  Fail "generated SDK is incomplete"
interface_sha="$("${SCRIPT_DIR}/interface_fingerprint.sh")"
rm -f -- "${LIBRARY_ROOT}/MENTOR-PI-CACHE.txt"
printf '%s\n' "${interface_sha}" \
  >"${LIBRARY_ROOT}/mentor_pi_interfaces.source.sha256"
printf '%s\n' \
  'format=mentor-pi-firmware-sdk-tree-v1' \
  'ros_distro=humble' \
  'toolchain=arm-gnu-toolchain-13.2.rel1' \
  'toolchain_amd64_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb' \
  'toolchain_arm64_sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a' \
  "interfaces_sha256=${interface_sha}" \
  "source_lock_sha256=$("${SCRIPT_DIR}/sha256.sh" "${MCU_ROOT}/config/microros_sources.lock")" \
  >"${LIBRARY_ROOT}/SDK-METADATA.txt"
tree_sha="$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${LIBRARY_ROOT}")"

mkdir -p "$(dirname "${ARCHIVE}")"
temporary_archive="${ARCHIVE}.tmp.$$"
trap 'rm -f -- "${temporary_archive}"' EXIT
tar --sort=name --mtime=@0 --owner=0 --group=0 --numeric-owner \
  -cJf "${temporary_archive}" -C "${LIBRARY_ROOT}" .
archive_sha="$("${SCRIPT_DIR}/sha256.sh" "${temporary_archive}")"
mv "${temporary_archive}" "${ARCHIVE}"
trap - EXIT
printf '%s\n' \
  'format=mentor-pi-firmware-sdk-v1' \
  'ros_distro=humble' \
  'toolchain=arm-gnu-toolchain-13.2.rel1' \
  'toolchain_amd64_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb' \
  'toolchain_arm64_sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a' \
  "interfaces_sha256=${interface_sha}" \
  "archive_sha256=${archive_sha}" \
  "tree_sha256=${tree_sha}" \
  "source_lock_sha256=$("${SCRIPT_DIR}/sha256.sh" "${MCU_ROOT}/config/microros_sources.lock")" \
  >"${MANIFEST}"
"${SCRIPT_DIR}/extract_microros_sdk.sh" >/dev/null
echo "Regenerated checked Humble firmware SDK: ${ARCHIVE}"
