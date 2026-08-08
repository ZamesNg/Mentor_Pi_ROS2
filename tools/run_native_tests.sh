#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

declare -a generator_args=()
if command -v ninja >/dev/null 2>&1; then
  generator_args=(-G Ninja)
fi

run_cmake_suite() {
  local name="$1"
  local source_directory="$2"
  shift 2
  local build_directory="${PROJECT_ROOT}/build/${name}"

  cmake -E echo "Configuring ${name}"
  cmake -S "${source_directory}" -B "${build_directory}" \
    "${generator_args[@]}" -DBUILD_TESTING=ON "$@"
  cmake --build "${build_directory}" --parallel
  ctest --test-dir "${build_directory}" --output-on-failure
}

python3 -m unittest discover \
  -s "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces/test" -v

run_cmake_suite \
  mentor_pi_mcu-native \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu"
run_cmake_suite \
  mentor_pi_mcu-drivers \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/drivers"
run_cmake_suite \
  mentor_pi_mcu-controller \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/controller"
run_cmake_suite \
  mentor_pi_microros-native \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/microros"
run_cmake_suite \
  mentor_pi_bringup-native \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup" \
  -DMENTOR_PI_BUILD_ROS2=OFF

cmake -E echo "All native RRCLite v2 suites passed"
