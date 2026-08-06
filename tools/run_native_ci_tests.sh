#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

build_type="Debug"
sanitizers="on"
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --build-type)
      [[ "$#" -ge 2 ]] || {
        echo "--build-type requires Debug or Release" >&2
        exit 2
      }
      build_type="$2"
      shift 2
      ;;
    --sanitizers)
      [[ "$#" -ge 2 ]] || {
        echo "--sanitizers requires on or off" >&2
        exit 2
      }
      sanitizers="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

case "${build_type}" in
  Debug|Release) ;;
  *)
    echo "--build-type must be Debug or Release" >&2
    exit 2
    ;;
esac
case "${sanitizers}" in
  on|off) ;;
  *)
    echo "--sanitizers must be on or off" >&2
    exit 2
    ;;
esac

readonly build_label="$(printf '%s' "${build_type}" | tr '[:upper:]' '[:lower:]')"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/ci-native-${build_label}-${sanitizers}"
declare -a generator_args=()
if command -v ninja >/dev/null 2>&1; then
  generator_args=(-G Ninja)
fi

cmake -E remove_directory "${BUILD_ROOT}"
cmake -E make_directory "${BUILD_ROOT}"

bash -n "${PROJECT_ROOT}"/tools/*.sh
"${PROJECT_ROOT}/tools/test_firmware_artifact_verification.sh"
"${PROJECT_ROOT}/tools/test_microros_agent_install_state.sh"
"${PROJECT_ROOT}/tools/test_package_board_handoff.sh"
"${PROJECT_ROOT}/tools/test_flash_firmware.sh"

RunCmakeSuite() {
  local name="$1"
  local source_directory="$2"
  shift 2
  local build_directory="${BUILD_ROOT}/${name}"
  declare -a sanitizer_args=(-DCMAKE_VERBOSE_MAKEFILE=OFF)
  case "${name}" in
    domain)
      sanitizer_args+=(-DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF)
      ;;
    drivers)
      sanitizer_args+=(-DMENTOR_PI_MCU_DRIVER_SANITIZERS=OFF)
      ;;
    controller)
      sanitizer_args+=(
        -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF
        -DMENTOR_PI_MCU_DRIVER_SANITIZERS=OFF
        -DMENTOR_PI_MCU_CONTROLLER_SANITIZERS=OFF
      )
      ;;
  esac
  if [[ "${sanitizers}" == "on" ]]; then
    sanitizer_args+=(
      '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer'
      '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
    )
  fi

  cmake -S "${source_directory}" -B "${build_directory}" \
    "${generator_args[@]}" \
    -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE="${build_type}" \
    "${sanitizer_args[@]}" \
    "$@"
  cmake --build "${build_directory}" --parallel
  ctest --test-dir "${build_directory}" --output-on-failure
}

python3 -m unittest discover \
  -s "${PROJECT_ROOT}/src/mentor_pi_interfaces/test" -v

RunCmakeSuite domain "${PROJECT_ROOT}/firmware/mentor_pi_mcu"
RunCmakeSuite drivers "${PROJECT_ROOT}/firmware/mentor_pi_mcu/drivers"
RunCmakeSuite controller \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/controller"
RunCmakeSuite microros \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/microros"
RunCmakeSuite bringup "${PROJECT_ROOT}/src/mentor_pi_bringup" \
  -DMENTOR_PI_BUILD_ROS2=OFF

echo "Native ${build_type} tests passed (sanitizers=${sanitizers})"
