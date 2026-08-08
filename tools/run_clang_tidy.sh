#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/clang-tidy"

FindTool() {
  local requested="$1"
  shift
  local candidate
  for candidate in "${requested}" "$@"; do
    if [[ -n "${candidate}" ]] && command -v "${candidate}" >/dev/null 2>&1; then
      command -v "${candidate}"
      return 0
    fi
  done
  return 1
}

if ! clang_tidy_binary="$(
  FindTool "${CLANG_TIDY:-}" clang-tidy clang-tidy-18
)"; then
  echo "clang-tidy was not found; set CLANG_TIDY or install clang-tidy 18." >&2
  exit 1
fi
readonly CLANG_TIDY_BINARY="${clang_tidy_binary}"
if ! run_clang_tidy_binary="$(
  FindTool "${RUN_CLANG_TIDY:-}" run-clang-tidy run-clang-tidy-18
)"; then
  echo "run-clang-tidy was not found; set RUN_CLANG_TIDY." >&2
  exit 1
fi
readonly RUN_CLANG_TIDY_BINARY="${run_clang_tidy_binary}"

# run-clang-tidy may aggregate child output without propagating configuration
# parse failures on every supported distro. Validate the repository policy once
# up front so an unknown key can never be reported as a passing analysis run.
(
  cd "${PROJECT_ROOT}"
  "${CLANG_TIDY_BINARY}" --verify-config
)

declare -a clang_tidy_platform_args=()
if [[ "$(uname -s)" == "Darwin" ]]; then
  command -v xcrun >/dev/null 2>&1 || {
    echo "xcrun is required to locate the macOS SDK for clang-tidy." >&2
    exit 1
  }
  readonly MACOS_SDK_ROOT="$(xcrun --show-sdk-path)"
  clang_tidy_platform_args=(
    -extra-arg-before=-isysroot
    "-extra-arg-before=${MACOS_SDK_ROOT}"
  )
fi

declare -a generator_args=()
if command -v ninja >/dev/null 2>&1; then
  generator_args=(-G Ninja)
fi

cmake -E remove_directory "${BUILD_ROOT}"
cmake -E make_directory "${BUILD_ROOT}"

ConfigureAndAnalyze() {
  local name="$1"
  local source_directory="$2"
  local file_pattern="$3"
  shift 3
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

  cmake -S "${source_directory}" -B "${build_directory}" \
    "${generator_args[@]}" \
    -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "${sanitizer_args[@]}" \
    "$@"
  "${RUN_CLANG_TIDY_BINARY}" \
    -clang-tidy-binary "${CLANG_TIDY_BINARY}" \
    -p "${build_directory}" \
    "${clang_tidy_platform_args[@]}" \
    -quiet \
    "${file_pattern}"
}

ConfigureAndAnalyze \
  domain \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu" \
  '.*/firmware/mentor_pi_mcu/(src|tests)/.*\.cc$'
ConfigureAndAnalyze \
  drivers \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/drivers" \
  '.*/firmware/mentor_pi_mcu/drivers/(src|tests)/.*\.cc$'
ConfigureAndAnalyze \
  controller \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/controller" \
  '.*/firmware/mentor_pi_mcu/app/controller/(src|tests)/.*\.cc$'
ConfigureAndAnalyze \
  microros \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/app/microros" \
  '.*/firmware/mentor_pi_mcu/app/microros/(src|tests)/.*\.cc$'
ConfigureAndAnalyze \
  bringup \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup" \
  '.*/mentor_pi_ros2/src/mentor_pi_bringup/(src|test)/.*\.cc$' \
  -DMENTOR_PI_BUILD_ROS2=OFF

echo "clang-tidy passed for every native first-party translation unit"
