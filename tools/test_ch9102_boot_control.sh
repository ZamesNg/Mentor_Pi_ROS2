#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly OUTPUT_DIRECTORY="${PROJECT_ROOT}/build/tools/tests"
readonly OUTPUT="${OUTPUT_DIRECTORY}/test_ch9102_boot_control"

compiler="${CXX:-}"
if [[ -z "${compiler}" ]]; then
  compiler="$(command -v g++ || command -v clang++ || true)"
fi
[[ -n "${compiler}" ]] || {
  echo "A C++17 compiler is required for CH9102F boot-control tests." >&2
  exit 1
}

mkdir -p "${OUTPUT_DIRECTORY}"
"${compiler}" -std=c++17 -O0 -g -Wall -Wextra -Wpedantic -Werror \
  -DMENTOR_PI_CH9102_BOOT_CONTROL_NO_MAIN \
  -I"${SCRIPT_DIR}" \
  "${SCRIPT_DIR}/ch9102_boot_control.cc" \
  "${SCRIPT_DIR}/test_ch9102_boot_control.cc" \
  -o "${OUTPUT}"
"${OUTPUT}"

readonly CLI="$(${SCRIPT_DIR}/build_ch9102_boot_control.sh)"
if output="$(${CLI} --device /dev/null --mode application 2>&1)"; then
  echo "The boot controller accepted an unsupported character device." >&2
  exit 1
fi
[[ "${output}" == *"expected 1a86:55d4"* ]] || {
  echo "The unsupported-adapter error was not precise: ${output}" >&2
  exit 1
}
if output="$(${CLI} --device /dev/mentor-pi-device-does-not-exist \
    --mode application 2>&1)"; then
  echo "The boot controller accepted a nonexistent device." >&2
  exit 1
fi
[[ "${output}" == *"existing /dev character device"* ]] || {
  echo "The nonexistent-device error was not precise: ${output}" >&2
  exit 1
}
if output="$(${CLI} --device /dev/null --mode invalid 2>&1)"; then
  echo "The boot controller accepted an invalid mode." >&2
  exit 1
fi
[[ "${output}" == *"mode must be bootloader or application"* ]] || {
  echo "The invalid-mode error was not precise: ${output}" >&2
  exit 1
}
