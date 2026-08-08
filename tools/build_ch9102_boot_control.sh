#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly OUTPUT_DIRECTORY="${PROJECT_ROOT}/build/tools"
readonly OUTPUT="${OUTPUT_DIRECTORY}/ch9102_boot_control"
readonly SOURCE="${SCRIPT_DIR}/ch9102_boot_control.cc"
readonly HEADER="${SCRIPT_DIR}/ch9102_boot_control.h"

compiler="${CXX:-}"
if [[ -z "${compiler}" ]]; then
  compiler="$(command -v g++ || command -v clang++ || true)"
fi
[[ -n "${compiler}" ]] || {
  echo "A C++17 compiler is required for CH9102F boot control." >&2
  exit 1
}

mkdir -p "${OUTPUT_DIRECTORY}"
if [[ ! -x "${OUTPUT}" || "${SOURCE}" -nt "${OUTPUT}" || \
    "${HEADER}" -nt "${OUTPUT}" ]]; then
  "${compiler}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
    -I"${SCRIPT_DIR}" "${SOURCE}" -o "${OUTPUT}"
fi
printf '%s\n' "${OUTPUT}"
