#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${FIRMWARE_ROOT}/mentor_pi_mcu/build"
[[ "${BUILD_ROOT}" == "${FIRMWARE_ROOT}/mentor_pi_mcu/build" && \
   "${BUILD_ROOT}" != / ]] || { echo "Refusing unsafe cleanup" >&2; exit 1; }
rm -rf -- "${BUILD_ROOT}"
