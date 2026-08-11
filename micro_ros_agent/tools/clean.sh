#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${COMPONENT_ROOT}/build"
[[ "${BUILD_ROOT}" == "${COMPONENT_ROOT}/build" && \
   "${BUILD_ROOT}" != / ]] || {
  echo "Refusing unsafe Agent cleanup: ${BUILD_ROOT}" >&2
  exit 1
}
rm -rf -- "${BUILD_ROOT}"
