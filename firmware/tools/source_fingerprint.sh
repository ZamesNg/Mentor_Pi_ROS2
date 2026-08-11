#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"
manifest="$(mktemp)"
trap 'rm -f -- "${manifest}"' EXIT
find "${FIRMWARE_ROOT}" -type f \
  ! -path '*/build/*' ! -path '*/third_party/*' ! -path '*/.deps/*' \
  ! -path '*/sdk/humble/libmicroros.tar.xz' ! -name '.DS_Store' \
  ! -name '*.pyc' | LC_ALL=C sort | while IFS= read -r file; do
    relative="${file#"${FIRMWARE_ROOT}/"}"
    printf '%s  %s\n' "$("${SHA256}" "${file}")" "${relative}"
  done >"${manifest}"
"${SHA256}" "${manifest}"
