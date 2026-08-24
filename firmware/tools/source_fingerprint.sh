#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_FIRMWARE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FIRMWARE_ROOT="${1:-${DEFAULT_FIRMWARE_ROOT}}"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"
[[ "$#" -le 1 && -d "${FIRMWARE_ROOT}" ]] || {
  echo "Usage: source_fingerprint.sh [FIRMWARE_ROOT]" >&2
  exit 2
}
manifest="$(mktemp)"
trap 'rm -f -- "${manifest}"' EXIT
files=()
while IFS= read -r file; do
  files+=("${file#"${FIRMWARE_ROOT}/"}")
done < <(
  find "${FIRMWARE_ROOT}" -type f \
    ! -path '*/build/*' ! -path '*/third_party/*' ! -path '*/.deps/*' \
    ! -path '*/sdk/humble/libmicroros.tar.xz' ! -name '.DS_Store' \
    ! -name '*.pyc' | LC_ALL=C sort
)
(( ${#files[@]} > 0 )) || {
  echo "Firmware source tree contains no fingerprintable files" >&2
  exit 1
}
"${SCRIPT_DIR}/sha256_manifest.sh" "${FIRMWARE_ROOT}" \
  "${files[@]}" >"${manifest}"
"${SHA256}" "${manifest}"
