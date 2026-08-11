#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SDK_ROOT="${1:?Usage: sdk_tree_fingerprint.sh SDK_ROOT}"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"
[[ -s "${SDK_ROOT}/libmicroros.a" && -d "${SDK_ROOT}/include" ]] || {
  echo "SDK tree is incomplete: ${SDK_ROOT}" >&2
  exit 1
}
[[ -z "$(find "${SDK_ROOT}" -type l -print -quit)" ]] || {
  echo "SDK tree contains a symbolic link" >&2
  exit 1
}
manifest="$(mktemp)"
trap 'rm -f -- "${manifest}"' EXIT
(
  cd "${SDK_ROOT}"
  find . -type f ! -name '.DS_Store' -print | LC_ALL=C sort | \
    while IFS= read -r file; do
      printf '%s  %s\n' "$("${SHA256}" "${file}")" "${file#./}"
    done
) >"${manifest}"
"${SHA256}" "${manifest}"
