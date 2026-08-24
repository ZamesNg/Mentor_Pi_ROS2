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
files=()
while IFS= read -r file; do
  files+=("${file#"${SDK_ROOT}/"}")
done < <(
  find "${SDK_ROOT}" -type f ! -name '.DS_Store' -print | LC_ALL=C sort
)
"${SCRIPT_DIR}/sha256_manifest.sh" "${SDK_ROOT}" \
  "${files[@]}" >"${manifest}"
"${SHA256}" "${manifest}"
