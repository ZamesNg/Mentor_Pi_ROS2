#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
readonly INTERFACE_ROOT="${1:-${REPOSITORY_ROOT}/ros2_ws/src/mentor_pi_interfaces}"
readonly SHA256="${SCRIPT_DIR}/sha256.sh"

[[ -f "${INTERFACE_ROOT}/CMakeLists.txt" && \
   -f "${INTERFACE_ROOT}/package.xml" && \
   -d "${INTERFACE_ROOT}/include" && -d "${INTERFACE_ROOT}/msg" && \
   -d "${INTERFACE_ROOT}/srv" ]] || {
  echo "mentor_pi_interfaces is incomplete: ${INTERFACE_ROOT}" >&2
  exit 1
}

manifest="$(mktemp)"
paths="$(mktemp)"
trap 'rm -f -- "${manifest}" "${paths}"' EXIT
printf '%s\n' 'CMakeLists.txt' 'package.xml' >"${paths}"
find "${INTERFACE_ROOT}/include" "${INTERFACE_ROOT}/msg" \
  "${INTERFACE_ROOT}/srv" -type f ! -name '.DS_Store' -print | \
  while IFS= read -r file; do
    printf '%s\n' "${file#"${INTERFACE_ROOT}/"}"
  done >>"${paths}"
files=()
while IFS= read -r file; do
  files+=("${file}")
done < <(LC_ALL=C sort -u "${paths}")
"${SCRIPT_DIR}/sha256_manifest.sh" "${INTERFACE_ROOT}" \
  "${files[@]}" >"${manifest}"
"${SHA256}" "${manifest}"
