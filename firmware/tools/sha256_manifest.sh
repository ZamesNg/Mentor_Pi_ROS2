#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly ROOT="${1:-}"
[[ "$#" -ge 2 && -d "${ROOT}" ]] || {
  echo "Usage: sha256_manifest.sh ROOT RELATIVE_FILE..." >&2
  exit 2
}
shift

relative_files=("$@")
absolute_files=()
for relative in "${relative_files[@]}"; do
  [[ -n "${relative}" && "${relative}" != /* && \
     "${relative}" != *$'\n'* && -f "${ROOT}/${relative}" ]] || {
    echo "Invalid manifest file below ${ROOT}: ${relative}" >&2
    exit 1
  }
  absolute_files+=("${ROOT}/${relative}")
done

digests="$(mktemp)"
trap 'rm -f -- "${digests}"' EXIT
"${SCRIPT_DIR}/sha256.sh" "${absolute_files[@]}" >"${digests}"

index=0
while IFS= read -r digest; do
  ((index < ${#relative_files[@]})) || {
    echo "SHA-256 backend returned too many digests" >&2
    exit 1
  }
  [[ "${digest}" =~ ^[0-9a-f]{64}$ ]] || {
    echo "SHA-256 backend returned an invalid digest" >&2
    exit 1
  }
  printf '%s  %s\n' "${digest}" "${relative_files[index]}"
  index=$((index + 1))
done <"${digests}"
((index == ${#relative_files[@]})) || {
  echo "SHA-256 backend returned too few digests" >&2
  exit 1
}
