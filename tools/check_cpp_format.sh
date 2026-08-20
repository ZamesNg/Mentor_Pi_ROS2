#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

FindClangFormat() {
  local candidate
  for candidate in "${CLANG_FORMAT:-}" clang-format clang-format-18; do
    if [[ -n "${candidate}" ]] && command -v "${candidate}" >/dev/null 2>&1; then
      command -v "${candidate}"
      return 0
    fi
  done
  echo "clang-format was not found; set CLANG_FORMAT or install clang-format 18." \
    >&2
  return 1
}

CLANG_FORMAT_BINARY="$(FindClangFormat)" || exit 1
readonly CLANG_FORMAT_BINARY
declare -a sources=()
while IFS= read -r -d '' source; do
  sources+=("${source}")
done < <(
  find \
    "${PROJECT_ROOT}/firmware/mentor_pi_mcu" \
    "${PROJECT_ROOT}/ros2_ws/src/mentor_pi_bringup" \
    "${PROJECT_ROOT}/ros2_ws/src/mentor_pi_hardwares" \
    "${PROJECT_ROOT}/ros2_ws/src/mentor_pi_interfaces" \
    -type f \( \
      -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o \
      -name '*.h' -o -name '*.hpp' \
    \) \
    -not -path '*/build/*' \
    -not -path '*/generated/*' \
    -not -path '*/third_party/*' \
    -print0
)

if [[ "${#sources[@]}" -eq 0 ]]; then
  echo "No first-party C/C++ sources were found." >&2
  exit 1
fi

"${CLANG_FORMAT_BINARY}" --dry-run --Werror "${sources[@]}"
echo "clang-format passed for ${#sources[@]} first-party files"
