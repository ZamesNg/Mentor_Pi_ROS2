#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FILES="$(mktemp)"
trap 'rm -f "${FILES}"' EXIT

Fail() {
  echo "active build-policy test failed: $*" >&2
  exit 1
}

git -C "${PROJECT_ROOT}" ls-files --cached --others --exclude-standard -z | \
    while IFS= read -r -d '' path; do
  case "${path}" in
    Makefile | \
    .github/workflows/* | \
    tools/docker/* | \
    tools/*.sh)
      case "${path}" in
        tools/test_*.sh) continue ;;
      esac
      ;;
    firmware/mentor_pi_mcu/*)
      case "${path}" in
        *.md | */tests/* | */test/*) continue ;;
      esac
      ;;
    src/mentor_pi_bringup/* | src/mentor_pi_interfaces/*)
      case "${path}" in
        *.md | */test/*) continue ;;
      esac
      ;;
    *) continue ;;
  esac
  [[ -e "${PROJECT_ROOT}/${path}" || -L "${PROJECT_ROOT}/${path}" ]] || \
    continue
  printf '%s\n' "${path}" >>"${FILES}"
done

[[ -s "${FILES}" ]] || Fail "active-file selection is empty"
while IFS= read -r path; do
  [[ -f "${PROJECT_ROOT}/${path}" && ! -L "${PROJECT_ROOT}/${path}" ]] || \
    Fail "selected path is missing or symbolic: ${path}"
  if LC_ALL=C grep -Ein 'platformio|(^|[^[:alnum:]_])[.]pio([^[:alnum:]_]|$)|jazzy' \
      "${PROJECT_ROOT}/${path}" >/dev/null; then
    echo "Disallowed active build/runtime reference in ${path}:" >&2
    LC_ALL=C grep -Ein \
      'platformio|(^|[^[:alnum:]_])[.]pio([^[:alnum:]_]|$)|jazzy' \
      "${PROJECT_ROOT}/${path}" >&2 || true
    exit 1
  fi
done <"${FILES}"

if git -C "${PROJECT_ROOT}" ls-files | \
    LC_ALL=C grep -Ei '(^|/)platformio[.]ini$|(^|/)[.]pio(/|$)' >/dev/null; then
  Fail "a PlatformIO configuration or cache path is tracked"
fi

echo "Active build, flash, CI, and runtime paths are Humble-only and PlatformIO-free."
