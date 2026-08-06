#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT="${1:-${DEFAULT_PROJECT_ROOT}}"
[[ "$#" -le 1 ]] || {
  echo "Usage: ./tools/microros_artifact_fingerprint.sh [PROJECT_ROOT]" >&2
  exit 2
}

Fail() {
  echo "micro-ROS artifact fingerprint error: $*" >&2
  exit 1
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    Fail "neither sha256sum nor shasum is installed"
  fi
}

readonly LIBRARY_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros"
readonly MANIFEST="$(mktemp)"
readonly UNSORTED_MANIFEST="$(mktemp)"
trap 'rm -f "${MANIFEST}" "${UNSORTED_MANIFEST}"' EXIT

[[ -s "${LIBRARY_ROOT}/libmicroros.a" ]] || \
  Fail "libmicroros.a is missing or empty"
[[ -d "${LIBRARY_ROOT}/include" ]] || Fail "generated include tree is missing"
[[ -f "${LIBRARY_ROOT}/available_ros2_types" ]] || \
  Fail "available_ros2_types is missing"
[[ -f "${LIBRARY_ROOT}/built_packages" ]] || Fail "built_packages is missing"
readonly ARTIFACT_SYMLINK="$(find "${LIBRARY_ROOT}" -type l -print -quit)"
[[ -z "${ARTIFACT_SYMLINK}" ]] || \
  Fail "generated artifact symlink is unsupported: ${ARTIFACT_SYMLINK}"

if command -v sha256sum >/dev/null 2>&1; then
  readonly -a HASH_COMMAND=(sha256sum)
else
  command -v shasum >/dev/null 2>&1 || \
    Fail "neither sha256sum nor shasum is installed"
  readonly -a HASH_COMMAND=(shasum -a 256)
fi

# Hash in batches to avoid spawning one process per generated header. Sort the
# resulting manifest afterward, so filesystem traversal order has no effect.
(
  cd "${LIBRARY_ROOT}"
  {
    printf '%s\0' libmicroros.a available_ros2_types built_packages
    find include -type f ! -name '.DS_Store' -print0
  } | xargs -0 "${HASH_COMMAND[@]}"
) >"${UNSORTED_MANIFEST}"
LC_ALL=C sort -k2,2 "${UNSORTED_MANIFEST}" >"${MANIFEST}"

Sha256 "${MANIFEST}"
