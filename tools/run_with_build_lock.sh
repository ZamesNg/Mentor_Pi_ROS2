#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly LOCK_ROOT="${PROJECT_ROOT}/build"
readonly LOCK_FILE="${LOCK_ROOT}/.mentor-pi-docker-build.lock"

[[ "$#" -gt 0 ]] || {
  echo "Usage: run_with_build_lock.sh COMMAND [ARG ...]" >&2
  exit 2
}
[[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]] || exec "$@"
command -v flock >/dev/null 2>&1 || {
  echo "Docker build serialization requires flock." >&2
  exit 1
}

mkdir -p "${LOCK_ROOT}"
exec 9>"${LOCK_FILE}"
echo "Waiting for the shared Mentor Pi Docker build lock."
flock 9
export RRCLITE_BUILD_LOCK_HELD=1
exec "$@"
