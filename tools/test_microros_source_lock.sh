#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly LOCK_TOOL="${SCRIPT_DIR}/apply_microros_source_lock.sh"
readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-source-lock.XXXXXX")"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "micro-ROS source-lock test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

MakeRepository() {
  local destination="$1"
  local origin="$2"
  git init -q "${destination}"
  git -C "${destination}" config user.name "Mentor Pi test"
  git -C "${destination}" config user.email "test@mentor-pi.invalid"
  printf '%s\n' source >"${destination}/source.txt"
  git -C "${destination}" add source.txt
  git -C "${destination}" commit -q -m source
  git -C "${destination}" remote add origin "${origin}"
}

readonly ORIGIN="https://github.com/example/micro-ros-source.git"
readonly DEFERRED_ORIGIN="https://github.com/example/deferred-source.git"
readonly REPOSITORY="${TEST_ROOT}/workspace/source"
readonly SOURCE_LOCK="${TEST_ROOT}/source.lock"
MakeRepository "${REPOSITORY}" "${ORIGIN}"
readonly COMMIT="$(git -C "${REPOSITORY}" rev-parse HEAD)"
printf '%s %s\n' \
  "${ORIGIN%.git}" "${COMMIT}" \
  "${DEFERRED_ORIGIN%.git}" "${COMMIT}" >"${SOURCE_LOCK}"

ApplyLock() {
  "${LOCK_TOOL}" "${TEST_ROOT}/workspace" "${SOURCE_LOCK}" \
    --deferred-repository "${DEFERRED_ORIGIN}"
}

ExpectFailure "${LOCK_TOOL}" "${TEST_ROOT}/workspace" "${SOURCE_LOCK}"
ApplyLock >/dev/null
git -C "${REPOSITORY}" symbolic-ref -q HEAD >/dev/null 2>&1 &&
  Fail "locked repository remained attached to a branch"

ExpectFailure "${LOCK_TOOL}" "${TEST_ROOT}/workspace" "${SOURCE_LOCK}" \
  --deferred-repository https://github.com/example/not-locked.git
ExpectFailure "${LOCK_TOOL}" "${TEST_ROOT}/workspace" "${SOURCE_LOCK}" \
  --deferred-repository "${DEFERRED_ORIGIN}" \
  --deferred-repository "${DEFERRED_ORIGIN}"

printf '%s\n' untracked >"${REPOSITORY}/untracked.txt"
ExpectFailure ApplyLock
rm -f -- "${REPOSITORY}/untracked.txt"

printf '%s\n' modified >"${REPOSITORY}/source.txt"
ExpectFailure ApplyLock

echo "micro-ROS source-lock tests passed."
