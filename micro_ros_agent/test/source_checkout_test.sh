#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CHECKOUT_HELPER="${TEST_DIR}/../tools/checkout_pinned_source.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "micro-ROS Agent source checkout test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output=""
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "expected '${expected}', got: ${output}"
}

readonly ORIGIN_WORK="${TEST_ROOT}/origin-work"
readonly ORIGIN_REPOSITORY="${TEST_ROOT}/origin.git"
git init -q "${ORIGIN_WORK}"
git -C "${ORIGIN_WORK}" config user.name 'Mentor Pi test'
git -C "${ORIGIN_WORK}" config user.email 'mentor-pi-test@example.invalid'
printf '%s\n' 'pinned source' >"${ORIGIN_WORK}/source.txt"
git -C "${ORIGIN_WORK}" add source.txt
git -C "${ORIGIN_WORK}" commit -q -m 'pinned source'
readonly PINNED_COMMIT="$(git -C "${ORIGIN_WORK}" rev-parse HEAD)"
git init -q --bare "${ORIGIN_REPOSITORY}"
git -C "${ORIGIN_WORK}" remote add origin "${ORIGIN_REPOSITORY}"
git -C "${ORIGIN_WORK}" push -q origin HEAD:refs/heads/main

readonly FRESH_CHECKOUT="${TEST_ROOT}/fresh"
"${CHECKOUT_HELPER}" "${ORIGIN_REPOSITORY}" "${PINNED_COMMIT}" \
  "${FRESH_CHECKOUT}" >/dev/null
[[ "$(git -C "${FRESH_CHECKOUT}" rev-parse HEAD)" == "${PINNED_COMMIT}" ]] || \
  Fail "fresh checkout did not select the pinned commit"
if git -C "${FRESH_CHECKOUT}" symbolic-ref -q HEAD >/dev/null 2>&1; then
  Fail "fresh checkout is not detached"
fi

readonly INTERRUPTED_CHECKOUT="${TEST_ROOT}/interrupted"
git init -q "${INTERRUPTED_CHECKOUT}"
git -C "${INTERRUPTED_CHECKOUT}" remote add origin "${ORIGIN_REPOSITORY}"
recovery_output="$("${CHECKOUT_HELPER}" "${ORIGIN_REPOSITORY}" \
  "${PINNED_COMMIT}" "${INTERRUPTED_CHECKOUT}")"
[[ "${recovery_output}" == *'Recovering incomplete pinned checkout:'* ]] || \
  Fail "interrupted checkout recovery was not reported"
[[ "$(git -C "${INTERRUPTED_CHECKOUT}" rev-parse HEAD)" == \
   "${PINNED_COMMIT}" ]] || \
  Fail "interrupted checkout did not recover the pinned commit"

readonly DIRTY_CHECKOUT="${TEST_ROOT}/dirty"
git init -q "${DIRTY_CHECKOUT}"
git -C "${DIRTY_CHECKOUT}" remote add origin "${ORIGIN_REPOSITORY}"
printf '%s\n' 'unexpected state' >"${DIRTY_CHECKOUT}/residue.txt"
ExpectFailure 'incomplete checkout has working-tree state' \
  "${CHECKOUT_HELPER}" "${ORIGIN_REPOSITORY}" "${PINNED_COMMIT}" \
  "${DIRTY_CHECKOUT}"

readonly WRONG_ORIGIN_CHECKOUT="${TEST_ROOT}/wrong-origin"
git init -q "${WRONG_ORIGIN_CHECKOUT}"
git -C "${WRONG_ORIGIN_CHECKOUT}" remote add origin \
  "${TEST_ROOT}/unexpected.git"
ExpectFailure 'existing checkout origin mismatch' \
  "${CHECKOUT_HELPER}" "${ORIGIN_REPOSITORY}" "${PINNED_COMMIT}" \
  "${WRONG_ORIGIN_CHECKOUT}"

echo "micro-ROS Agent source checkout tests passed."
