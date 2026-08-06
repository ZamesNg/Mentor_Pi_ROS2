#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly BOOTSTRAP_TOOL="${SCRIPT_DIR}/bootstrap_firmware_dependencies.sh"
readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-dependency-provenance.XXXXXX")"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "firmware dependency provenance test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

readonly CHECKOUT="${TEST_ROOT}/checkout"
readonly ORIGIN="https://github.com/example/pinned-dependency.git"
git init -q "${CHECKOUT}"
git -C "${CHECKOUT}" config user.name "Mentor Pi test"
git -C "${CHECKOUT}" config user.email "test@mentor-pi.invalid"
printf '%s\n' pinned >"${CHECKOUT}/source.txt"
git -C "${CHECKOUT}" add source.txt
git -C "${CHECKOUT}" commit -q -m pinned
git -C "${CHECKOUT}" remote add origin "${ORIGIN}"
readonly COMMIT="$(git -C "${CHECKOUT}" rev-parse HEAD)"
readonly BRANCH="$(git -C "${CHECKOUT}" symbolic-ref --short HEAD)"
git -C "${CHECKOUT}" checkout -q --detach "${COMMIT}"

Verify() {
  "${BOOTSTRAP_TOOL}" --verify-existing \
    "${ORIGIN}" "${COMMIT}" "${CHECKOUT}"
}

Verify

git -C "${CHECKOUT}" checkout -q "${BRANCH}"
ExpectFailure Verify
git -C "${CHECKOUT}" checkout -q --detach "${COMMIT}"

printf '%s\n' dirty >"${CHECKOUT}/source.txt"
ExpectFailure Verify
git -C "${CHECKOUT}" restore source.txt

printf '%s\n' untracked >"${CHECKOUT}/untracked.txt"
ExpectFailure Verify
rm -f -- "${CHECKOUT}/untracked.txt"

git -C "${CHECKOUT}" remote set-url origin \
  https://github.com/example/wrong-dependency.git
ExpectFailure Verify
git -C "${CHECKOUT}" remote set-url origin "${ORIGIN}"

git -C "${CHECKOUT}" remote set-url --add origin \
  https://github.com/example/alternate-dependency.git
ExpectFailure Verify
git -C "${CHECKOUT}" config --unset-all remote.origin.url
git -C "${CHECKOUT}" remote set-url origin "${ORIGIN}"

git -C "${CHECKOUT}" remote set-url --push origin \
  https://github.com/example/push-dependency.git
ExpectFailure Verify
git -C "${CHECKOUT}" config --unset-all remote.origin.pushurl

git -C "${CHECKOUT}" checkout -q "${BRANCH}"
printf '%s\n' second >>"${CHECKOUT}/source.txt"
git -C "${CHECKOUT}" add source.txt
git -C "${CHECKOUT}" commit -q -m second
git -C "${CHECKOUT}" checkout -q --detach HEAD
ExpectFailure Verify
git -C "${CHECKOUT}" checkout -q --detach "${COMMIT}"

Verify
echo "firmware dependency provenance tests passed."
