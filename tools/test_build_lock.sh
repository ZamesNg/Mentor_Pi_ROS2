#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly LOCKER="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly TEST_ROOT="$(mktemp -d)"

Cleanup() {
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

[[ -x "${LOCKER}" ]] || {
  echo "Build-lock contract test failed: lock helper is unavailable." >&2
  exit 1
}

readonly STATE="${TEST_ROOT}/state"
"${LOCKER}" bash -c 'printf "held\n" >"$1"; sleep 0.2; printf "released\n" >"$1"' \
  _ "${STATE}" \
  >"${TEST_ROOT}/first" &
first_pid=$!
for _ in {1..50}; do
  [[ -f "${STATE}" ]] && break
  sleep 0.01
done
[[ "$(tr -d '[:space:]' <"${STATE}")" == held ]] || {
  echo "Build-lock contract test failed: first holder did not acquire." >&2
  exit 1
}
"${LOCKER}" bash -c 'test "$(tr -d "[:space:]" <"$1")" = released; printf "second\n"' \
  _ "${STATE}" >"${TEST_ROOT}/second" &
second_pid=$!
wait "${first_pid}" "${second_pid}"

[[ "$(tr -d '[:space:]' <"${STATE}")" == released ]] || {
  echo "Build-lock contract test failed: first holder did not complete." >&2
  exit 1
}
grep -Fqx second "${TEST_ROOT}/second" || {
  echo "Build-lock contract test failed: second holder did not complete." >&2
  exit 1
}
first_end_time="$(stat -c %Y "${TEST_ROOT}/first")"
second_end_time="$(stat -c %Y "${TEST_ROOT}/second")"
((second_end_time >= first_end_time)) || {
  echo "Build-lock contract test failed: lock wait ordering is invalid." >&2
  exit 1
}

echo "Shared Docker build-lock contract tests passed."
