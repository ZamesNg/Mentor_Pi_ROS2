#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IGNORE_FILE="${PROJECT_ROOT}/.gitignore"
readonly TEMPORARY_PARENT="${TMPDIR:-/tmp}"
TEST_ROOT="$(mktemp -d "${TEMPORARY_PARENT}/rrclite-gitignore-test.XXXXXX")"
readonly TEST_ROOT

Fail() {
  echo "Git ignore contract test failure: $*" >&2
  exit 1
}

Cleanup() {
  case "${TEST_ROOT}" in
    "${TEMPORARY_PARENT}"/rrclite-gitignore-test.*)
      rm -rf -- "${TEST_ROOT}"
      ;;
    *)
      echo "Refusing unsafe temporary cleanup: ${TEST_ROOT}" >&2
      ;;
  esac
}
trap Cleanup EXIT

command -v git >/dev/null 2>&1 || Fail "git is not installed"
[[ -f "${IGNORE_FILE}" ]] || Fail ".gitignore is missing"

git -C "${TEST_ROOT}" init --quiet
cp "${IGNORE_FILE}" "${TEST_ROOT}/.gitignore"
mkdir -p \
  "${TEST_ROOT}/docs/reference/RosRobotControllerLite_ros_250811" \
  "${TEST_ROOT}/docs/framework" \
  "${TEST_ROOT}/.pio/build" \
  "${TEST_ROOT}/firmware/mentor_pi_mcu/.pio/build"
touch "${TEST_ROOT}/docs/reference/legacy.txt"
touch "${TEST_ROOT}/docs/reference/README.md"
git -C "${TEST_ROOT}/docs/reference/RosRobotControllerLite_ros_250811" \
  init --quiet
touch \
  "${TEST_ROOT}/docs/reference/RosRobotControllerLite_ros_250811/legacy.c" \
  "${TEST_ROOT}/.pio/build/root-cache.bin" \
  "${TEST_ROOT}/firmware/mentor_pi_mcu/.pio/build/nested-cache.bin"
touch "${TEST_ROOT}/docs/framework/design.md"

git -C "${TEST_ROOT}" check-ignore --quiet docs/reference/legacy.txt || \
  Fail "docs/reference content is not ignored"
git -C "${TEST_ROOT}" check-ignore --quiet \
  docs/reference/RosRobotControllerLite_ros_250811/legacy.c || \
  Fail "a nested legacy Git worktree is not ignored by the parent repository"
if git -C "${TEST_ROOT}" check-ignore --quiet docs/reference/README.md; then
  Fail "the tracked legacy-reference policy is unexpectedly ignored"
fi
if git -C "${TEST_ROOT}" check-ignore --quiet docs/framework/design.md; then
  Fail "the maintained framework documentation is unexpectedly ignored"
fi
git -C "${TEST_ROOT}" check-ignore --quiet .pio/build/root-cache.bin || \
  Fail "the root PlatformIO cache is not ignored"
git -C "${TEST_ROOT}" check-ignore --quiet \
  firmware/mentor_pi_mcu/.pio/build/nested-cache.bin || \
  Fail "a nested PlatformIO cache is not ignored"

readonly DRY_RUN_ADD="$(git -C "${TEST_ROOT}" add --dry-run .)"
[[ "${DRY_RUN_ADD}" != *"docs/reference/legacy.txt"* ]] || \
  Fail "a parent-repository add would include raw legacy evidence"
[[ "${DRY_RUN_ADD}" != *"docs/reference/RosRobotControllerLite"* ]] || \
  Fail "a parent-repository add would include a nested legacy worktree"
[[ "${DRY_RUN_ADD}" == *"docs/reference/README.md"* ]] || \
  Fail "a parent-repository add would omit the legacy-reference policy"
if [[ "${DRY_RUN_ADD}" == *".pio/"* ]]; then
  Fail "a parent-repository add would include obsolete PlatformIO cache data"
fi
[[ "${DRY_RUN_ADD}" == *"docs/framework/design.md"* ]] || \
  Fail "a parent-repository add would omit maintained framework documentation"

echo "Git ignore contract tests passed"
