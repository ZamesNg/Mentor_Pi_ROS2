#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly WORKSPACE_ROOT="${PROJECT_ROOT}/mentor_pi_ros2"
readonly ACTIVE_PATHS=(
  "${PROJECT_ROOT}/Makefile"
  "${PROJECT_ROOT}/.github"
  "${PROJECT_ROOT}/firmware"
  "${PROJECT_ROOT}/tools"
  "${WORKSPACE_ROOT}"
)

Fail() {
  echo "ROS workspace layout test failure: $*" >&2
  exit 1
}

[[ ! -e "${PROJECT_ROOT}/src" && ! -L "${PROJECT_ROOT}/src" ]] || \
  Fail "the repository-root src path must not exist"
[[ -f "${WORKSPACE_ROOT}/README.md" ]] || \
  Fail "the ROS workspace README is missing"

for package in mentor_pi_interfaces mentor_pi_bringup mentor_pi_hardwares; do
  package_root="${WORKSPACE_ROOT}/src/${package}"
  [[ -f "${package_root}/package.xml" && \
    -f "${package_root}/CMakeLists.txt" ]] || \
    Fail "workspace package is incomplete: ${package}"
  [[ ! -L "${package_root}" ]] || \
    Fail "workspace package must not be symbolic: ${package}"
done

if find "${WORKSPACE_ROOT}" -type d -name ros_package_schema -print -quit | \
    grep -q .; then
  Fail "the removed package-schema snapshot is present"
fi

if rg -n --pcre2 \
    --glob '!test_ros_workspace_layout.sh' \
    --glob '!**/build/**' --glob '!**/install/**' --glob '!**/log/**' \
    --glob '!**/third_party/**' --glob '!**/generated/**' \
    '(?<!mentor_pi_ros2/)src/mentor_pi_(interfaces|bringup|hardwares)|src/[*][*]|src/ros_package_schema|ros_package_schema|xml-model' \
    "${ACTIVE_PATHS[@]}" >/dev/null; then
  rg -n --pcre2 \
    --glob '!test_ros_workspace_layout.sh' \
    --glob '!**/build/**' --glob '!**/install/**' --glob '!**/log/**' \
    --glob '!**/third_party/**' --glob '!**/generated/**' \
    '(?<!mentor_pi_ros2/)src/mentor_pi_(interfaces|bringup|hardwares)|src/[*][*]|src/ros_package_schema|ros_package_schema|xml-model' \
    "${ACTIVE_PATHS[@]}" >&2 || true
  Fail "an active build, runtime, package, CI, or test path uses the old layout"
fi

echo "ROS workspace layout and schema-removal contracts passed."
