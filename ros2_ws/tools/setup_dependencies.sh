#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly CONTROLLERS_ROOT="${WORKSPACE_ROOT}/third_party/ros2_controllers"
readonly CONTROLLERS_URL="https://github.com/ros-controls/ros2_controllers.git"
readonly CONTROLLERS_COMMIT="ad559d9c8054f128296ac094d7130e162e61b37a"
readonly CONTROLLERS_PATCH="${WORKSPACE_ROOT}/patches/ros2-controllers-zero-stamp-warning.patch"

Fail() {
  echo "ROS dependency setup error: $*" >&2
  exit 1
}

CanonicalGitUrl() {
  local url="${1%/}"
  printf '%s' "${url%.git}"
}

"${SCRIPT_DIR}/check_environment.sh" >/dev/null
mkdir -p "${WORKSPACE_ROOT}/third_party"
if [[ ! -d "${CONTROLLERS_ROOT}/.git" ]]; then
  [[ ! -e "${CONTROLLERS_ROOT}" ]] || \
    Fail "refusing to replace ${CONTROLLERS_ROOT}"
  vcs import --input "${WORKSPACE_ROOT}/dependencies.repos" \
    "${WORKSPACE_ROOT}"
fi
[[ "$(CanonicalGitUrl \
  "$(git -C "${CONTROLLERS_ROOT}" config --get remote.origin.url)")" == \
  "$(CanonicalGitUrl "${CONTROLLERS_URL}")" ]] || \
  Fail "ros2_controllers origin differs from dependencies.repos"
[[ "$(git -C "${CONTROLLERS_ROOT}" rev-parse HEAD)" == \
  "${CONTROLLERS_COMMIT}" ]] || Fail "ros2_controllers is not at the pin"
[[ -f "${CONTROLLERS_PATCH}" ]] || Fail "controller patch is missing"
if git -C "${CONTROLLERS_ROOT}" apply --check "${CONTROLLERS_PATCH}"; then
  git -C "${CONTROLLERS_ROOT}" apply "${CONTROLLERS_PATCH}"
elif ! git -C "${CONTROLLERS_ROOT}" apply --reverse --check \
  "${CONTROLLERS_PATCH}"; then
  Fail "controller patch does not apply cleanly"
fi
git -C "${CONTROLLERS_ROOT}" diff --check
mapfile -t changed < <(git -C "${CONTROLLERS_ROOT}" diff --name-only)
readonly -a expected_changed=(
  mecanum_drive_controller/src/mecanum_drive_controller.cpp
  steering_controllers_library/src/steering_controllers_library.cpp
)
[[ "${changed[*]}" == "${expected_changed[*]}" ]] || \
  Fail "controller checkout contains changes outside the reviewed patch"

# Resolve only against the supported binary Humble underlay, regardless of
# ROS workspaces sourced by the caller.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset ROS_PACKAGE_PATH

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
rosdep install --rosdistro humble --from-paths \
  "${WORKSPACE_ROOT}/src" \
  "${CONTROLLERS_ROOT}/ackermann_steering_controller" \
  "${CONTROLLERS_ROOT}/mecanum_drive_controller" \
  "${CONTROLLERS_ROOT}/steering_controllers_library" \
  --ignore-src --as-root pip:false -y
echo "Pinned and patched ROS workspace dependencies are ready."
