#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly CONTROLLERS_ROOT="${WORKSPACE_ROOT}/third_party/ros2_controllers"
readonly -a PACKAGES=(
  ackermann_steering_controller
  mecanum_drive_controller
  mentor_pi_bringup
  mentor_pi_hardwares
  mentor_pi_interfaces
  steering_controllers_library
)

"${SCRIPT_DIR}/check_environment.sh" >/dev/null

# Workspace setup files retain the underlays active when they were built.
# Always generate this workspace against the supported binary Humble underlay.
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
export ROS_DISTRO=humble

cd "${WORKSPACE_ROOT}"
command_name="${1:-}"
[[ -n "${command_name}" ]] || {
  echo "usage: colcon.sh build|test|test-result|list [arguments]" >&2
  exit 2
}
shift
case "${command_name}" in
  build)
    exec colcon --log-base log build "$@" \
      --base-paths src "${CONTROLLERS_ROOT}" \
      --packages-select "${PACKAGES[@]}" \
      --build-base build --install-base install \
      --cmake-clean-cache
    ;;
  test)
    exec colcon --log-base log test "$@" \
      --packages-select "${PACKAGES[@]}" \
      --build-base build --install-base install
    ;;
  test-result)
    exec colcon --log-base log test-result "$@" --test-result-base build
    ;;
  list)
    exec colcon --log-base log list "$@" \
      --base-paths src "${CONTROLLERS_ROOT}" \
      --packages-select "${PACKAGES[@]}"
    ;;
  *)
    echo "unsupported colcon command: ${command_name}" >&2
    exit 2
    ;;
esac
