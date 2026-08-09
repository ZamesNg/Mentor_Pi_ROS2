#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ENVIRONMENT_CHECK="${SCRIPT_DIR}/verify_host_build_environment.sh"
readonly IDLE_GUARD="${SCRIPT_DIR}/require_microros_agent_install_idle.sh"
readonly CUBEPROGRAMMER_INSTALLER="${SCRIPT_DIR}/install_onboard_stm32cubeprogrammer.sh"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"

Fail() {
  echo "Host dependency preparation error: $*" >&2
  exit 1
}

[[ "$#" == 0 ]] || Fail "this command accepts no arguments"
[[ "$(id -u)" == 0 ]] || Fail "run this preparation command as root"
[[ -x "${ENVIRONMENT_CHECK}" ]] || Fail "environment verifier is missing"
[[ -x "${IDLE_GUARD}" ]] || Fail "controller idle guard is missing"
[[ -x "${CUBEPROGRAMMER_INSTALLER}" ]] || \
  Fail "STM32CubeProgrammer installer is missing"

"${ENVIRONMENT_CHECK}" --check-tools no
"${IDLE_GUARD}"

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  curl \
  git \
  libxml2-utils \
  ninja-build \
  psmisc \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  rsync \
  ros-humble-micro-ros-setup \
  unzip \
  xz-utils

if [[ "$(dpkg --print-architecture)" == "arm64" ]]; then
  "${CUBEPROGRAMMER_INSTALLER}" --install
else
  echo "Skipping the repository arm64 STM32CubeProgrammer package on this amd64 host."
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro humble

set +u
source "${ROS_SETUP}"
set -u
rosdep install --from-paths \
  "${PROJECT_ROOT}/mentor_pi_ros2/src" \
  --ignore-src --rosdistro humble --as-root pip:false -y

for native_firmware_tool in ninja vcs rsync; do
  command -v "${native_firmware_tool}" >/dev/null 2>&1 || \
    Fail "native firmware tool is unavailable: ${native_firmware_tool}"
done
ros2 pkg prefix micro_ros_setup >/dev/null 2>&1 || \
  Fail "ROS 2 package micro_ros_setup is unavailable"
if [[ "$(dpkg --print-architecture)" == "arm64" ]]; then
  [[ -x /usr/bin/STM32_Programmer_CLI ]] || \
    Fail "STM32CubeProgrammer CLI is unavailable"
fi

"${ENVIRONMENT_CHECK}" --check-tools yes
echo "Mentor Pi host build dependencies are ready. Build as an unprivileged user."
