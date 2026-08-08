#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly INSTALL_STATE_VALIDATOR="${SCRIPT_DIR}/verify_microros_agent_install_state.sh"
readonly INSTALL_IDLE_GUARD="${SCRIPT_DIR}/require_microros_agent_install_idle.sh"
readonly AGENT_BUILD_HELPER="${SCRIPT_DIR}/build_microros_agent_from_lock.sh"
readonly AGENT_SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"

Fail() {
  echo "micro-ROS Agent install error: $*" >&2
  exit 1
}

ReadLockValue() {
  local key="$1"
  local line=""
  line="$(grep -E "^${key}=" "${AGENT_SOURCE_LOCK}" || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] ||
    Fail "Agent source lock must contain exactly one ${key}= entry"
  printf '%s' "${line#*=}"
}

[[ "$#" -eq 0 ]] || Fail "this installer accepts no arguments"

[[ -f "${AGENT_SOURCE_LOCK}" && ! -L "${AGENT_SOURCE_LOCK}" ]] ||
  Fail "Agent source lock is missing or symbolic: ${AGENT_SOURCE_LOCK}"
readonly LOCK_FORMAT="$(ReadLockValue format)"
readonly LOCK_ROS_DISTRO="$(ReadLockValue ros_distro)"
readonly AGENT_REPOSITORY="$(ReadLockValue agent_repository)"
readonly AGENT_COMMIT="$(ReadLockValue agent_commit)"
readonly MSGS_REPOSITORY="$(ReadLockValue messages_repository)"
readonly MSGS_COMMIT="$(ReadLockValue messages_commit)"
readonly XRCE_AGENT_REPOSITORY="$(ReadLockValue xrce_agent_repository)"
readonly XRCE_AGENT_COMMIT="$(ReadLockValue xrce_agent_commit)"
[[ "${LOCK_FORMAT}" == "mentor-pi-micro-ros-agent-source-lock-v2" ]] ||
  Fail "unsupported Agent source lock format"
[[ "${LOCK_ROS_DISTRO}" == "humble" ]] ||
  Fail "Agent source lock targets a different ROS distribution"
[[ "${AGENT_REPOSITORY}" == \
    "https://github.com/micro-ROS/micro-ROS-Agent.git" ]] ||
  Fail "unexpected Agent repository in source lock"
[[ "${MSGS_REPOSITORY}" == \
    "https://github.com/micro-ROS/micro_ros_msgs.git" ]] ||
  Fail "unexpected message repository in source lock"
[[ "${XRCE_AGENT_REPOSITORY}" == \
    "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" ]] ||
  Fail "unexpected XRCE Agent repository in source lock"
[[ "${AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ ]] ||
  Fail "Agent commit is not a lowercase 40-hex SHA"
[[ "${MSGS_COMMIT}" =~ ^[0-9a-f]{40}$ ]] ||
  Fail "message commit is not a lowercase 40-hex SHA"
[[ "${XRCE_AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ ]] ||
  Fail "XRCE Agent commit is not a lowercase 40-hex SHA"

readonly BUILD_ROOT="/opt/mentor_pi/build/micro_ros_agent_${AGENT_COMMIT}"
readonly SOURCE_ROOT="${BUILD_ROOT}/src"
readonly INSTALL_ROOT="${BUILD_ROOT}/install"
readonly ACTIVE_ROOT="/opt/mentor_pi/micro_ros_agent"
readonly WRAPPER_TARGET="/opt/mentor_pi/bin/mentor_pi_micro_ros_agent"

if [[ "$(id -u)" != "0" ]]; then
  Fail "run this installer as root on Ubuntu 22.04"
fi
if [[ ! -x "${INSTALL_IDLE_GUARD}" ]]; then
  Fail "Agent install idle guard is missing at ${INSTALL_IDLE_GUARD}"
fi
"${INSTALL_IDLE_GUARD}"
if [[ ! -r "${ROS_SETUP}" ]]; then
  Fail "ROS 2 Humble is missing at ${ROS_SETUP}"
fi
if [[ ! -x "${INSTALL_STATE_VALIDATOR}" ]]; then
  Fail "Agent install-state validator is missing at ${INSTALL_STATE_VALIDATOR}"
fi
if [[ ! -x "${AGENT_BUILD_HELPER}" ]]; then
  Fail "Agent source-build helper is missing at ${AGENT_BUILD_HELPER}"
fi
if [[ -e "${ACTIVE_ROOT}" && ! -L "${ACTIVE_ROOT}" ]]; then
  Fail "${ACTIVE_ROOT} exists and is not a managed symbolic link"
fi

readonly HOST_ARCHITECTURE="$(dpkg --print-architecture)"
"${INSTALL_STATE_VALIDATOR}" \
  --os-release /etc/os-release \
  --architecture "${HOST_ARCHITECTURE}"

# Generated ROS setup hooks are not guaranteed to be nounset-clean. Source the
# deployment environment exactly once before the first package/source mutation;
# the resulting environment remains active for rosdep and colcon below.
set +u
if ! source "${ROS_SETUP}"; then
  set -u
  Fail "could not source ROS 2 Humble setup at ${ROS_SETUP}"
fi
set -u
[[ "${ROS_DISTRO:-}" == "humble" ]] || \
  Fail "ROS setup must identify ROS_DISTRO=humble"

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  libcurl4-openssl-dev \
  python3-colcon-common-extensions \
  python3-rosdep

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro humble

install -d -m 0755 "${SOURCE_ROOT}"
"${AGENT_BUILD_HELPER}" fetch --work-root "${BUILD_ROOT}"
"${AGENT_BUILD_HELPER}" build --work-root "${BUILD_ROOT}" \
  --dependency-mode install

test -x "${INSTALL_ROOT}/lib/micro_ros_agent/micro_ros_agent"
ln -sfn "${INSTALL_ROOT}" "${ACTIVE_ROOT}"
install -d -m 0755 /opt/mentor_pi/bin
install -m 0755 \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/scripts/run_micro_ros_agent" \
  "${WRAPPER_TARGET}"

# Upstream prints its usage text for --help but intentionally returns 1 because
# Agent::create() reports that no transport was selected. Accept that documented
# CLI result while still rejecting wrapper/loader failures (normally 126/127)
# and an executable that does not emit the expected usage banner.
smoke_status=0
smoke_output_file="$(mktemp)"
trap 'rm -f "${smoke_output_file}"' EXIT
set +e
"${WRAPPER_TARGET}" --help >"${smoke_output_file}" 2>&1
smoke_status=$?
set -e
if [[ "${smoke_status}" != "0" && "${smoke_status}" != "1" ]]; then
  Fail "installed Agent failed to load (status ${smoke_status})"
fi
if ! grep -Fq "Usage:" "${smoke_output_file}"; then
  Fail "installed Agent did not emit its expected usage banner"
fi
rm -f "${smoke_output_file}"
trap - EXIT
echo "Installed pinned Humble micro-ROS Agent: ${WRAPPER_TARGET}"
