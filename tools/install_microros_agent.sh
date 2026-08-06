#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly INSTALL_STATE_VALIDATOR="${SCRIPT_DIR}/verify_microros_agent_install_state.sh"
readonly INSTALL_IDLE_GUARD="${SCRIPT_DIR}/require_microros_agent_install_idle.sh"
readonly AGENT_SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"

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

[[ -f "${AGENT_SOURCE_LOCK}" && ! -L "${AGENT_SOURCE_LOCK}" ]] ||
  Fail "Agent source lock is missing or symbolic: ${AGENT_SOURCE_LOCK}"
readonly LOCK_FORMAT="$(ReadLockValue format)"
readonly AGENT_REPOSITORY="$(ReadLockValue agent_repository)"
readonly AGENT_COMMIT="$(ReadLockValue agent_commit)"
readonly MSGS_REPOSITORY="$(ReadLockValue messages_repository)"
readonly MSGS_COMMIT="$(ReadLockValue messages_commit)"
[[ "${LOCK_FORMAT}" == "mentor-pi-micro-ros-agent-source-lock-v1" ]] ||
  Fail "unsupported Agent source lock format"
[[ "${AGENT_REPOSITORY}" == \
    "https://github.com/micro-ROS/micro-ROS-Agent.git" ]] ||
  Fail "unexpected Agent repository in source lock"
[[ "${MSGS_REPOSITORY}" == \
    "https://github.com/micro-ROS/micro_ros_msgs.git" ]] ||
  Fail "unexpected message repository in source lock"
[[ "${AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ ]] ||
  Fail "Agent commit is not a lowercase 40-hex SHA"
[[ "${MSGS_COMMIT}" =~ ^[0-9a-f]{40}$ ]] ||
  Fail "message commit is not a lowercase 40-hex SHA"

readonly BUILD_ROOT="/opt/mentor_pi/build/micro_ros_agent_${AGENT_COMMIT}"
readonly SOURCE_ROOT="${BUILD_ROOT}/src"
readonly INSTALL_ROOT="${BUILD_ROOT}/install"
readonly ACTIVE_ROOT="/opt/mentor_pi/micro_ros_agent"
readonly WRAPPER_TARGET="/opt/mentor_pi/bin/mentor_pi_micro_ros_agent"

if [[ "$(id -u)" != "0" ]]; then
  Fail "run this installer as root on Ubuntu 24.04"
fi
if [[ ! -x "${INSTALL_IDLE_GUARD}" ]]; then
  Fail "Agent install idle guard is missing at ${INSTALL_IDLE_GUARD}"
fi
"${INSTALL_IDLE_GUARD}"
if [[ ! -r "${ROS_SETUP}" ]]; then
  Fail "ROS 2 Jazzy is missing at ${ROS_SETUP}"
fi
if [[ ! -x "${INSTALL_STATE_VALIDATOR}" ]]; then
  Fail "Agent install-state validator is missing at ${INSTALL_STATE_VALIDATOR}"
fi
if [[ -e "${ACTIVE_ROOT}" && ! -L "${ACTIVE_ROOT}" ]]; then
  Fail "${ACTIVE_ROOT} exists and is not a managed symbolic link"
fi

readonly HOST_ARCHITECTURE="$(dpkg --print-architecture)"
"${INSTALL_STATE_VALIDATOR}" \
  --os-release /etc/os-release \
  --architecture "${HOST_ARCHITECTURE}"

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
rosdep update --rosdistro jazzy

CloneAndVerify() {
  local repository="$1"
  local commit="$2"
  local destination="$3"
  if [[ ! -d "${destination}/.git" ]]; then
    if [[ -e "${destination}" ]]; then
      Fail "refusing to replace non-Git path ${destination}"
    fi
    git init "${destination}"
    git -C "${destination}" remote add origin "${repository}"
    git -C "${destination}" fetch --depth 1 origin "${commit}"
    git -C "${destination}" checkout --detach FETCH_HEAD
  fi
  local actual_commit
  actual_commit="$(git -C "${destination}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${commit}" ]]; then
    Fail "revision mismatch at ${destination}: ${actual_commit}"
  fi
}

install -d -m 0755 "${SOURCE_ROOT}"
CloneAndVerify "${AGENT_REPOSITORY}" "${AGENT_COMMIT}" \
  "${SOURCE_ROOT}/micro-ROS-Agent"
CloneAndVerify "${MSGS_REPOSITORY}" "${MSGS_COMMIT}" \
  "${SOURCE_ROOT}/micro_ros_msgs"
"${INSTALL_STATE_VALIDATOR}" \
  --os-release /etc/os-release \
  --architecture "${HOST_ARCHITECTURE}" \
  --repository "${SOURCE_ROOT}/micro-ROS-Agent" \
  --origin "${AGENT_REPOSITORY}" \
  --commit "${AGENT_COMMIT}" \
  --repository "${SOURCE_ROOT}/micro_ros_msgs" \
  --origin "${MSGS_REPOSITORY}" \
  --commit "${MSGS_COMMIT}"

# The sourced scripts are upstream build environment only; they are not part
# of the running transport path.
set +u
source "${ROS_SETUP}"
set -u
rosdep install --rosdistro jazzy --from-paths "${SOURCE_ROOT}" \
  --ignore-src --as-root pip:false -y

cd "${BUILD_ROOT}"
colcon --log-base log build \
  --merge-install \
  --base-paths "${SOURCE_ROOT}" \
  --build-base build \
  --install-base "${INSTALL_ROOT}" \
  --packages-up-to micro_ros_agent \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

test -x "${INSTALL_ROOT}/lib/micro_ros_agent/micro_ros_agent"
ln -sfn "${INSTALL_ROOT}" "${ACTIVE_ROOT}"
install -d -m 0755 /opt/mentor_pi/bin
install -m 0755 \
  "${PROJECT_ROOT}/src/mentor_pi_bringup/scripts/run_micro_ros_agent" \
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
echo "Installed pinned Jazzy micro-ROS Agent: ${WRAPPER_TARGET}"
