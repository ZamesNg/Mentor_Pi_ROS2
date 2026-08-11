#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly WORK_ROOT="${COMPONENT_ROOT}/build/native"
readonly INSTALL_ROOT="${WORK_ROOT}/install"
readonly EXECUTABLE="${INSTALL_ROOT}/lib/micro_ros_agent/micro_ros_agent"
readonly METADATA="${INSTALL_ROOT}/AGENT-BUILD-METADATA.txt"
readonly SOURCE_LOCK="${COMPONENT_ROOT}/sources.lock"
readonly PATCH="${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch"

setup_only=0
[[ "$#" -le 1 ]] || { echo "Usage: build.sh [--setup]" >&2; exit 2; }
if [[ "${1:-}" == --setup ]]; then
  setup_only=1
elif [[ "$#" == 1 ]]; then
  echo "Usage: build.sh [--setup]" >&2
  exit 2
fi

"${SCRIPT_DIR}/check_environment.sh" >/dev/null
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export ROS_DISTRO=humble

mkdir -p "${WORK_ROOT}"
"${SCRIPT_DIR}/build_from_lock.sh" fetch --work-root "${WORK_ROOT}"
if ((setup_only == 1)); then
  rosdep install --rosdistro humble --from-paths "${WORK_ROOT}/src" \
    --ignore-src --as-root pip:false -y
  echo "Pinned Agent sources and dependencies are ready."
  exit 0
fi

"${SCRIPT_DIR}/build_from_lock.sh" build --work-root "${WORK_ROOT}" \
  --dependency-mode preinstalled
[[ -x "${EXECUTABLE}" && ! -L "${EXECUTABLE}" ]] || {
  echo "Agent executable was not produced" >&2
  exit 1
}

Sha256() {
  sha256sum "$1" | awk '{print $1}'
}

readonly ARCHITECTURE="$(dpkg --print-architecture)"
readonly EXECUTABLE_SHA="$(Sha256 "${EXECUTABLE}")"
readonly TEMP_METADATA="${METADATA}.tmp.$$"
printf '%s\n' \
  'format=mentor-pi-native-agent-v1' \
  'ubuntu=22.04' \
  'ros_distro=humble' \
  "architecture=${ARCHITECTURE}" \
  "source_lock_sha256=$(Sha256 "${SOURCE_LOCK}")" \
  "rrclite_patch_sha256=$(Sha256 "${PATCH}")" \
  "executable_sha256=${EXECUTABLE_SHA}" \
  >"${TEMP_METADATA}"
mv "${TEMP_METADATA}" "${METADATA}"
echo "Native Agent build ready: ${EXECUTABLE}"
