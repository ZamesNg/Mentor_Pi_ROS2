#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT="$(cd "${WORKSPACE_ROOT}/.." && pwd)"
readonly VEHICLE="${1:-}"
readonly TRACKING_CONTROLLER="${2:-none}"
readonly REQUIRED_ACK="PID_FIRMWARE_ACTUATORS_PREPARED"

Fail() {
  echo "ROS application launch error: $*" >&2
  exit 1
}

[[ "${VEHICLE}" == mecanum || "${VEHICLE}" == ackermann ]] || \
  Fail "vehicle must be mecanum or ackermann"
[[ "${TRACKING_CONTROLLER}" == none || \
   "${TRACKING_CONTROLLER}" == "${VEHICLE}" ]] || \
  Fail "tracking controller must be none or match the vehicle"
[[ ! -f /.dockerenv ]] || \
  Fail "ROS applications must run on native Ubuntu 22.04, not the Dev Container"
[[ "${RRCLITE_RUNTIME_ACK:-}" == "${REQUIRED_ACK}" ]] || \
  Fail "set RUNTIME_ACK=${REQUIRED_ACK} only after the passive safety gates"
[[ -r "${WORKSPACE_ROOT}/install/setup.bash" ]] || \
  Fail "workspace is not built; run make build"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
export ROS_DISTRO=humble
export RRCLITE_RUNTIME_ACK
export MENTOR_PI_DEVELOPMENT_RUNTIME=1
export MENTOR_PI_PROJECT_ROOT="${PROJECT_ROOT}"
export MENTOR_PI_FIRMWARE_VERIFIER="${PROJECT_ROOT}/firmware/tools/verify.sh"
"${MENTOR_PI_FIRMWARE_VERIFIER}" >/dev/null

if command -v systemctl >/dev/null 2>&1 && \
    systemctl list-unit-files mentor-pi-agent.service >/dev/null 2>&1; then
  systemctl is-active --quiet mentor-pi-agent.service || \
    Fail "mentor-pi-agent.service is installed but not active"
else
  echo "Agent service state is remote or not locally available; checking the ROS graph."
fi

heartbeat="$(mktemp)"
trap 'rm -f -- "${heartbeat}"' EXIT
timeout 5 ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/heartbeat mentor_pi_interfaces/msg/Heartbeat \
  >"${heartbeat}" 2>/dev/null || \
  Fail "the Agent/firmware heartbeat is unavailable; application startup remains disarmed"
awk '$1 == "agent_session_id:" && $2 ~ /^[1-9][0-9]*$/ { found=1 } END { exit !found }' \
  "${heartbeat}" || \
  Fail "the Agent/firmware heartbeat has no live session"
rm -f -- "${heartbeat}"
trap - EXIT

exec ros2 launch mentor_pi_hardwares "${VEHICLE}.launch.py" \
  tracking_controller:="${TRACKING_CONTROLLER}"
