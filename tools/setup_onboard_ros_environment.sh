#!/usr/bin/env bash

# Source this file from an Ubuntu 22.04 shell after a conventional colcon build.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source tools/setup_onboard_ros_environment.sh" >&2
  exit 2
fi

_mentor_pi_fail() {
  echo "Onboard ROS environment error: $*" >&2
  return 1
}

_mentor_pi_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return
_mentor_pi_project_root="$(cd "${_mentor_pi_script_dir}/.." && pwd)" || return
_mentor_pi_install_prefix="${MENTOR_PI_NATIVE_INSTALL_PREFIX:-${_mentor_pi_project_root}/mentor_pi_ros2/install}"

grep -Eq '^ID=ubuntu$' /etc/os-release || \
  _mentor_pi_fail "the onboard computer must run Ubuntu" || return
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  _mentor_pi_fail "the onboard computer must run Ubuntu 22.04" || return
[[ -r /opt/ros/humble/setup.bash ]] || \
  _mentor_pi_fail "ROS 2 Humble is missing" || return
[[ -r "${_mentor_pi_install_prefix}/setup.bash" ]] || \
  _mentor_pi_fail "run colcon build from mentor_pi_ros2 first" || return
MENTOR_PI_NATIVE_INSTALL_PREFIX="${_mentor_pi_install_prefix}" \
  "${_mentor_pi_script_dir}/onboard_colcon_state.sh" verify >/dev/null || return

_mentor_pi_agent_prefix="$("${_mentor_pi_script_dir}/build_agent.sh" --print-output)" || return
_mentor_pi_agent_executable="${_mentor_pi_agent_prefix}/lib/micro_ros_agent/micro_ros_agent"
[[ -r "${_mentor_pi_agent_prefix}/local_setup.bash" && \
   -x "${_mentor_pi_agent_executable}" ]] || \
  _mentor_pi_fail "run make agent first" || return
"${_mentor_pi_script_dir}/verify_firmware_artifact.sh" PID \
  "${_mentor_pi_project_root}" >/dev/null || return

_mentor_pi_had_nounset=0
case "$-" in
  *u*) _mentor_pi_had_nounset=1 ;;
esac
set +u
source /opt/ros/humble/setup.bash
source "${_mentor_pi_agent_prefix}/local_setup.bash"
source "${_mentor_pi_install_prefix}/setup.bash"
if [[ "${_mentor_pi_had_nounset}" == "1" ]]; then
  set -u
else
  set +u
fi

export MENTOR_PI_NATIVE_INSTALL_PREFIX="${_mentor_pi_install_prefix}"
export MENTOR_PI_DEVELOPMENT_RUNTIME=1
export MENTOR_PI_PROJECT_ROOT="${_mentor_pi_project_root}"
export MENTOR_PI_FIRMWARE_VERIFIER="${_mentor_pi_script_dir}/verify_firmware_artifact.sh"
export MENTOR_PI_HOST_PREFIX="${_mentor_pi_install_prefix}"
export MENTOR_PI_AGENT_PREFIX="${_mentor_pi_agent_prefix}"
export MENTOR_PI_AGENT_EXECUTABLE="${_mentor_pi_agent_executable}"
export MENTOR_PI_RRCLITE_AUTORESET=1

unset _mentor_pi_script_dir _mentor_pi_project_root _mentor_pi_install_prefix \
  _mentor_pi_agent_prefix _mentor_pi_agent_executable
unset _mentor_pi_had_nounset
unset -f _mentor_pi_fail
