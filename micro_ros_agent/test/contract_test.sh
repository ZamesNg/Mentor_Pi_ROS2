#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${TEST_DIR}/.." && pwd)"

bash -n "${COMPONENT_ROOT}"/tools/*.sh
grep -Fqx 'ros_distro=humble' "${COMPONENT_ROOT}/sources.lock"
grep -Fq 'MENTOR_PI_RRCLITE_AUTORESET' \
  "${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch"
grep -Fq 'User=mentor-pi' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fq '/opt/mentor_pi/agent/current/lib/micro_ros_agent/micro_ros_agent' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fqx 'Restart=always' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fqx 'StartLimitIntervalSec=0' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fq 'ATTRS{idVendor}=="1a86"' \
  "${COMPONENT_ROOT}/udev/99-mentor-pi-mcu.rules.in"
if rg -n 'docker (run|build|exec|pull|load)|mentor-pi-runtime|configuration_supervisor' \
    "${COMPONENT_ROOT}/Makefile" "${COMPONENT_ROOT}/tools" \
    "${COMPONENT_ROOT}/systemd"; then
  echo "Agent component contains forbidden runtime/container coupling" >&2
  exit 1
fi
grep -Fq '/.dockerenv' "${COMPONENT_ROOT}/tools/install_service.sh" || {
  echo "Agent service installation does not reject the Dev Container" >&2
  exit 1
}

executable="${COMPONENT_ROOT}/build/native/install/lib/micro_ros_agent/micro_ros_agent"
metadata="${COMPONENT_ROOT}/build/native/install/AGENT-BUILD-METADATA.txt"
if [[ -e "${executable}" || -e "${metadata}" ]]; then
  [[ -x "${executable}" && -f "${metadata}" ]] || {
    echo "Agent build output is incomplete" >&2
    exit 1
  }
  expected="$(sed -n 's/^executable_sha256=//p' "${metadata}")"
  [[ "${expected}" == "$(sha256sum "${executable}" | awk '{print $1}')" ]] || {
    echo "Agent executable does not match metadata" >&2
    exit 1
  }
fi
echo "micro-ROS Agent component contract passed."
