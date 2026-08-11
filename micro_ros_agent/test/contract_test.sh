#!/usr/bin/env bash

set -euo pipefail

readonly TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly COMPONENT_ROOT="$(cd "${TEST_DIR}/.." && pwd)"

bash -n "${COMPONENT_ROOT}"/tools/*.sh
grep -Eq '^find-device:' "${COMPONENT_ROOT}/Makefile"
grep -Fq 'SERIAL_ACCESS_HELPER=' \
  "${COMPONENT_ROOT}/tools/install_service.sh"
grep -Fq 'automatic CH9102F discovery' \
  "${COMPONENT_ROOT}/tools/configure_serial_access.sh"
grep -Fqx 'ros_distro=humble' "${COMPONENT_ROOT}/sources.lock"
grep -Fq 'MENTOR_PI_RRCLITE_AUTORESET' \
  "${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch"
normal_boot_sequence="$(tr '\n' ' ' <"${COMPONENT_ROOT}/patches/micro_xrce_agent_rrclite_modem_lines.patch")"
[[ "${normal_boot_sequence}" =~ bits\ =\ TIOCM_RTS\;.*TIOCMBIS.*bits\ =\ TIOCM_DTR\;.*TIOCMBIC.*milliseconds\(100\).*bits\ =\ TIOCM_RTS\;.*TIOCMBIC.*milliseconds\(100\) ]] || {
  echo "micro-ROS Agent patch lost the separate normal-boot RTS/DTR sequence" >&2
  exit 1
}
grep -Fq 'User=mentor-pi' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fq '/opt/mentor_pi/agent/current/bin/mentor-pi-agent' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fq 'source "${ROS_SETUP}"' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent"
grep -Fq 'source "${LOCAL_SETUP}"' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent"
grep -Fqx 'Restart=always' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fqx 'StartLimitIntervalSec=0' \
  "${COMPONENT_ROOT}/systemd/mentor-pi-agent.service"
grep -Fq 'ATTRS{idVendor}=="1a86"' \
  "${COMPONENT_ROOT}/udev/99-mentor-pi-mcu.rules.in"
if grep -R -n -E 'docker (run|build|exec|pull|load)|mentor-pi-runtime|configuration_supervisor' \
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
launcher="${COMPONENT_ROOT}/build/native/install/bin/mentor-pi-agent"
metadata="${COMPONENT_ROOT}/build/native/install/AGENT-BUILD-METADATA.txt"
if [[ -e "${executable}" || -e "${launcher}" || -e "${metadata}" ]]; then
  [[ -x "${executable}" && -x "${launcher}" && -f "${metadata}" ]] || {
    echo "Agent build output is incomplete" >&2
    exit 1
  }
  expected="$(sed -n 's/^executable_sha256=//p' "${metadata}")"
  [[ "${expected}" == "$(sha256sum "${executable}" | awk '{print $1}')" ]] || {
    echo "Agent executable does not match metadata" >&2
    exit 1
  }
  expected_launcher="$(sed -n 's/^launcher_sha256=//p' "${metadata}")"
  [[ "${expected_launcher}" == "$(sha256sum "${launcher}" | awk '{print $1}')" ]] || {
    echo "Agent launcher does not match metadata" >&2
    exit 1
  }
  set +e
  launcher_help="$(env -i PATH=/usr/bin:/bin HOME=/tmp \
    "${launcher}" --help 2>&1 | LC_ALL=C tr -d '\000')"
  launcher_status=$?
  set -e
  [[ "${launcher_status}" -eq 1 && "${launcher_help}" == Usage:* ]] || {
    echo "Agent launcher does not load its ROS runtime in a clean environment" >&2
    exit 1
  }
fi
echo "micro-ROS Agent component contract passed."
