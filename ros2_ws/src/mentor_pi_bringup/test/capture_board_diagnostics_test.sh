#!/usr/bin/env bash

set -euo pipefail

readonly SOURCE_ROOT="$1"
readonly CAPTURE="${SOURCE_ROOT}/scripts/capture_board_diagnostics"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT
readonly SYSTEM_ROOT="${TEST_ROOT}/system"
readonly FAKE_BIN="${TEST_ROOT}/bin"
readonly OUTPUT="${TEST_ROOT}/capture"
readonly FAKE_AGENT="${TEST_ROOT}/micro_ros_agent"

mkdir -p "${FAKE_BIN}" "${SYSTEM_ROOT}/etc/systemd/system" \
  "${SYSTEM_ROOT}/etc/udev/rules.d" "${SYSTEM_ROOT}/dev"
printf '%s\n' '[Unit]' 'Description=test Agent' \
  >"${SYSTEM_ROOT}/etc/systemd/system/mentor-pi-agent.service"
printf '%s\n' 'SUBSYSTEM=="tty"' \
  >"${SYSTEM_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules"
: >"${SYSTEM_ROOT}/dev/mentor_pi_mcu"

printf '%s\n' '#!/usr/bin/env bash' 'exit 0' >"${FAKE_BIN}/ros2"
printf '%s\n' '#!/usr/bin/env bash' 'echo "micro_ros_agent fixture"' \
  >"${FAKE_AGENT}"
chmod +x "${FAKE_BIN}/ros2" "${FAKE_AGENT}"

PATH="${FAKE_BIN}:${PATH}" \
MENTOR_PI_CAPTURE_TEST_ROOT="${SYSTEM_ROOT}" \
MENTOR_PI_DEVELOPMENT_RUNTIME=1 \
MENTOR_PI_AGENT_EXECUTABLE="${FAKE_AGENT}" \
ROS_DOMAIN_ID=0 \
  "${CAPTURE}" --output "${OUTPUT}"

grep -Fqx 'capture_schema=mentor-pi-board-diagnostics-v1' \
  "${OUTPUT}/SUMMARY.txt"
grep -Fqx 'result=CAPTURE_COMPLETE' "${OUTPUT}/SUMMARY.txt"
grep -Fq 'mentor-pi-agent.service' "${OUTPUT}/systemd-status.txt"
! grep -R -Fq 'mentor-pi-runtime.service' "${OUTPUT}"
(cd "${OUTPUT}" && sha256sum --check SHA256SUMS >/dev/null)
(cd "${TEST_ROOT}" && sha256sum --check capture.tar.gz.sha256 >/dev/null)

echo "native board diagnostic capture test passed"
