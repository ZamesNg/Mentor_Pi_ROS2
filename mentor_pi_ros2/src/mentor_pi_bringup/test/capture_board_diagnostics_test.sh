#!/usr/bin/env bash

set -euo pipefail

readonly SOURCE_ROOT="${1:?source root argument is required}"
readonly CAPTURE_TOOL="${SOURCE_ROOT}/scripts/capture_board_diagnostics"

Fail() {
  echo "diagnostic capture test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  local status=0
  "$@" >/dev/null 2>&1 || status=$?
  ((status != 0)) || Fail "command unexpectedly succeeded: $*"
}

[[ -x "${CAPTURE_TOOL}" ]] || Fail "capture tool is not executable"

readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-capture.XXXXXX")"
Cleanup() {
  [[ -d "${TEST_ROOT}" ]] || return
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

readonly SYSTEM_ROOT="${TEST_ROOT}/system-root"
readonly FAKE_BIN="${TEST_ROOT}/fake-bin"
readonly ROS_LOG="${TEST_ROOT}/ros2-commands.log"
mkdir -p "${SYSTEM_ROOT}/etc/default" \
  "${SYSTEM_ROOT}/etc/mentor-pi" \
  "${SYSTEM_ROOT}/etc/udev/rules.d" \
  "${SYSTEM_ROOT}/etc/systemd/system" \
  "${SYSTEM_ROOT}/opt/mentor_pi/bin" \
  "${SYSTEM_ROOT}/opt/mentor_pi/releases/host/r1" \
  "${SYSTEM_ROOT}/opt/mentor_pi/releases/agent/a1" \
  "${SYSTEM_ROOT}/dev" \
  "${FAKE_BIN}"

cat >"${SYSTEM_ROOT}/etc/os-release" <<'EOF'
ID=ubuntu
VERSION_ID="22.04"
EOF
printf '%s\n' 'ROS_DOMAIN_ID=37' \
  >"${SYSTEM_ROOT}/etc/default/mentor-pi"
printf '%s\n' 'controller fixture' \
  >"${SYSTEM_ROOT}/etc/mentor-pi/controller.yaml"
printf '%s\n' 'sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' \
  >"${SYSTEM_ROOT}/etc/mentor-pi/runtime-image"
printf '%s\n' 'udev fixture' \
  >"${SYSTEM_ROOT}/etc/udev/rules.d/99-mentor-pi-mcu.rules"
for unit in mentor-pi-runtime.service mentor-pi-controller.target; do
  printf '[Unit]\nDescription=%s fixture\n' "${unit}" \
    >"${SYSTEM_ROOT}/etc/systemd/system/${unit}"
done
cat >"${SYSTEM_ROOT}/opt/mentor_pi/releases/host/r1/setup.bash" <<'EOF'
export MENTOR_PI_CAPTURE_SETUP_SOURCED=1
EOF
ln -s "${SYSTEM_ROOT}/opt/mentor_pi/releases/host/r1" \
  "${SYSTEM_ROOT}/opt/mentor_pi/host"
ln -s "${SYSTEM_ROOT}/opt/mentor_pi/releases/agent/a1" \
  "${SYSTEM_ROOT}/opt/mentor_pi/micro_ros_agent"
printf '%s\n' 'device fixture' >"${SYSTEM_ROOT}/dev/ttyUSB0"
ln -s "${SYSTEM_ROOT}/dev/ttyUSB0" \
  "${SYSTEM_ROOT}/dev/mentor_pi_mcu"
mkdir -p "${SYSTEM_ROOT}/opt/mentor_pi/releases/agent/a1/lib/micro_ros_agent"
cat >"${SYSTEM_ROOT}/opt/mentor_pi/releases/agent/a1/lib/micro_ros_agent/micro_ros_agent" <<'EOF'
#!/usr/bin/env bash
echo 'Usage: micro_ros_agent <transport>'
exit 1
EOF
chmod +x \
  "${SYSTEM_ROOT}/opt/mentor_pi/releases/agent/a1/lib/micro_ros_agent/micro_ros_agent"

cat >"${FAKE_BIN}/timeout" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
while [[ "${1:-}" == --* ]]; do
  shift
done
shift
exec "$@"
EOF
cat >"${FAKE_BIN}/systemctl" <<'EOF'
#!/usr/bin/env bash
printf 'systemctl fixture:'
printf ' %s' "$@"
printf '\n'
exit 0
EOF
cat >"${FAKE_BIN}/systemd-analyze" <<'EOF'
#!/usr/bin/env bash
printf 'systemd-analyze fixture:'
printf ' %s' "$@"
printf '\n'
EOF
cat >"${FAKE_BIN}/journalctl" <<'EOF'
#!/usr/bin/env bash
echo 'fixture service journal'
EOF
cat >"${FAKE_BIN}/udevadm" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' \
  'ID_VENDOR_ID=1a86' \
  'ID_MODEL_ID=55d4' \
  'ID_SERIAL_SHORT=RRCLITE-TEST' \
  'ID_PATH=pci-test-usb-1'
EOF
cat >"${FAKE_BIN}/fuser" <<'EOF'
#!/usr/bin/env bash
echo 'fixture Agent owns the device'
EOF
cat >"${FAKE_BIN}/ros2" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >>"${MENTOR_PI_CAPTURE_ROS_LOG:?}"
if [[ "$*" == 'run mentor_pi_bringup qualification_monitor '* ]]; then
  echo 'fixture qualification result'
  exit "${MENTOR_PI_CAPTURE_QUALIFICATION_STATUS:-0}"
fi
printf 'fixture ros2 output: '
printf '%s ' "$@"
printf '\n'
EOF
chmod +x "${FAKE_BIN}/"*

readonly HANDOFF="${TEST_ROOT}/handoff"
readonly REPOSITORY="${TEST_ROOT}/repository"
mkdir -p "${HANDOFF}/firmware-pid-release" \
  "${REPOSITORY}/tools" \
  "${REPOSITORY}/firmware/mentor_pi_mcu/build/stm32"
printf '%s\n' 'package_format=rrclite-board-handoff-v1' \
  >"${HANDOFF}/HANDOFF.txt"
printf '%s\n' \
  'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
  'motor_mode=PID' \
  'control_mode=CLOSED_LOOP' \
  >"${HANDOFF}/firmware-pid-release/BUILD-MODE.txt"
printf '%s\n' 'elf_sha256=fixture' \
  >"${HANDOFF}/firmware-pid-release/BUILD-METADATA.txt"
printf '%s\n' 'fixture elf' \
  >"${HANDOFF}/firmware-pid-release/mentor_pi_mcu-firmware-pid-release.elf"
(
  cd "${HANDOFF}/firmware-pid-release"
  sha256sum BUILD-MODE.txt BUILD-METADATA.txt \
    mentor_pi_mcu-firmware-pid-release.elf \
    >SHA256SUMS
)
(
  cd "${HANDOFF}"
  sha256sum HANDOFF.txt firmware-pid-release/BUILD-MODE.txt \
    firmware-pid-release/BUILD-METADATA.txt \
    firmware-pid-release/mentor_pi_mcu-firmware-pid-release.elf \
    firmware-pid-release/SHA256SUMS >SHA256SUMS
)
cp "${HANDOFF}/firmware-pid-release/mentor_pi_mcu-firmware-pid-release.elf" \
  "${REPOSITORY}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
cp "${HANDOFF}/firmware-pid-release/BUILD-METADATA.txt" \
  "${REPOSITORY}/firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt"
cat >"${REPOSITORY}/tools/verify_firmware_artifact.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ "$1" == 'PID' ]]
echo 'fixture authoritative PID artifact verified'
EOF
chmod +x "${REPOSITORY}/tools/verify_firmware_artifact.sh"

ExpectFailure "${CAPTURE_TOOL}"
ExpectFailure "${CAPTURE_TOOL}" --output relative

readonly OUTPUT_NONE="${TEST_ROOT}/evidence-none"
PATH="${FAKE_BIN}:${PATH}" \
MENTOR_PI_CAPTURE_TEST_ROOT="${SYSTEM_ROOT}" \
MENTOR_PI_CAPTURE_ROS_LOG="${ROS_LOG}" \
  "${CAPTURE_TOOL}" \
    --output "${OUTPUT_NONE}" \
    --handoff-directory "${HANDOFF}" \
    --repository-root "${REPOSITORY}"

grep -Fqx 'result=CAPTURE_COMPLETE' "${OUTPUT_NONE}/SUMMARY.txt" ||
  Fail "successful capture summary is missing"
grep -Fqx 'serial_transport_opened=0' "${OUTPUT_NONE}/SUMMARY.txt" ||
  Fail "capture does not attest that the serial transport stayed closed"
grep -Fqx 'ros_command_published=0' "${OUTPUT_NONE}/SUMMARY.txt" ||
  Fail "capture does not attest that no ROS command was published"
grep -Fqx 'qualification_mode=none' "${OUTPUT_NONE}/SUMMARY.txt" ||
  Fail "default capture unexpectedly ran qualification"
grep -Fq 'ID_SERIAL_SHORT=RRCLITE-TEST' \
  "${OUTPUT_NONE}/usb-identity.txt" ||
  Fail "USB identity was not captured"
grep -Fq 'fixture authoritative PID artifact verified' \
  "${OUTPUT_NONE}/authoritative-firmware-verification.txt" ||
  Fail "authoritative artifact verification was not captured"
(
  cd "${OUTPUT_NONE}"
  sha256sum --check SHA256SUMS >/dev/null
)
[[ -f "${OUTPUT_NONE}.tar.gz" && -f "${OUTPUT_NONE}.tar.gz.sha256" ]] ||
  Fail "capture archive or archive digest is missing"
(
  cd "${TEST_ROOT}"
  sha256sum --check "$(basename "${OUTPUT_NONE}.tar.gz.sha256")" >/dev/null
)
if grep -Eq '(^|[[:space:]])(topic[[:space:]]+pub|service[[:space:]]+call)([[:space:]]|$)' \
    "${ROS_LOG}"; then
  Fail "diagnostic capture invoked a ROS publisher or service"
fi
if grep -Fq 'qualification_monitor' "${ROS_LOG}"; then
  Fail "default diagnostic capture ran qualification"
fi
ExpectFailure "${CAPTURE_TOOL}" --output "${OUTPUT_NONE}"

: >"${ROS_LOG}"
readonly OUTPUT_CHARACTERIZATION="${TEST_ROOT}/evidence-characterization"
PATH="${FAKE_BIN}:${PATH}" \
MENTOR_PI_CAPTURE_TEST_ROOT="${SYSTEM_ROOT}" \
MENTOR_PI_CAPTURE_ROS_LOG="${ROS_LOG}" \
  "${CAPTURE_TOOL}" \
    --output "${OUTPUT_CHARACTERIZATION}" \
    --qualification imu-characterization \
    --qualification-duration-sec 1
grep -Fq \
  'run mentor_pi_bringup qualification_monitor --ros-args -p duration_sec:=1.0 -p imu_characterization_mode:=true' \
  "${ROS_LOG}" ||
  Fail "characterization mode did not invoke the exact read-only monitor"
if grep -Fq 'publish_zero_motor_commands' "${ROS_LOG}"; then
  Fail "diagnostic capture enabled the qualification command publisher"
fi
grep -Fqx 'ros_command_published=0' \
  "${OUTPUT_CHARACTERIZATION}/SUMMARY.txt" ||
  Fail "qualification capture lost its no-command attestation"
grep -Fqx 'qualification_exit_status=0' \
  "${OUTPUT_CHARACTERIZATION}/SUMMARY.txt" ||
  Fail "passing qualification status was not recorded"

: >"${ROS_LOG}"
readonly OUTPUT_FAILURE="${TEST_ROOT}/evidence-failure"
failure_status=0
PATH="${FAKE_BIN}:${PATH}" \
MENTOR_PI_CAPTURE_TEST_ROOT="${SYSTEM_ROOT}" \
MENTOR_PI_CAPTURE_ROS_LOG="${ROS_LOG}" \
MENTOR_PI_CAPTURE_QUALIFICATION_STATUS=9 \
  "${CAPTURE_TOOL}" \
    --output "${OUTPUT_FAILURE}" \
    --qualification strict \
    --qualification-duration-sec 1 >/dev/null 2>&1 || failure_status=$?
[[ "${failure_status}" == "1" ]] ||
  Fail "failed qualification did not return capture status 1"
grep -Fqx 'result=CAPTURE_COMPLETE_WITH_FAILURE' \
  "${OUTPUT_FAILURE}/SUMMARY.txt" ||
  Fail "failed qualification was not prominent in the summary"
grep -Fqx 'qualification_exit_status=9' \
  "${OUTPUT_FAILURE}/SUMMARY.txt" ||
  Fail "failed qualification exit status was not retained"
[[ -f "${OUTPUT_FAILURE}.tar.gz" ]] ||
  Fail "qualification failure did not preserve a shareable archive"

printf '%s\n' 'ROS_DOMAIN_ID=999' \
  >"${SYSTEM_ROOT}/etc/default/mentor-pi"
: >"${ROS_LOG}"
readonly OUTPUT_BAD_IDENTITY="${TEST_ROOT}/evidence-bad-identity"
bad_identity_status=0
PATH="${FAKE_BIN}:${PATH}" \
MENTOR_PI_CAPTURE_TEST_ROOT="${SYSTEM_ROOT}" \
MENTOR_PI_CAPTURE_ROS_LOG="${ROS_LOG}" \
  "${CAPTURE_TOOL}" --output "${OUTPUT_BAD_IDENTITY}" \
    >/dev/null 2>&1 || bad_identity_status=$?
[[ "${bad_identity_status}" == "1" ]] ||
  Fail "invalid production ROS identity did not fail capture"
[[ ! -s "${ROS_LOG}" ]] ||
  Fail "invalid production ROS identity still invoked ros2"
[[ -f "${OUTPUT_BAD_IDENTITY}/ros-capture-skipped.txt" ]] ||
  Fail "invalid production identity lacks an actionable skip artifact"
[[ -f "${OUTPUT_BAD_IDENTITY}.tar.gz" ]] ||
  Fail "prerequisite failure did not preserve a shareable archive"

echo "diagnostic capture tests passed"
