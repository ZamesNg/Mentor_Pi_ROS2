#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly TEST_ROOT="$(mktemp -d)"

Cleanup() {
  [[ -d "${TEST_ROOT}" ]] || return
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Tutorial action contract test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output=""
  if output="$("$@" </dev/null 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "expected '${expected}' in failure: ${output}"
}

grep -Fqx 'PORT ?= /dev/mentor_pi_mcu' "${PROJECT_ROOT}/Makefile" || \
  Fail "Makefile does not default to the stable MCU alias"
grep -Fqx 'ROS_DOMAIN_ID ?= 0' "${PROJECT_ROOT}/Makefile" || \
  Fail "Makefile does not default to ROS domain 0"

for target in serial-setup flash start start-hardware start-mecanum start-ackermann \
    passive-check peripheral-smoke characterize-board \
    release-software-gates release-onboard-gates; do
  grep -Eq "^[^:#]*\\b${target}([ :]|$)" "${PROJECT_ROOT}/Makefile" || \
    Fail "Makefile target is missing: ${target}"
done
grep -Fq 'RUN_ONBOARD_DOCKER_GATES' "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "RDK Docker gates do not require their distinct acknowledgement"
grep -Fq 'run_fuzz_smoke.sh' "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "normal-computer software gates lost fuzzing"
grep -Fq 'onboard Docker gates require the detected RDK X5' \
  "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "reduced onboard gates do not require RDK X5 device-tree detection"
grep -Fq 'full software gates must run on the normal computer' \
  "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "full software gates do not reject the RDK X5 profile"
onboard_gate_block="$(sed -n '/release-onboard-gates)/,/;;/p' \
  "${SCRIPT_DIR}/tutorial_action.sh")"
[[ "${onboard_gate_block}" != *'run_fuzz_smoke.sh'* ]] || \
  Fail "RDK Docker gates must not invoke fuzzing"
[[ "${onboard_gate_block}" != *'run_coverage_tests.sh'* ]] || \
  Fail "RDK Docker gates must not require the normal-computer coverage toolchain"

grep -Fq './tools/verify_firmware_artifact.sh PID' \
  "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "PID start action does not require the PID artifact"
grep -E 'guided_flash\.sh.*"\$\(PORT\)"' "${PROJECT_ROOT}/Makefile" >/dev/null || \
  Fail "Makefile flash target must route through interactive guided_flash.sh, not the low-level helper"
grep -Fq 'start-hardware | start-mecanum | start-ackermann)' \
  "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "hardware start actions are not handled together"
grep -Fq -- '--vehicle-config "${vehicle_config}"' \
  "${SCRIPT_DIR}/tutorial_action.sh" || \
  Fail "hardware start action does not forward its YAML profile"
if grep -Eq 'ROBOT_NAME|--robot-name|--hardware-mode|MENTOR_PI_(ROBOT_NAME|HARDWARE_MODE)' \
    "${PROJECT_ROOT}/Makefile" "${SCRIPT_DIR}/tutorial_action.sh" \
    "${SCRIPT_DIR}/run_runtime.sh"; then
  Fail "runtime retains a direct robot-name or vehicle-type override"
fi
grep -Fq -- \
  '--volume "${resolved_vehicle_config}:/opt/mentor_pi/vehicle.yaml:ro"' \
  "${SCRIPT_DIR}/run_runtime.sh" || \
  Fail "Docker runtime does not mount the selected YAML profile read-only"
grep -Fq '/mentor_pi/motors/set_pid' \
  "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "controller readiness omits the PID service"
grep -Fq 'all 21 MCU endpoints and heartbeat' \
  "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "controller readiness reports a stale endpoint count"
if grep -ERq 'source [^`]*setup[.](bash|zsh)|setup_onboard_ros_environment' \
    "${PROJECT_ROOT}/docs/tutorials"; then
  Fail "Docker tutorials expose a native ROS environment command"
fi
grep -Fq '/usr/bin/zsh -d -i' "${SCRIPT_DIR}/open_runtime_shell.sh" || \
  Fail "make shell no longer opens the runtime zsh configuration"

readonly CONTROLLER_LAUNCH="${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/launch/controller.launch.py"
[[ -f "${CONTROLLER_LAUNCH}" ]] || Fail "Python controller launch is missing"
[[ ! -e "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/launch/controller.launch.xml" ]] || \
  Fail "legacy XML controller launch still exists"
grep -Fq 'RRCLITE_RUNTIME_ACK' "${CONTROLLER_LAUNCH}" || \
  Fail "direct launch does not require the runtime acknowledgement"
grep -Fq '_validate_development_artifact' "${CONTROLLER_LAUNCH}" || \
  Fail "direct development launch does not validate the PID artifact"
grep -Fq 'OnProcessExit' "${CONTROLLER_LAUNCH}" || \
  Fail "controller launch does not couple process lifecycles"
if grep -Fq 'validate_runtime' "${CONTROLLER_LAUNCH}" \
    "${SCRIPT_DIR}/run_runtime.sh"; then
  Fail "operator-facing runtime still exposes a preflight bypass"
fi
if grep -Eq 'MENTOR_PI_LAUNCH_TESTING|/\.dockerenv' "${CONTROLLER_LAUNCH}"; then
  Fail "controller launch conditionally bypasses serial identity checks"
fi
if find "${PROJECT_ROOT}/mentor_pi_ros2/src" -path '*/launch/*.xml' -print -quit | \
    grep -q .; then
  Fail "ROS package launch directories still contain an XML launch file"
fi
PYTHONPYCACHEPREFIX="${TEST_ROOT}/pycache" python3 -m py_compile \
  "${CONTROLLER_LAUNCH}" \
  "${PROJECT_ROOT}"/mentor_pi_ros2/src/*/launch/*.py

ExpectFailure "unsupported tutorial action" \
  "${SCRIPT_DIR}/tutorial_action.sh" this-action-does-not-exist-xyz
ExpectFailure "does not resolve to an existing character device" \
  "${SCRIPT_DIR}/guided_flash.sh" /dev/rrclite-nonexistent-port-abc123xyz
ExpectFailure "interactive terminal" \
  "${SCRIPT_DIR}/tutorial_action.sh" start
ExpectFailure "interactive terminal" \
  "${SCRIPT_DIR}/tutorial_action.sh" characterize-board
ExpectFailure "explicit absolute path" \
  "${SCRIPT_DIR}/run_runtime.sh" --device /dev/mentor_pi_mcu \
  --ros-domain-id 0 --vehicle-config relative.yaml
ExpectFailure "VEHICLE_CONFIG must select" env \
  RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  "${SCRIPT_DIR}/tutorial_action.sh" start-hardware

mkdir -p "${TEST_ROOT}/tools"
cp "${SCRIPT_DIR}/tutorial_action.sh" \
  "${TEST_ROOT}/tools/tutorial_action.sh"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'printf "%s\n" "$*" >"${FAKE_START_LOG}"' \
  >"${TEST_ROOT}/tools/run_runtime.sh"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'printf "%s" "${ROS_DOMAIN_ID}" >"${FAKE_RUNTIME_LOG}"' \
  'printf "\t%s" "$@" >>"${FAKE_RUNTIME_LOG}"' \
  'printf "\n" >>"${FAKE_RUNTIME_LOG}"' \
  >"${TEST_ROOT}/tools/run_runtime_action.sh"
chmod +x "${TEST_ROOT}/tools/tutorial_action.sh" \
  "${TEST_ROOT}/tools/run_runtime.sh" \
  "${TEST_ROOT}/tools/run_runtime_action.sh"
env RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  VEHICLE_CONFIG=/opt/robots/robot_two.yaml \
  FAKE_START_LOG="${TEST_ROOT}/start.log" \
  "${TEST_ROOT}/tools/tutorial_action.sh" start-hardware
grep -Fqx -- \
  '--device /dev/mentor_pi_mcu --ros-domain-id 0 --vehicle-config /opt/robots/robot_two.yaml --tracking-controller none' \
  "${TEST_ROOT}/start.log" || \
  Fail "hardware start did not forward the selected YAML profile and disabled tracker"
env ROS_DOMAIN_ID=37 PERIPHERAL_SMOKE_ACK=PASSIVE_OUTPUTS_GUARDED \
  OLED_PRESENT=1 \
  FAKE_RUNTIME_LOG="${TEST_ROOT}/runtime.log" \
  "${TEST_ROOT}/tools/tutorial_action.sh" peripheral-smoke
grep -Fqx $'37\tperipheral-smoke\t1' "${TEST_ROOT}/runtime.log" || \
  Fail "readonly ROS domain was not passed to the runtime action"

docker_setup_block="$(grep -A5 -F "'set -euo pipefail" \
  "${SCRIPT_DIR}/run_runtime_action.sh")"
[[ "${docker_setup_block}" == *$'set +u\n'* && \
  "${docker_setup_block}" == *$'source /opt/ros/humble/setup.bash\n'* && \
  "${docker_setup_block}" == *'set -u'* ]] || \
  Fail "Docker runtime actions do not safely source the Humble environment"

mkdir -p "${TEST_ROOT}/bin"
printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -euo pipefail' \
  'printf "%s\n" "$*" >>"${FAKE_ROS_LOG}"' \
  'if [[ "$*" == "node list --no-daemon --spin-time 0.2" ]]; then' \
  '  printf "/mentor_pi/controller\n/mentor_pi/configuration_supervisor\n"' \
  '  exit 0' \
  'fi' \
  'if [[ "$*" == "topic list --no-daemon --spin-time 0.2" ]]; then' \
  '  printf "%s\n" /mentor_pi/battery/state /mentor_pi/bus_servos/command /mentor_pi/buttons/events /mentor_pi/buzzer/command /mentor_pi/diagnostics /mentor_pi/heartbeat /mentor_pi/imu /mentor_pi/leds/command /mentor_pi/motors/command /mentor_pi/motors/state /mentor_pi/oled/command /mentor_pi/pwm_servos/command /mentor_pi/pwm_servos/state /mentor_pi/rgb/command' \
  '  exit 0' \
  'fi' \
  'if [[ "$*" == "service list --no-daemon --spin-time 0.2" ]]; then' \
  '  printf "%s\n" /mentor_pi/battery/set_low_threshold /mentor_pi/bus_servos/configure /mentor_pi/bus_servos/get_state /mentor_pi/bus_servos/stop /mentor_pi/motors/set_model /mentor_pi/pwm_servos/set_offsets' \
  '  [[ "${FAKE_INCLUDE_PID:-0}" == "1" ]] && printf "%s\n" /mentor_pi/motors/set_pid' \
  '  exit 0' \
  'fi' \
  'if [[ "$*" == *"/mentor_pi/heartbeat"* ]]; then' \
  '  printf "sequence: 1\n"' \
  '  exit 0' \
  'fi' \
  'if [[ "$*" == *"/mentor_pi/buttons/events"* ]]; then' \
  '  count=0' \
  '  [[ ! -f "${FAKE_BUTTON_COUNT}" ]] || read -r count <"${FAKE_BUTTON_COUNT}"' \
  '  count=$((count + 1))' \
  '  printf "%s\n" "${count}" >"${FAKE_BUTTON_COUNT}"' \
  '  printf "button_id: %s\nevent: 1\n" "${count}"' \
  'fi' \
  >"${TEST_ROOT}/bin/ros2"
chmod +x "${TEST_ROOT}/bin/ros2"
ExpectFailure "services=6/7" env PATH="${TEST_ROOT}/bin:${PATH}" \
  ROS_DOMAIN_ID=37 FAKE_INCLUDE_PID=0 \
  FAKE_ROS_LOG="${TEST_ROOT}/ros.log" \
  "${SCRIPT_DIR}/run_runtime_action.sh" --inside controller-wait 1 \
  missing-pid 0
env PATH="${TEST_ROOT}/bin:${PATH}" ROS_DOMAIN_ID=37 FAKE_INCLUDE_PID=1 \
  FAKE_ROS_LOG="${TEST_ROOT}/ros.log" \
  "${SCRIPT_DIR}/run_runtime_action.sh" --inside controller-wait 2 \
  complete-graph 0 >/dev/null
env PATH="${TEST_ROOT}/bin:${PATH}" ROS_DOMAIN_ID=37 \
  FAKE_ROS_LOG="${TEST_ROOT}/ros.log" \
  FAKE_BUTTON_COUNT="${TEST_ROOT}/button.count" \
  "${SCRIPT_DIR}/run_runtime_action.sh" --inside peripheral-smoke 1 \
  >/dev/null
grep -Fq \
  'mentor_pi_interfaces/msg/RgbCommand {update_mask: 1, red: [0, 0], green: [0, 0], blue: [32, 0]}' \
  "${TEST_ROOT}/ros.log" || Fail "RGB smoke command does not match its schema"
if grep -Fq 'led_id: 3' "${TEST_ROOT}/ros.log"; then
  Fail "peripheral smoke attempted to override firmware-owned LED3"
fi
grep -Fq \
  "mentor_pi_interfaces/msg/OledCommand {update_mask: 3, line_1: 'Mentor Pi', line_2: 'PASSIVE'}" \
  "${TEST_ROOT}/ros.log" || Fail "OLED smoke command does not match its schema"
grep -Fq \
  'mentor_pi_interfaces/msg/RgbCommand {update_mask: 1, red: [0, 0], green: [0, 0], blue: [0, 0]}' \
  "${TEST_ROOT}/ros.log" || Fail "RGB cleanup does not match its schema"
grep -Fqx 2 "${TEST_ROOT}/button.count" || \
  Fail "peripheral smoke did not verify both buttons"

echo "Tutorial action contract tests passed."
