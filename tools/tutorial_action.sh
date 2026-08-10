#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PORT="${PORT:-/dev/mentor_pi_mcu}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
readonly HOST_PROFILE_DETECTOR="${SCRIPT_DIR}/detect_host_profile.sh"

Fail() {
  echo "Mentor Pi tutorial action failed: $*" >&2
  exit 1
}

Prompt() {
  local prompt="$1"
  local value=""
  [[ -t 0 ]] || Fail "this action requires an interactive terminal"
  printf '%b\n> ' "${prompt}" >&2
  IFS= read -r value
  printf '%s' "${value}"
}

RequireExact() {
  local current="$1" required="$2" prompt="$3"
  if [[ -z "${current}" ]]; then
    current="$(Prompt "${prompt}\nType exactly: ${required}")"
  fi
  [[ "${current}" == "${required}" ]] || \
    Fail "acknowledgement did not match ${required}"
}

RunRuntime() {
  env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    "${SCRIPT_DIR}/run_runtime_action.sh" "$@"
}

WaitForDevice() {
  local deadline=$((SECONDS + 5))
  while ((SECONDS < deadline)); do
    [[ -c "${PORT}" ]] && return 0
    sleep 0.2
  done
  Fail "stable MCU alias did not return within five seconds"
}

BoardSerial() {
  local resolved properties serial
  resolved="$(readlink -f "${PORT}")"
  properties="$(udevadm info --query=property --name="${resolved}")"
  serial="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"${properties}" | head -n 1)"
  [[ "${serial}" =~ ^[A-Za-z0-9._-]+$ ]] || \
    Fail "could not obtain a safe CH9102F board serial"
  printf '%s' "${serial}"
}

ValidateInteger() {
  local value="$1" minimum="$2" maximum="$3" name="$4"
  [[ "${value}" =~ ^-?[0-9]+$ ]] || Fail "${name} must be an integer"
  ((value >= minimum && value <= maximum)) || \
    Fail "${name} must be in [${minimum}, ${maximum}]"
}

TrimAsciiWhitespace() {
  local value="$1"
  value="${value#"${value%%[!$' \t\r\n']*}"}"
  value="${value%"${value##*[!$' \t\r\n']}"}"
  printf '%s' "${value}"
}

OledPresence() {
  local value="${OLED_PRESENT:-}"
  if [[ -z "${value}" ]]; then
    value="$(Prompt 'Is the 128x32 OLED physically installed? Enter y or n:')"
  fi
  case "${value}" in
    y | Y | yes | YES | 1) printf '1' ;;
    n | N | no | NO | 0) printf '0' ;;
    *) Fail "OLED answer must be y or n" ;;
  esac
}

Campaign() {
  local mode="$1"
  RequireExact "${CAMPAIGN_FIXTURE_ACK:-}" \
    PERIPHERALS_DISCONNECTED_OR_GUARDED \
    "Confirm every peripheral is disconnected or physically guarded."
  local fixture_revision="${FIXTURE_REVISION:-}"
  local bus_id="${CAMPAIGN_BUS_ID:-}"
  local bus_hold="${CAMPAIGN_BUS_HOLD:-}"
  local bus_tolerance="${CAMPAIGN_BUS_TOLERANCE:-}"
  local bus_offset="${CAMPAIGN_BUS_OFFSET:-}"
  local bus_torque="${CAMPAIGN_BUS_TORQUE:-}"
  [[ -n "${fixture_revision}" ]] || fixture_revision="$(Prompt 'Fixture revision (letters, digits, dot, underscore, or dash):')"
  [[ -n "${bus_id}" ]] || bus_id="$(Prompt 'Measured bus-servo ID (1-253):')"
  [[ -n "${bus_hold}" ]] || bus_hold="$(Prompt 'Reviewed safe bus-servo hold position (0-1000):')"
  [[ -n "${bus_tolerance}" ]] || bus_tolerance="$(Prompt 'Measured bus-servo tolerance (0-100):')"
  [[ -n "${bus_offset}" ]] || bus_offset="$(Prompt 'Measured bus-servo offset (-125 to 125):')"
  [[ -n "${bus_torque}" ]] || bus_torque="$(Prompt 'Current bus-servo torque state (true or false):')"
  [[ "${fixture_revision}" =~ ^[A-Za-z0-9._-]+$ ]] || \
    Fail "fixture revision contains unsupported characters"
  ValidateInteger "${bus_id}" 1 253 "bus-servo ID"
  ValidateInteger "${bus_hold}" 0 1000 "bus-servo hold position"
  ValidateInteger "${bus_tolerance}" 0 100 "bus-servo tolerance"
  ValidateInteger "${bus_offset}" -125 125 "bus-servo offset"
  [[ "${bus_torque}" == "true" || "${bus_torque}" == "false" ]] || \
    Fail "bus-servo torque state must be true or false"
  local source_revision firmware_sha host_revision board_serial
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${PROJECT_ROOT}"
  source_revision="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
  host_revision="${source_revision}"
  firmware_sha="$(sha256sum "${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf" | awk '{print $1}')"
  board_serial="$(BoardSerial)"
  RunRuntime campaign "${mode}" "${source_revision}" "${firmware_sha}" \
    "${host_revision}" "${board_serial}" "${fixture_revision}" \
    "${bus_id}" "${bus_hold}" "${bus_tolerance}" "${bus_offset}" \
    "${bus_torque}"
}

[[ "$#" == 1 ]] || {
  echo "Usage: tutorial_action.sh ACTION" >&2
  exit 2
}
readonly ACTION="$1"
cd "${PROJECT_ROOT}"

case "${ACTION}" in
  serial-setup)
    RequireExact "${SERIAL_SETUP_ACK:-}" CONFIGURE_SERIAL_ACCESS \
      "This installs a udev rule and adds ${USER} to mentor-pi-serial."
    mapfile -t candidates < <(find /dev/serial/by-id -maxdepth 1 -type l \
      -name 'usb-1a86_USB_Single_Serial_*-if00' -print 2>/dev/null | sort)
    [[ "${#candidates[@]}" == 1 ]] || \
      Fail "expected exactly one Mentor Pi CH9102F under /dev/serial/by-id"
    sudo "${SCRIPT_DIR}/configure_dev_serial_access.sh" \
      --device "${candidates[0]}" --user "${USER}"
    ;;
  start)
    RequireExact "${RRCLITE_RUNTIME_ACK:-}" \
      PID_FIRMWARE_ACTUATORS_PREPARED \
      "Confirm safe actuator state before starting the normal default closed-loop PID firmware runtime."
    exec env RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}"
    ;;
  start-hardware | start-mecanum | start-ackermann)
    RequireExact "${RRCLITE_RUNTIME_ACK:-}" \
      PID_FIRMWARE_ACTUATORS_PREPARED \
      "Confirm safe actuator state before starting ros2_control with the default PID firmware."
    vehicle_config="${VEHICLE_CONFIG:-}"
    [[ -n "${vehicle_config}" ]] || \
      Fail "VEHICLE_CONFIG must select an absolute YAML deployment profile"
    exec env RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}" \
      --vehicle-config "${vehicle_config}"
    ;;
  passive-check)
    RequireExact "${PASSIVE_CHECK_ACK:-}" ACTUATORS_DISCONNECTED \
      "Confirm motor power and every servo mechanism are disconnected."
    oled_present="$(OledPresence)"
    RunRuntime passive-check "${oled_present}"
    ;;
  peripheral-smoke)
    RequireExact "${PERIPHERAL_SMOKE_ACK:-}" PASSIVE_OUTPUTS_GUARDED \
      "Confirm no servo mechanism is attached and output pins are guarded."
    RunRuntime peripheral-smoke "$(OledPresence)"
    ;;
  characterize-board)
    RequireExact "${CHARACTERIZATION_ACK:-}" \
      ACTUATORS_DISCONNECTED_WHEELS_RAISED \
      "Confirm motor power and servos are disconnected, encoders remain connected, and all four wheels are raised."
    RunRuntime characterize-board
    ;;
  release-software-gates)
    RequireExact "${RELEASE_GATES_ACK:-}" RUN_RELEASE_SOFTWARE_GATES \
      "These checks can take a long time and use substantial CPU and disk."
    [[ "$("${HOST_PROFILE_DETECTOR}" | sed -n 's/^profile=//p')" != rdk-x5 ]] || \
      Fail "full software gates must run on the normal computer, not the RDK X5"
    ./tools/run_quality_tests_container.sh --profile full
    ./tools/run_fuzz_smoke.sh
    ./tools/check_firmware_reproducibility.sh
    ./tools/run_firmware_target_ci.sh
    ./tools/verify_firmware_artifact.sh PID
    ;;
  release-onboard-gates)
    RequireExact "${RELEASE_GATES_ACK:-}" RUN_ONBOARD_DOCKER_GATES \
      "These bounded Docker arm64 checks can take a long time and use substantial CPU and disk."
    [[ "$("${HOST_PROFILE_DETECTOR}" | sed -n 's/^profile=//p')" == rdk-x5 ]] || \
      Fail "onboard Docker gates require the detected RDK X5"
    grep -Eq '^ID=ubuntu$' /etc/os-release
    grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release
    [[ "$(uname -m)" == "aarch64" || "$(uname -m)" == "arm64" ]] || \
      Fail "onboard Docker gates require the arm64 Ubuntu 22.04 computer"
    ./tools/build_host.sh
    ./tools/run_quality_tests_container.sh --profile rdk
    ./tools/check_firmware_reproducibility.sh
    ./tools/verify_firmware_artifact.sh PID
    echo "ONBOARD DOCKER GATES PASS: fuzz, coverage, and full Clang 18 analysis remain normal-computer gates."
    ;;
  qualification-preflight)
    RequireExact "${PREFLIGHT_ACK:-}" ACTUATORS_DISCONNECTED \
      "Keep the robot guarded with motor and servo power disconnected."
    RunRuntime qualification-preflight
    ;;
  campaign-load) Campaign load500 ;;
  campaign-soak) Campaign soak ;;
  campaign-recovery)
    mode="${RECOVERY_MODE:-}"
    [[ -n "${mode}" ]] || mode="$(Prompt 'Recovery mode: reconnect_usb, reconnect_agent, or reset_mcu:')"
    case "${mode}" in
      reconnect_usb | reconnect_agent | reset_mcu) ;;
      *) Fail "unsupported recovery mode" ;;
    esac
    Campaign "${mode}"
    ;;
  *) Fail "unsupported tutorial action: ${ACTION}" ;;
esac
