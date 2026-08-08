#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PORT="${PORT:-/dev/mentor_pi_mcu}"
readonly ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

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
  "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED "${PROJECT_ROOT}"
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
      LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED \
      "Disconnect motor power, PWM servos, and bus servos before starting."
    exec env RRCLITE_RUNTIME_ACK=LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}" --firmware-mode LOCKED
    ;;
  start-commissioning)
    RequireExact "${RRCLITE_RUNTIME_ACK:-}" MOTORS_RAISED_CURRENT_LIMITED \
      "Confirm every wheel is raised, the current limit is enabled, and the physical stop is reachable."
    exec env RRCLITE_RUNTIME_ACK=MOTORS_RAISED_CURRENT_LIMITED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}" --firmware-mode COMMISSIONING
    ;;
  start-commissioning-pid)
    RequireExact "${RRCLITE_RUNTIME_ACK:-}" MOTORS_RAISED_CURRENT_LIMITED \
      "Confirm every wheel is raised, the current limit is enabled, and the physical stop is reachable."
    exec env RRCLITE_RUNTIME_ACK=MOTORS_RAISED_CURRENT_LIMITED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}" --firmware-mode COMMISSIONING_PID
    ;;
  start-hardware | start-mecanum | start-ackermann)
    RequireExact "${RRCLITE_RUNTIME_ACK:-}" \
      LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED \
      "Disconnect motor power, PWM servos, and bus servos before starting ros2_control."
    vehicle_config="${VEHICLE_CONFIG:-}"
    [[ -n "${vehicle_config}" ]] || \
      Fail "VEHICLE_CONFIG must select an absolute YAML deployment profile"
    exec env RRCLITE_RUNTIME_ACK=LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED \
      "${SCRIPT_DIR}/run_runtime.sh" --device "${PORT}" \
      --ros-domain-id "${ROS_DOMAIN_ID}" --firmware-mode LOCKED \
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
  recovery-check)
    RequireExact "${RECOVERY_CHECK_ACK:-}" ACTUATORS_DISCONNECTED \
      "Confirm motor power and every servo mechanism are disconnected."
    [[ -t 0 ]] || Fail "recovery rehearsal requires an interactive terminal"
    RunRuntime controller-wait 30 baseline 0
    printf '%s\n' \
      "In the make start terminal, press Ctrl-C and run make start again." \
      "Press Enter here immediately after make start begins; this helper will" \
      "print discovery progress and enforce the five-second recovery limit." >&2
    IFS= read -r _
    RunRuntime controller-wait 15 agent-restart 5
    printf '%s\n' \
      "Unplug only the CH9102F data cable for two seconds, reconnect it," \
      "restart make start if needed, then press Enter." >&2
    IFS= read -r _
    WaitForDevice
    RunRuntime controller-wait 15 usb-reconnect 5
    echo "RECOVERY CHECK PASS"
    ;;
  characterize-board)
    RequireExact "${CHARACTERIZATION_ACK:-}" \
      ACTUATORS_DISCONNECTED_WHEELS_RAISED \
      "Confirm motor power and servos are disconnected, encoders remain connected, and all four wheels are raised."
    "${SCRIPT_DIR}/verify_firmware_artifact.sh" LOCKED
    RunRuntime characterize-board
    ;;
  build-commissioning)
    RequireExact "${COMMISSIONING_BUILD_ACK:-}" MOTORS_RAISED \
      "Raise every wheel and enable a conservative current limit."
    exec make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED
    ;;
  build-commissioning-pid)
    RequireExact "${COMMISSIONING_BUILD_ACK:-}" MOTORS_RAISED \
      "Raise every wheel and enable a conservative current limit."
    exec make firmware-commissioning-pid COMMISSIONING_BUILD_ACK=MOTORS_RAISED
    ;;
  commission-motor)
    RequireExact "${COMMISSIONING_RUN_ACK:-}" \
      MOTORS_RAISED_CURRENT_LIMITED \
      "Confirm one selected motor is guarded and the physical stop is reachable."
    motor_id="${MOTOR_ID:-}"
    target_rps="${TARGET_RPS:-}"
    duration_ms="${DURATION_MS:-}"
    [[ -n "${motor_id}" ]] || motor_id="$(Prompt 'Motor ID (1-4):')"
    [[ -n "${target_rps}" ]] || target_rps="$(Prompt 'Target RPS (-0.25 to -0.01 or 0.01 to 0.25):')"
    [[ -n "${duration_ms}" ]] || duration_ms="$(Prompt 'Duration in milliseconds (100-5000; use 1000 for an initial direction test):')"
    motor_id="$(TrimAsciiWhitespace "${motor_id}")"
    target_rps="$(TrimAsciiWhitespace "${target_rps}")"
    duration_ms="$(TrimAsciiWhitespace "${duration_ms}")"
    ValidateInteger "${motor_id}" 1 4 "motor ID"
    ValidateInteger "${duration_ms}" 100 5000 "duration"
    if [[ ! "${target_rps}" =~ ^-?0[.][0-9]{1,6}$ ]]; then
      printf -v rendered_target '%q' "${target_rps}"
      Fail "target RPS is malformed: ${rendered_target}"
    fi
    awk -v value="${target_rps}" 'BEGIN {m=value<0?-value:value; exit !(m>=0.01 && m<=0.25)}' || \
      Fail "target magnitude must be in [0.01, 0.25] RPS"
    echo "DIRECTION CHECK: fixed 250-permille drive; PID is bypassed; 0.50 RPS (30 RPM) overspeed cutoff."
    RunRuntime commission-motor "${motor_id}" "${target_rps}" "${duration_ms}"
    physical_direction="${PHYSICAL_DIRECTION_CONFIRMED:-}"
    if [[ -z "${physical_direction}" ]]; then
      physical_direction="$(Prompt 'Did the selected wheel physically rotate in the requested direction (+ forward, - reverse)? Enter y or n:')"
    fi
    case "${physical_direction}" in
      y | Y | yes | YES | 1)
        echo "PHYSICAL DIRECTION PASS: motor=${motor_id}"
        ;;
      n | N | no | NO | 0)
        Fail "physical wheel direction was not confirmed; do not test another motor"
        ;;
      *) Fail "physical direction answer must be y or n" ;;
    esac
    ;;
  hil-start)
    output="${PROJECT_ROOT}/build/hil/hil-$(date -u +%Y%m%dT%H%M%SZ)"
    mkdir -p "${output}"
    sha256sum firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf \
      | tee "${output}/artifact-sha256.txt"
    printf '%s\n' "board_serial=$(BoardSerial)" \
      "source_revision=$(git rev-parse HEAD)" \
      | tee "${output}/identity.txt"
    printf '%s\n' "${output}" >build/hil/current-path
    echo "HIL RUN READY: ${output}"
    ;;
  hil-peripheral-check)
    echo "BLOCKED: an instrumented peripheral fixture command is not configured." >&2
    exit 1
    ;;
  hil-recovery-check)
    echo "BLOCKED: independent outage/reset timing instrumentation is not configured." >&2
    exit 1
    ;;
  release-software-gates)
    RequireExact "${RELEASE_GATES_ACK:-}" RUN_RELEASE_SOFTWARE_GATES \
      "These checks can take a long time and use substantial CPU and disk."
    python3 tools/check_framework_docs.py
    ./tools/run_native_ci_tests.sh --build-type Debug --sanitizers on
    ./tools/run_native_ci_tests.sh --build-type Release --sanitizers off
    ./tools/run_tsan_tests.sh
    ./tools/run_coverage_tests.sh
    CLANG_FORMAT=clang-format-18 ./tools/check_cpp_format.sh
    CLANG_TIDY=clang-tidy-18 RUN_CLANG_TIDY=run-clang-tidy-18 \
      ./tools/run_clang_tidy.sh
    ./tools/run_fuzz_smoke.sh
    ./tools/check_firmware_reproducibility.sh
    ./tools/run_firmware_target_ci.sh
    ./tools/verify_firmware_artifact.sh LOCKED
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
