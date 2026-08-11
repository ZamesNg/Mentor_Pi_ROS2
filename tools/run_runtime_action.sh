#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly RUNTIME_CONTEXT="${MENTOR_PI_RUNTIME_ACTION_CONTEXT:-development}"
if [[ "${RUNTIME_CONTEXT}" == production ]]; then
  readonly DEFAULT_EVIDENCE_ROOT="/var/log/mentor-pi/actions"
else
  readonly DEFAULT_EVIDENCE_ROOT="${PROJECT_ROOT}/build"
fi
readonly EVIDENCE_ROOT="${MENTOR_PI_ACTION_EVIDENCE_ROOT:-${DEFAULT_EVIDENCE_ROOT}}"
if [[ "${RUNTIME_CONTEXT}" == production ]]; then
  readonly DEFAULT_CAPTURE_TOOL="/opt/mentor_pi/tools/capture_board_diagnostics"
else
  readonly DEFAULT_CAPTURE_TOOL="${PROJECT_ROOT}/ros2_ws/src/mentor_pi_bringup/scripts/capture_board_diagnostics"
fi
readonly CAPTURE_TOOL="${MENTOR_PI_CAPTURE_TOOL:-${DEFAULT_CAPTURE_TOOL}}"
readonly PACKAGED_FIRMWARE_SHA256="${MENTOR_PI_PACKAGED_FIRMWARE_SHA256:-}"

Fail() {
  echo "Mentor Pi runtime action failed: $*" >&2
  exit 1
}

case "${RUNTIME_CONTEXT}" in
  development | production) ;;
  *) Fail "runtime action context must be development or production" ;;
esac
if [[ "${RUNTIME_CONTEXT}" == development ]]; then
  export MENTOR_PI_DEVELOPMENT_RUNTIME=1
else
  export MENTOR_PI_DEVELOPMENT_RUNTIME=0
fi
[[ "${EVIDENCE_ROOT}" == /* && "${EVIDENCE_ROOT}" != / && \
   "${EVIDENCE_ROOT}" != *$'\n'* ]] || \
  Fail "runtime action evidence root must be an absolute non-root path"
[[ "${CAPTURE_TOOL}" == /* && "${CAPTURE_TOOL}" != *$'\n'* ]] || \
  Fail "runtime action capture tool must be an absolute path"
[[ -x "${CAPTURE_TOOL}" ]] || Fail "runtime action capture tool is not installed"
if [[ "${RUNTIME_CONTEXT}" == production ]]; then
  [[ "${PACKAGED_FIRMWARE_SHA256}" =~ ^[0-9a-f]{64}$ ]] || \
    Fail "production runtime action lacks a verified packaged firmware identity"
fi

CleanupPeripheralOutputs() {
  set +e
  local led_id
  timeout 3 ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
    /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
    '{frequency_hz: 0, on_time_ms: 0, off_time_ms: 0, repeat: 0}' \
    >/dev/null 2>&1
  for led_id in 1 2; do
    timeout 3 ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
      /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
      "{led_id: ${led_id}, on_time_ms: 0, off_time_ms: 0, repeat: 0}" \
      >/dev/null 2>&1
  done
  timeout 3 ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
    /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
    '{update_mask: 1, red: [0, 0], green: [0, 0], blue: [0, 0]}' \
    >/dev/null 2>&1
}

RequireZeroMotorTargets() {
  local file="$1"
  awk '
    /^target_rps:$/ { in_targets=1; found=1; next }
    in_targets && /^- / {
      value=$0
      sub(/^- /, "", value)
      count++
      if ((value + 0) != 0) bad=1
      if (count == 4) in_targets=0
    }
    END { exit !(found && count == 4 && !bad) }
  ' "${file}" || Fail "zero-command preflight reported a nonzero motor target"
}

InsideRuntime() {
  local action="$1"
  shift
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

  case "${action}" in
    controller-present)
      ros2 node list --no-daemon --spin-time 1.0 2>/dev/null \
        | grep -Fqx '/mentor_pi/controller'
      ;;
    controller-wait)
      local timeout_seconds="${1:-5}"
      local phase="${2:-controller}"
      local acceptance_seconds="${3:-0}"
      [[ "${timeout_seconds}" =~ ^[1-9][0-9]*$ &&
        "${acceptance_seconds}" =~ ^[0-9]+$ ]] ||
        Fail "controller-wait received an invalid timeout"
      local started_at=${SECONDS}
      local deadline=$((started_at + timeout_seconds))
      local next_report=${started_at}
      local -a topics=(
        /mentor_pi/battery/state
        /mentor_pi/bus_servos/command
        /mentor_pi/buttons/events
        /mentor_pi/buzzer/command
        /mentor_pi/diagnostics
        /mentor_pi/heartbeat
        /mentor_pi/imu
        /mentor_pi/leds/command
        /mentor_pi/motors/command
        /mentor_pi/motors/state
        /mentor_pi/oled/command
        /mentor_pi/pwm_servos/command
        /mentor_pi/pwm_servos/state
        /mentor_pi/rgb/command
      )
      local -a services=(
        /mentor_pi/battery/set_low_threshold
        /mentor_pi/bus_servos/configure
        /mentor_pi/bus_servos/get_state
        /mentor_pi/bus_servos/stop
        /mentor_pi/motors/set_model
        /mentor_pi/motors/set_pid
        /mentor_pi/pwm_servos/set_offsets
      )
      echo "CONTROLLER WAIT [${phase}]: monitoring graph and heartbeat for up to ${timeout_seconds} seconds."
      local nodes="" topic_list="" service_list="" expected=""
      local node_count=0 topic_count=0 service_count=0 missing=0
      local wait_directory=""
      wait_directory="$(mktemp -d /tmp/mentor-pi-controller-wait.XXXXXX)"
      timeout "${timeout_seconds}" ros2 topic echo --once \
        --qos-reliability reliable /mentor_pi/heartbeat \
        mentor_pi_interfaces/msg/Heartbeat \
        >"${wait_directory}/heartbeat" 2>/dev/null &
      local heartbeat_pid=$!
      while ((SECONDS < deadline)); do
        node_count=0
        topic_count=0
        service_count=0
        missing=0
        ros2 node list --no-daemon --spin-time 0.2 \
          >"${wait_directory}/nodes" 2>/dev/null &
        local node_pid=$!
        ros2 topic list --no-daemon --spin-time 0.2 \
          >"${wait_directory}/topics" 2>/dev/null &
        local topic_pid=$!
        ros2 service list --no-daemon --spin-time 0.2 \
          >"${wait_directory}/services" 2>/dev/null &
        local service_pid=$!
        wait "${node_pid}" || true
        wait "${topic_pid}" || true
        wait "${service_pid}" || true
        nodes="$(<"${wait_directory}/nodes")"
        topic_list="$(<"${wait_directory}/topics")"
        service_list="$(<"${wait_directory}/services")"
        if grep -Fqx '/mentor_pi/controller' <<<"${nodes}"; then
          ((node_count += 1))
        else
          missing=1
        fi
        if grep -Fqx '/mentor_pi/configuration_supervisor' <<<"${nodes}"; then
          ((node_count += 1))
        else
          missing=1
        fi
        for expected in "${topics[@]}"; do
          if grep -Fqx "${expected}" <<<"${topic_list}"; then
            ((topic_count += 1))
          else
            missing=1
          fi
        done
        for expected in "${services[@]}"; do
          if grep -Fqx "${expected}" <<<"${service_list}"; then
            ((service_count += 1))
          else
            missing=1
          fi
        done
        if ((missing == 0)) &&
            grep -Fq 'sequence:' "${wait_directory}/heartbeat"; then
          local elapsed=$((SECONDS - started_at))
          echo "CONTROLLER READY [${phase}]: all 21 MCU endpoints and heartbeat are present after ${elapsed} seconds."
          if ((acceptance_seconds > 0 && elapsed > acceptance_seconds)); then
            kill "${heartbeat_pid}" 2>/dev/null || true
            wait "${heartbeat_pid}" 2>/dev/null || true
            rm -rf -- "${wait_directory}"
            Fail "${phase} recovered in ${elapsed} seconds, exceeding the ${acceptance_seconds}-second limit"
          fi
          kill "${heartbeat_pid}" 2>/dev/null || true
          wait "${heartbeat_pid}" 2>/dev/null || true
          rm -rf -- "${wait_directory}"
          return 0
        fi
        if ((SECONDS >= next_report)); then
          echo "CONTROLLER WAIT [${phase}]: $((SECONDS - started_at))/${timeout_seconds}s nodes=${node_count}/2 topics=${topic_count}/${#topics[@]} services=${service_count}/${#services[@]} heartbeat=pending"
          next_report=$((SECONDS + 2))
        fi
        sleep 0.2
      done
      echo "CONTROLLER TIMEOUT [${phase}]: discovered nodes:" >&2
      if [[ -n "${nodes}" ]]; then
        printf '%s\n' "${nodes}" >&2
      else
        echo "(none)" >&2
      fi
      kill "${heartbeat_pid}" 2>/dev/null || true
      wait "${heartbeat_pid}" 2>/dev/null || true
      rm -rf -- "${wait_directory}"
      Fail "${phase} graph/heartbeat did not become complete within ${timeout_seconds} seconds (nodes=${node_count}/2 topics=${topic_count}/${#topics[@]} services=${service_count}/${#services[@]})"
      ;;
    passive-check)
      [[ "$#" == 1 && ("$1" == "0" || "$1" == "1") ]] ||
        Fail "passive-check requires OLED presence as 0 or 1"
      local oled_present="$1"
      local output="${EVIDENCE_ROOT}/diagnostics/passive-$(date -u +%Y%m%dT%H%M%SZ)"
      mkdir -p "$(dirname "${output}")"
      echo "PASSIVE CHECK [1/3]: verifying firmware and controller graph."
      if [[ "${RUNTIME_CONTEXT}" == development ]]; then
        "${PROJECT_ROOT}/firmware/tools/verify.sh"
      else
        echo "Verified packaged PID firmware SHA-256: ${PACKAGED_FIRMWARE_SHA256}"
      fi
      ros2 node list --no-daemon --spin-time 2.0 \
        | tee "${output}.nodes.txt"
      grep -Fqx '/mentor_pi/controller' "${output}.nodes.txt" || \
        Fail "/mentor_pi/controller is absent"
      grep -Fqx '/mentor_pi/configuration_supervisor' "${output}.nodes.txt" || \
        Fail "/mentor_pi/configuration_supervisor is absent"
      local -a capture_arguments=(
        --output "${output}"
        --qualification imu-characterization
        --qualification-duration-sec 60
      )
      if [[ "${RUNTIME_CONTEXT}" == development ]]; then
        capture_arguments+=(--repository-root "${PROJECT_ROOT}")
      fi
      if [[ "${oled_present}" == "0" ]]; then
        capture_arguments+=(--allow-missing-oled)
        echo "PASSIVE CHECK LIMITATION: OLED is not installed and will not be credited as tested."
      fi
      echo "PASSIVE CHECK [2/3]: collecting diagnostics and monitoring telemetry for 60 seconds."
      "${CAPTURE_TOOL}" "${capture_arguments[@]}"
      echo "PASSIVE CHECK [3/3]: proving the PID image remains at zero with actuator power disconnected."
      ros2 topic pub --once --qos-reliability best_effort \
        --qos-durability volatile --qos-depth 1 \
        /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
        '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'
      timeout 10 ros2 topic echo --once --qos-reliability best_effort \
        --qos-durability volatile /mentor_pi/motors/state \
        mentor_pi_interfaces/msg/MotorState \
        | tee "${output}/pid-zero-motor-state.yaml"
      RequireZeroMotorTargets "${output}/pid-zero-motor-state.yaml"
      timeout 10 ros2 topic echo --once --qos-reliability reliable \
        /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics \
        | tee "${output}/pid-zero-motor-diagnostics.yaml"
      if [[ "${RUNTIME_CONTEXT}" == production ]]; then
        sed -i 's/^development_runtime=1$/development_runtime=0/' \
          "${output}/SUMMARY.txt"
        printf 'runtime_context=production\npackaged_firmware_sha256=%s\n' \
          "${PACKAGED_FIRMWARE_SHA256}" \
          >"${output}/production-runtime.txt"
        (
          cd "${output}"
          find . -type f ! -name SHA256SUMS -print0 | sort -z | \
            xargs -0 sha256sum
        ) >"${output}/SHA256SUMS"
        tar -C "$(dirname "${output}")" -czf "${output}.tar.gz" -- \
          "$(basename "${output}")"
        (
          cd "$(dirname "${output}")"
          sha256sum -- "$(basename "${output}").tar.gz"
        ) >"${output}.tar.gz.sha256"
      fi
      if [[ "${oled_present}" == "0" ]]; then
        echo "PASSIVE CHECK PASS WITH LIMITATION: OLED NOT INSTALLED/NOT TESTED; ${output}"
      else
        echo "PASSIVE CHECK PASS: ${output}"
      fi
      ;;
    peripheral-smoke)
      [[ "$#" == 1 && ("$1" == "0" || "$1" == "1") ]] ||
        Fail "peripheral-smoke requires OLED presence as 0 or 1"
      local oled_present="$1"
      local led_id
      trap CleanupPeripheralOutputs EXIT
      trap 'exit 130' INT
      trap 'exit 143' TERM
      for led_id in 1 2; do
        timeout 10 ros2 topic pub --once --qos-reliability reliable \
          --qos-depth 1 \
          /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
          "{led_id: ${led_id}, on_time_ms: 200, off_time_ms: 200, repeat: 2}"
      done
      timeout 10 ros2 topic pub --once --qos-reliability reliable \
        --qos-depth 1 \
        /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
        '{frequency_hz: 1000, on_time_ms: 100, off_time_ms: 100, repeat: 2}'
      timeout 10 ros2 topic pub --once --qos-reliability reliable \
        --qos-depth 1 \
        /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
        '{update_mask: 1, red: [0, 0], green: [0, 0], blue: [32, 0]}'
      if [[ "${oled_present}" == "1" ]]; then
        timeout 10 ros2 topic pub --once --qos-reliability reliable \
          --qos-depth 1 \
          /mentor_pi/oled/command mentor_pi_interfaces/msg/OledCommand \
          "{update_mask: 3, line_1: 'Mentor Pi', line_2: 'PASSIVE'}"
      else
        echo "PERIPHERAL SMOKE LIMITATION: OLED is not installed; display command skipped."
      fi
      timeout 10 ros2 topic pub --times 5 --rate 10 \
        --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
        /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
        '{update_mask: 15, duration_ms: 20, pulse_width_us: [1500, 1500, 1500, 1500]}'
      for led_id in 1 2; do
        echo "Press and release board button ${led_id} within 10 seconds."
        local button_output
        button_output="$(timeout 10 ros2 topic echo --once \
          --qos-reliability reliable /mentor_pi/buttons/events \
          mentor_pi_interfaces/msg/ButtonEvent)" || \
          Fail "no event arrived for button ${led_id}"
        grep -Fq "button_id: ${led_id}" <<<"${button_output}" || \
          Fail "the observed event was not from button ${led_id}"
        printf '%s\n' "${button_output}"
      done
      timeout 10 ros2 service call /mentor_pi/bus_servos/get_state \
        mentor_pi_interfaces/srv/GetBusServoState \
        '{servo_id: 1, fields: 511}' || true
      if [[ "${oled_present}" == "0" ]]; then
        echo "PERIPHERAL SMOKE COMPLETE WITH LIMITATION: OLED NOT INSTALLED/NOT TESTED; verify all other observed outputs."
      else
        echo "PERIPHERAL SMOKE COMPLETE: verify the observed outputs and stop on any mismatch."
      fi
      ;;
    characterize-board)
      [[ "$#" == 0 ]] || Fail "characterize-board does not accept arguments"
      exec "${SCRIPT_DIR}/characterize_board.sh"
      ;;
    commission-motor)
      [[ "$#" == 3 ]] || Fail "commission-motor requires motor, target, duration"
      local motor_id="$1"
      local target_rps="$2"
      local duration_ms="$3"
      mkdir -p "${EVIDENCE_ROOT}/motor-commissioning"
      ros2 run mentor_pi_bringup motor_commissioning --ros-args \
        -p acknowledgement:=MOTORS_RAISED_CURRENT_LIMITED \
        -p motor_id:="${motor_id}" \
        -p target_rps:="${target_rps}" \
        -p duration_ms:="${duration_ms}" \
        | tee "${EVIDENCE_ROOT}/motor-commissioning/$(date -u +%Y%m%dT%H%M%SZ)-motor-${motor_id}.log"
      ;;
    qualification-preflight)
      ros2 run mentor_pi_bringup qualification_monitor --ros-args \
        -p duration_sec:=60.0
      ros2 run mentor_pi_bringup qualification_monitor --ros-args \
        -p duration_sec:=60.0 \
        -p publish_zero_motor_commands:=true \
        -p zero_command_rate_hz:=500.0
      ;;
    campaign)
      [[ "$#" == 11 ]] || Fail "campaign received an invalid argument count"
      local mode="$1" source_revision="$2" firmware_sha="$3"
      local host_revision="$4" board_serial="$5" fixture_revision="$6"
      local bus_id="$7" bus_hold="$8" bus_tolerance="$9"
      shift 9
      local bus_offset="$1" bus_torque="$2"
      local evidence_parent="${EVIDENCE_ROOT}/qualification"
      local run_id="${mode}-$(date -u +%Y%m%dT%H%M%SZ)"
      local output="${evidence_parent}/${run_id}"
      mkdir -p "${evidence_parent}"
      [[ ! -e "${output}" ]] || Fail "campaign output already exists"
      ros2 run mentor_pi_bringup qualification_campaign --ros-args \
        -p mode:="${mode}" \
        -p duration_sec:=-1.0 \
        -p evidence_directory:="${output}" \
        -p run_id:="${run_id}" \
        -p source_revision:="${source_revision}" \
        -p firmware_sha256:="${firmware_sha}" \
        -p host_revision:="${host_revision}" \
        -p board_serial:="${board_serial}" \
        -p fixture_revision:="${fixture_revision}" \
        -p fixture_acknowledgement:=PERIPHERALS_DISCONNECTED_OR_GUARDED \
        -p bus_servo_id:="${bus_id}" \
        -p bus_hold_position:="${bus_hold}" \
        -p bus_position_tolerance:="${bus_tolerance}" \
        -p bus_current_offset:="${bus_offset}" \
        -p bus_torque_enabled:="${bus_torque}" \
        -p require_button_stimulus:=true \
        -p require_valid_imu:=true
      ;;
    *) Fail "unsupported runtime action: ${action}" ;;
  esac
}

if [[ "${1:-}" == "--inside" ]]; then
  shift
  [[ "$#" -ge 1 ]] || Fail "missing runtime action"
  InsideRuntime "$@"
  exit
fi

[[ "$#" -ge 1 ]] || Fail "missing runtime action"
readonly ACTION="$1"
shift

[[ ! -f /.dockerenv ]] || \
  Fail "runtime actions require native Ubuntu 22.04, not the Dev Container"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "runtime actions require native Ubuntu 22.04"
[[ "$(uname -s)" == Linux && -r /opt/ros/humble/setup.bash ]] || \
  Fail "runtime actions require native Ubuntu 22.04 with ROS 2 Humble"
if [[ "${RUNTIME_CONTEXT}" == production ]]; then
  "${PROJECT_ROOT}/firmware/tools/verify.sh" >/dev/null
  verified_firmware_sha="$(sha256sum \
    "${PROJECT_ROOT}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf" \
    | awk '{print $1}')"
  [[ "${PACKAGED_FIRMWARE_SHA256}" == "${verified_firmware_sha}" ]] || \
    Fail "production firmware identity does not match the verified PID artifact"
fi
[[ -r "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]] || \
  Fail "ROS applications are not built; run make -C ros2_ws build"
command -v systemctl >/dev/null 2>&1 || \
  Fail "systemctl is required for onboard runtime actions"
systemctl is-active --quiet mentor-pi-agent.service || \
  Fail "mentor-pi-agent.service is not active"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ros2 daemon stop >/dev/null 2>&1 || true
InsideRuntime "${ACTION}" "$@"
