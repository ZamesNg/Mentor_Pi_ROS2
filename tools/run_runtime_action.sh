#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly CONTAINER_NAME="mentor-pi-runtime"

Fail() {
  echo "Mentor Pi runtime action failed: $*" >&2
  exit 1
}

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
      local output="${PROJECT_ROOT}/build/diagnostics/passive-$(date -u +%Y%m%dT%H%M%SZ)"
      echo "PASSIVE CHECK [1/3]: verifying firmware and controller graph."
      "${SCRIPT_DIR}/verify_firmware_artifact.sh" PID "${PROJECT_ROOT}"
      ros2 node list --no-daemon --spin-time 2.0 \
        | tee "${output}.nodes.txt"
      grep -Fqx '/mentor_pi/controller' "${output}.nodes.txt" || \
        Fail "/mentor_pi/controller is absent"
      grep -Fqx '/mentor_pi/configuration_supervisor' "${output}.nodes.txt" || \
        Fail "/mentor_pi/configuration_supervisor is absent"
      local -a capture_arguments=(
        --output "${output}"
        --repository-root "${PROJECT_ROOT}"
        --qualification imu-characterization
        --qualification-duration-sec 60
      )
      if [[ "${oled_present}" == "0" ]]; then
        capture_arguments+=(--allow-missing-oled)
        echo "PASSIVE CHECK LIMITATION: OLED is not installed and will not be credited as tested."
      fi
      echo "PASSIVE CHECK [2/3]: collecting diagnostics and monitoring telemetry for 60 seconds."
      "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/scripts/capture_board_diagnostics" \
        "${capture_arguments[@]}"
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
      mkdir -p "${PROJECT_ROOT}/build/motor-commissioning"
      ros2 run mentor_pi_bringup motor_commissioning --ros-args \
        -p acknowledgement:=MOTORS_RAISED_CURRENT_LIMITED \
        -p motor_id:="${motor_id}" \
        -p target_rps:="${target_rps}" \
        -p duration_ms:="${duration_ms}" \
        | tee "${PROJECT_ROOT}/build/motor-commissioning/$(date -u +%Y%m%dT%H%M%SZ)-motor-${motor_id}.log"
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
      local evidence_parent="${PROJECT_ROOT}/build/qualification"
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

if [[ "$(docker container inspect "${CONTAINER_NAME}" \
    --format '{{.State.Running}}' 2>/dev/null || true)" == "true" ]]; then
  readonly EXPECTED_HOST_PREFIX="$(${SCRIPT_DIR}/build_host.sh --runtime --print-output)"
  readonly MOUNTED_HOST_PREFIX="$(docker container inspect "${CONTAINER_NAME}" \
    --format '{{range .Mounts}}{{if eq .Destination "/opt/mentor_pi/host"}}{{.Source}}{{end}}{{end}}')"
  [[ -n "${MOUNTED_HOST_PREFIX}" && "${MOUNTED_HOST_PREFIX}" == /* &&
    "${MOUNTED_HOST_PREFIX}" != *$'\n'* ]] ||
    Fail "running runtime does not expose one valid /opt/mentor_pi/host mount"
  [[ "${MOUNTED_HOST_PREFIX}" == "${EXPECTED_HOST_PREFIX}" ]] ||
    Fail "running runtime is stale (${MOUNTED_HOST_PREFIX}); stop its make start with Ctrl-C, then run make start again to load ${EXPECTED_HOST_PREFIX}"
  exec docker exec --interactive \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "${CONTAINER_NAME}" /bin/bash -lc \
    'set -euo pipefail
     set +u
     source /opt/ros/humble/setup.bash
     source /opt/mentor_pi/micro_ros_agent/local_setup.bash
     source /opt/mentor_pi/host/setup.bash
     set -u
     ros2 daemon stop >/dev/null 2>&1 || true
     export MENTOR_PI_DEVELOPMENT_RUNTIME=1
     export MENTOR_PI_HOST_PREFIX=/opt/mentor_pi/host
     export MENTOR_PI_AGENT_PREFIX=/opt/mentor_pi/micro_ros_agent
     export MENTOR_PI_AGENT_EXECUTABLE=/opt/mentor_pi/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent
     exec /workspace/tools/run_runtime_action.sh --inside "$@"' \
    mentor-pi-runtime-action "${ACTION}" "$@"
fi

grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "the host must be Ubuntu"
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  Fail "start make start first; non-22.04 hosts require its Docker runtime"
if [[ -n "${MENTOR_PI_NATIVE_INSTALL_PREFIX:-}" ]]; then
  HOST_PREFIX="${MENTOR_PI_NATIVE_INSTALL_PREFIX}"
elif [[ -r "${PROJECT_ROOT}/mentor_pi_ros2/install/setup.bash" ]]; then
  HOST_PREFIX="${PROJECT_ROOT}/mentor_pi_ros2/install"
else
  HOST_PREFIX="$(${SCRIPT_DIR}/build_host.sh --runtime --print-output)"
fi
readonly HOST_PREFIX
readonly AGENT_PREFIX="$(${SCRIPT_DIR}/build_agent.sh --print-output)"
[[ -r "${HOST_PREFIX}/setup.bash" ]] || \
  Fail "native ROS install is missing: ${HOST_PREFIX}; run colcon build first"
if [[ "${HOST_PREFIX}" == "${PROJECT_ROOT}/mentor_pi_ros2/install" || \
      -n "${MENTOR_PI_NATIVE_INSTALL_PREFIX:-}" ]]; then
  MENTOR_PI_NATIVE_INSTALL_PREFIX="${HOST_PREFIX}" \
    "${SCRIPT_DIR}/onboard_colcon_state.sh" verify >/dev/null
fi
set +u
source /opt/ros/humble/setup.bash
source "${AGENT_PREFIX}/local_setup.bash"
source "${HOST_PREFIX}/setup.bash"
set -u
export MENTOR_PI_DEVELOPMENT_RUNTIME=1
export MENTOR_PI_HOST_PREFIX="${HOST_PREFIX}"
export MENTOR_PI_NATIVE_INSTALL_PREFIX="${HOST_PREFIX}"
export MENTOR_PI_AGENT_PREFIX="${AGENT_PREFIX}"
export MENTOR_PI_AGENT_EXECUTABLE="${AGENT_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
InsideRuntime "${ACTION}" "$@"
