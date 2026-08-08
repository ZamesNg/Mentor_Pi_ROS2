#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

Fail() {
  echo "Board characterization failed: $*" >&2
  exit 1
}

PromptContinue() {
  printf '%s\nPress Enter when ready.\n' "$1"
  IFS= read -r _ || Fail "input ended before characterization completed"
}

CaptureTopic() {
  local topic="$1" type="$2" reliability="$3" output="$4"
  timeout 10 ros2 topic echo --once \
    --qos-reliability "${reliability}" --qos-durability volatile \
    "${topic}" "${type}" >"${output}" 2>/dev/null
}

ExtractArray() {
  local field="$1" count="$2" input="$3"
  awk -v target="${field}:" -v expected="${count}" '
    $0 == target { inside = 1; next }
    inside && /^[[:space:]]*-[[:space:]]*/ {
      value = $0
      sub(/^[[:space:]]*-[[:space:]]*/, "", value)
      print value
      found++
      if (found == expected) exit
    }
    END { if (found != expected) exit 1 }
  ' "${input}"
}

ExtractScalar() {
  local field="$1" input="$2"
  sed -n "s/^${field}: //p" "${input}" | head -n 1
}

EncoderEvidenceIsReusable() {
  local input="$1"
  awk -F '\t' '
    function abs(value) { return value < 0 ? -value : value }
    NR == 1 { next }
    NF >= 8 {
      rows++
      changed = 0
      channel = 0
      for (field = 3; field <= 6; field++) {
        if (abs($field) > 50) {
          changed++
          channel = field - 2
        }
      }
      if (changed != 1 || abs($(channel + 2)) <= 100 || seen[channel]++) {
        failed = 1
      }
    }
    END { exit !(rows == 4 && !failed) }
  ' "${input}"
}

ReviewEncoderEvidence() {
  local input="$1" output="$2"
  local sample position unused old_result
  local delta1 delta2 delta3 delta4
  local -A seen_channels=()
  local rows=0 failed=0

  printf 'physical_position\tros_channel\tforward_delta\tforward_target_sign\tresult\n' \
    >"${output}"
  printf '\n%-13s %-11s %14s %-19s %s\n' \
    "Position" "ROS channel" "Forward delta" "Forward target sign" "Result"
  printf '%-13s %-11s %14s %-19s %s\n' \
    "-------------" "-----------" "--------------" "-------------------" "------"

  while IFS=$'\t' read -r sample position delta1 delta2 delta3 delta4 \
      unused old_result; do
    [[ "${sample}" != "motor" && "${sample}" != "sample" ]] || continue
    [[ -n "${position}" ]] || continue
    local -a deltas=("${delta1}" "${delta2}" "${delta3}" "${delta4}")
    local changed=0 observed_index=-1
    for index in 0 1 2 3; do
      local absolute=${deltas[index]#-}
      if ((absolute > 50)); then
        changed=$((changed + 1))
        observed_index=${index}
      fi
    done

    local result="PASS" channel="-" forward_delta=0 target_sign="-"
    if ((changed != 1 || observed_index < 0)); then
      result="FAIL_CHANNEL"
      failed=1
    else
      channel="M$((observed_index + 1))"
      forward_delta=${deltas[observed_index]}
      if [[ -n "${seen_channels[${channel}]:-}" ]]; then
        result="FAIL_DUPLICATE_CHANNEL"
        failed=1
      fi
      seen_channels[${channel}]=1
      if ((forward_delta > 0)); then
        target_sign="positive"
      else
        target_sign="negative"
      fi
    fi
    rows=$((rows + 1))
    printf '%-13s %-11s %14d %-19s %s\n' \
      "${position}" "${channel}" "${forward_delta}" "${target_sign}" "${result}"
    printf '%s\t%s\t%d\t%s\t%s\n' \
      "${position}" "${channel}" "${forward_delta}" "${target_sign}" \
      "${result}" >>"${output}"
  done <"${input}"

  ((rows == 4 && ${#seen_channels[@]} == 4 && failed == 0))
}

reuse_encoder_source=""
reuse_encoders=0
latest_encoder_results="$(find "${PROJECT_ROOT}/build/diagnostics" -mindepth 2 \
  -maxdepth 2 -type f -name encoders.tsv -print 2>/dev/null | sort | tail -n 1)"
if [[ -n "${latest_encoder_results}" ]] && \
    EncoderEvidenceIsReusable "${latest_encoder_results}"; then
  printf '%s\n' \
    "Found a complete one-wheel-at-a-time encoder capture:" \
    "  ${latest_encoder_results}" \
    "It proves a unique ROS channel for every physical wheel." \
    "Reuse it and continue directly to the IMU test? [Y/n]"
  IFS= read -r reuse_answer || Fail "input ended before reuse was selected"
  case "${reuse_answer}" in
    "" | y | Y | yes | YES)
      reuse_encoder_source="${latest_encoder_results}"
      reuse_encoders=1
      ;;
    n | N | no | NO) ;;
    *) Fail "reuse answer must be y or n" ;;
  esac
fi
output="${PROJECT_ROOT}/build/diagnostics/characterization-$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -e "${output}" ]]; then
  output="${output}-$$"
fi
readonly OUTPUT="${output}"
mkdir -p "${OUTPUT}"
readonly MOTOR_RESULTS="${OUTPUT}/encoders.tsv"
readonly MOTOR_MAPPING="${OUTPUT}/encoder-mapping.tsv"
readonly IMU_RESULTS="${OUTPUT}/imu-six-face.tsv"

cat >"${OUTPUT}/README.txt" <<'EOF'
Motor-locked, probe-free board characterization.
Physical wheel positions are mapped to ROS M1-M4 by observation; no legacy
chassis position diagram is assumed.
Battery conversion uses the reference-compatible firmware defaults and is not
externally calibrated by this procedure.
EOF
if ((reuse_encoders == 1)); then
  cp "${reuse_encoder_source}" "${MOTOR_RESULTS}"
  {
    printf 'Encoder evidence copied from: %s\n' "${reuse_encoder_source}"
    printf 'Encoder evidence SHA-256: '
    sha256sum "${MOTOR_RESULTS}" | awk '{print $1}'
  } >>"${OUTPUT}/README.txt"
fi

echo "CHARACTERIZATION [1/3]: checking the controller and motor-state stream."
ros2 node list --no-daemon --spin-time 1.0 2>/dev/null \
  | grep -Fqx '/mentor_pi/controller' || Fail "/mentor_pi/controller is absent; keep make start running"
stream_probe="${OUTPUT}/motor-initial.yaml"
if ((reuse_encoders == 1)); then
  stream_probe="${OUTPUT}/motor-resume.yaml"
fi
CaptureTopic /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
  best_effort "${stream_probe}" || \
  Fail "no motor-state sample arrived within ten seconds"

readonly -a MOTOR_POSITIONS=("front-right" "rear-right" "front-left" "rear-left")
if ((reuse_encoders == 1)); then
  echo "Reusing the prior encoder capture; no wheel rotation is required."
else
  printf 'sample\tposition\tdelta_m1\tdelta_m2\tdelta_m3\tdelta_m4\tselected_delta\tresult\n' \
    >"${MOTOR_RESULTS}"
  for position_index in 0 1 2 3; do
    sample_id=$((position_index + 1))
    position="${MOTOR_POSITIONS[position_index]}"
    before_file="${OUTPUT}/position-${position}-before.yaml"
    after_file="${OUTPUT}/position-${position}-after.yaml"
    CaptureTopic /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
      best_effort "${before_file}" || Fail "no baseline arrived for ${position}"
    mapfile -t before < <(ExtractArray encoder_count 4 "${before_file}")
    [[ "${#before[@]}" == 4 ]] || Fail "invalid encoder baseline for ${position}"

    PromptContinue "Rotate only the physical ${position} wheel forward by about one revolution. Do not select it by a board connector number. Forward means the direction that would drive the robot straight forward. Keep motor power disconnected."

    CaptureTopic /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
      best_effort "${after_file}" || Fail "no final sample arrived for ${position}"
    mapfile -t after < <(ExtractArray encoder_count 4 "${after_file}")
    [[ "${#after[@]}" == 4 ]] || Fail "invalid encoder result for ${position}"

    deltas=(0 0 0 0)
    changed=0
    observed="none"
    observed_delta=0
    for index in 0 1 2 3; do
      deltas[index]=$((after[index] - before[index]))
      absolute=${deltas[index]#-}
      if ((absolute > 50)); then
        changed=$((changed + 1))
        observed="M$((index + 1))"
        observed_delta=${deltas[index]}
      fi
    done
    echo "Captured ${position}: changed=${observed} delta=${observed_delta}"
    printf 'P%d\t%s\t%d\t%d\t%d\t%d\t0\tCAPTURED\n' \
      "${sample_id}" "${position}" "${deltas[0]}" "${deltas[1]}" \
      "${deltas[2]}" "${deltas[3]}" >>"${MOTOR_RESULTS}"
  done
fi

if ! ReviewEncoderEvidence "${MOTOR_RESULTS}" "${MOTOR_MAPPING}"; then
  echo
  echo "STOP: encoder ownership is missing, ambiguous, or duplicated."
  echo "Do not run powered commissioning."
  echo "Evidence: ${OUTPUT}"
  exit 1
fi
echo "Encoder ownership PASS. Positive and negative forward signs are both valid; chassis software owns those wheel-specific inversions."

echo
echo "CHARACTERIZATION [2/3]: recording the six stationary IMU faces."
printf 'orientation\tx_m_s2\ty_m_s2\tz_m_s2\tvalid\tresult\n' >"${IMU_RESULTS}"
printf '\n%-8s %10s %10s %10s %-7s %s\n' \
  "Face" "Accel X" "Accel Y" "Accel Z" "Valid" "Result"
printf '%-8s %10s %10s %10s %-7s %s\n' \
  "--------" "----------" "----------" "----------" "-------" "------"

readonly -a IMU_LABELS=("+X" "-X" "+Y" "-Y" "+Z" "-Z")
readonly -a IMU_INSTRUCTIONS=(
  "Hold the board vertically with the USB-C connector edge upward and keep it still."
  "Hold the board vertically with the edge opposite the USB-C connectors upward and keep it still."
  "Hold the board vertically with the PWM-servo connector edge upward and keep it still."
  "Hold the board vertically with the edge opposite the PWM-servo connectors upward and keep it still."
  "Lay the board flat with the component side facing upward and keep it still."
  "Lay the board flat with the component side facing downward and keep it still."
)
readonly -a IMU_AXES=(0 0 1 1 2 2)
readonly -a IMU_SIGNS=(1 -1 1 -1 1 -1)

imu_limited=0
for orientation_index in 0 1 2 3 4 5; do
  label="${IMU_LABELS[orientation_index]}"
  PromptContinue "${IMU_INSTRUCTIONS[orientation_index]} Expected board-axis gravity: ${label}."
  imu_slug="${label//+/_plus_}"
  imu_slug="${imu_slug//-/_minus_}"
  imu_file="${OUTPUT}/imu-${imu_slug}.yaml"
  if ! CaptureTopic /mentor_pi/imu mentor_pi_interfaces/msg/ImuState \
      best_effort "${imu_file}"; then
    echo "IMU       -          -          -       -       BLOCKED_NO_SAMPLES"
    printf '%s\tNA\tNA\tNA\tfalse\tBLOCKED_NO_SAMPLES\n' "${label}" \
      >>"${IMU_RESULTS}"
    imu_limited=1
    break
  fi
  mapfile -t acceleration < <(ExtractArray linear_acceleration_m_s2 3 "${imu_file}")
  valid="$(ExtractScalar valid "${imu_file}")"
  [[ "${valid}" == "true" ]] || valid="false"
  result="$(awk \
    -v x="${acceleration[0]}" -v y="${acceleration[1]}" \
    -v z="${acceleration[2]}" -v axis="${IMU_AXES[orientation_index]}" \
    -v sign="${IMU_SIGNS[orientation_index]}" -v valid="${valid}" '
      function abs(value) { return value < 0 ? -value : value }
      BEGIN {
        values[0] = x; values[1] = y; values[2] = z
        first_other = (axis + 1) % 3
        second_other = (axis + 2) % 3
        magnitude = sqrt(x*x + y*y + z*z)
        pass = valid == "true" && sign * values[axis] >= 7.0 &&
               abs(values[first_other]) <= 4.0 &&
               abs(values[second_other]) <= 4.0 &&
               magnitude >= 8.0 && magnitude <= 12.0
        print pass ? "PASS" : "REVIEW_AXIS"
      }
    ')"
  [[ "${result}" == "PASS" ]] || imu_limited=1
  printf '%-8s %10.3f %10.3f %10.3f %-7s %s\n' \
    "${label}" "${acceleration[0]}" "${acceleration[1]}" \
    "${acceleration[2]}" "${valid}" "${result}"
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${label}" "${acceleration[0]}" "${acceleration[1]}" \
    "${acceleration[2]}" "${valid}" "${result}" >>"${IMU_RESULTS}"
done

echo
echo "CHARACTERIZATION [3/3]: battery uses the RRCLite reference defaults."
printf '%s\n' \
  "  ADC input       PB0 / ADC1 channel 8" \
  "  Divider         11:1" \
  "  Filter weight   0.05" \
  "  Low threshold   6300 mV" \
  "  Absent battery  <=4900 mV" \
  "  Calibration     NOT REQUIRED FOR USABLE BRING-UP"

if ((imu_limited != 0)); then
  echo
  echo "CHARACTERIZATION PASS WITH IMU LIMITATION"
  echo "Encoder direction is safe to use for guarded commissioning."
  echo "IMU release qualification remains blocked until its rows pass."
else
  echo
  echo "CHARACTERIZATION PASS"
fi
echo "Evidence: ${OUTPUT}"
