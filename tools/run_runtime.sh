#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly HOST_BUILDER="${SCRIPT_DIR}/build_host.sh"
readonly AGENT_BUILDER="${SCRIPT_DIR}/build_agent.sh"
readonly FIRMWARE_VERIFIER="${SCRIPT_DIR}/verify_firmware_artifact.sh"
readonly TIME_SYNC_CHECK="${SCRIPT_DIR}/check_time_sync.sh"
readonly BUILD_IMAGE_PREPARER="${SCRIPT_DIR}/prepare_build_images.sh"
readonly CONTAINER_NAME="mentor-pi-runtime"
readonly REQUIRED_PID_ACK="PID_FIRMWARE_ACTUATORS_PREPARED"
readonly firmware_mode="PID"

serial_device=""
ros_domain_id=""
vehicle_config=""
tracking_controller="none"
resolved_vehicle_config=""
dry_run=0

Fail() {
  echo "Adaptive Mentor Pi runtime error: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: run_runtime.sh --device /dev/mentor_pi_mcu \
  --ros-domain-id 0..232 [--vehicle-config /absolute/robot.yaml] [--dry-run]
  [--tracking-controller none|mecanum|ackermann]

Every supported Ubuntu host runs the architecture-matched pinned ROS 2 Humble
container with only the reviewed MCU character device passed through.

Before starting a normal default PID firmware session, confirm safe actuator
state and operational clearances, then set:

  RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED
EOF
  exit 2
}

ReadOsValue() {
  local key="$1"
  local line=""
  local value=""
  line="$(grep -E "^${key}=" /etc/os-release || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] || \
    Fail "/etc/os-release must contain exactly one ${key}= entry"
  value="${line#*=}"
  value="${value#\"}"
  value="${value%\"}"
  printf '%s' "${value}"
}

PropertyValue() {
  local properties="$1"
  local key="$2"
  sed -n "s/^${key}=//p" <<<"${properties}" | head -n 1
}

while (($# > 0)); do
  case "$1" in
    --device)
      (($# >= 2)) || Usage
      serial_device="$2"
      shift 2
      ;;
    --ros-domain-id)
      (($# >= 2)) || Usage
      ros_domain_id="$2"
      shift 2
      ;;
    --vehicle-config)
      (($# >= 2)) || Usage
      vehicle_config="$2"
      shift 2
      ;;
    --tracking-controller)
      (($# >= 2)) || Usage
      tracking_controller="$2"
      shift 2
      ;;
    --dry-run) dry_run=1; shift ;;
    -h | --help) Usage ;;
    *) Usage ;;
  esac
done

if [[ -n "${vehicle_config}" ]]; then
  [[ "${vehicle_config}" == /* && "${vehicle_config}" != *:* && \
    "${vehicle_config}" != *$'\n'* ]] || \
    Fail "vehicle config must be an explicit absolute path without colon or newline"
  [[ -f "${vehicle_config}" && -r "${vehicle_config}" ]] || \
    Fail "vehicle config is not a readable regular file: ${vehicle_config}"
  resolved_vehicle_config="$(readlink -f -- "${vehicle_config}")"
  [[ -n "${resolved_vehicle_config}" && -f "${resolved_vehicle_config}" && \
    -r "${resolved_vehicle_config}" ]] || \
    Fail "vehicle config did not resolve to a readable regular file"
fi
readonly resolved_vehicle_config
[[ "${tracking_controller}" == none || "${tracking_controller}" == mecanum || \
   "${tracking_controller}" == ackermann ]] || \
  Fail "tracking controller must be none, mecanum, or ackermann"
[[ -n "${resolved_vehicle_config}" || "${tracking_controller}" == none ]] || \
  Fail "tracking controller requires a vehicle configuration"

[[ -r /etc/os-release && "$(ReadOsValue ID)" == "ubuntu" ]] || \
  Fail "the host must be Ubuntu"
readonly ubuntu_version="$(ReadOsValue VERSION_ID)"
case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
  *) Fail "the host architecture must be amd64 or arm64" ;;
esac
readonly architecture

[[ "${ros_domain_id}" =~ ^(0|[1-9][0-9]{0,2})$ ]] || \
  Fail "ROS domain ID must be an integer in [0, 232]"
((ros_domain_id <= 232)) || \
  Fail "ROS domain ID must be an integer in [0, 232]"
[[ "${serial_device}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && \
  "${serial_device}" != *"/../"* && "${serial_device}" != */.. && \
  "${serial_device}" != *"/./"* && "${serial_device}" != */. ]] || \
  Fail "device must be an explicit, well-formed /dev path"
[[ -c "${serial_device}" ]] || \
  Fail "device is not an existing character device: ${serial_device}"
readonly resolved_device="$(readlink -f "${serial_device}")"
[[ "${resolved_device}" == /dev/* && -c "${resolved_device}" ]] || \
  Fail "device did not resolve to a character device below /dev"

command -v udevadm >/dev/null 2>&1 || Fail "udevadm is unavailable"
device_properties="$(udevadm info --query=property \
  --name="${resolved_device}" 2>/dev/null)" || \
  Fail "udevadm could not inspect ${resolved_device}"
[[ "$(PropertyValue "${device_properties}" ID_VENDOR_ID)" == "1a86" && \
  "$(PropertyValue "${device_properties}" ID_MODEL_ID)" == "55d4" ]] || \
  Fail "device is not the Mentor Pi CH9102F (expected 1a86:55d4)"
if command -v fuser >/dev/null 2>&1 && \
    fuser "${resolved_device}" >/dev/null 2>&1; then
  fuser -v "${resolved_device}" >&2 || true
  Fail "another process already owns the MCU serial device"
fi

"${FIRMWARE_VERIFIER}" "${firmware_mode}" "${PROJECT_ROOT}" >/dev/null || \
  Fail "the current authoritative firmware artifact is not verified ${firmware_mode}"

"${HOST_BUILDER}" --runtime
"${AGENT_BUILDER}"
readonly host_prefix="$(${HOST_BUILDER} --runtime --print-output)"
readonly agent_prefix="$(${AGENT_BUILDER} --print-output)"
readonly agent_executable="${agent_prefix}/lib/micro_ros_agent/micro_ros_agent"
[[ -r "${host_prefix}/setup.bash" && -x "${agent_executable}" ]] || \
  Fail "adaptive runtime build is incomplete"

if ((dry_run == 1)); then
  printf '%s\n' \
    "mode=docker" \
    "ubuntu=${ubuntu_version}" \
    "architecture=${architecture}" \
    "device=${resolved_device}" \
    "ros_domain_id=${ros_domain_id}" \
    "vehicle_config=${resolved_vehicle_config:-none}" \
    "tracking_controller=${tracking_controller}" \
    "host_prefix=${host_prefix}" \
    "agent_prefix=${agent_prefix}" \
    'result=validated; runtime not started'
  exit 0
fi

readonly required_ack="${REQUIRED_PID_ACK}"
[[ "${RRCLITE_RUNTIME_ACK:-}" == "${required_ack}" ]] || {
  Usage
  Fail "set RRCLITE_RUNTIME_ACK=${required_ack} only after completing the required fixture checks"
}

if [[ "${tracking_controller}" != none ]]; then
  "${TIME_SYNC_CHECK}"
fi

printf '%s\n' \
  "Starting Mentor Pi runtime on Ubuntu ${ubuntu_version}." \
  "MCU: ${resolved_device}" \
  "Firmware mode: ${firmware_mode}" \
  "ROS_DOMAIN_ID=${ros_domain_id}" \
  'Keep this terminal open and press Ctrl-C once to stop.' \
  "Use make shell ROS_DOMAIN_ID=${ros_domain_id} from a second terminal."

command -v docker >/dev/null 2>&1 || Fail "Docker is unavailable"
[[ -d /run/udev ]] || Fail "host udev database is unavailable at /run/udev"
if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  Fail "container ${CONTAINER_NAME} already exists; stop that exact container first"
fi
"${BUILD_IMAGE_PREPARER}" --architecture "${architecture}"
readonly image="$("${BUILD_IMAGE_PREPARER}" --architecture "${architecture}" --print project)"
readonly device_gid="$(stat -Lc '%g' "${resolved_device}")"
docker_vehicle_arguments=()
container_vehicle_config=""
if [[ -n "${resolved_vehicle_config}" ]]; then
  docker_vehicle_arguments=(
    --volume "${resolved_vehicle_config}:/opt/mentor_pi/vehicle.yaml:ro"
  )
  container_vehicle_config="/opt/mentor_pi/vehicle.yaml"
fi

exec docker run --rm \
  --name "${CONTAINER_NAME}" \
  --init \
  --stop-timeout 5 \
  --platform "linux/${architecture}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  --group-add "${device_gid}" \
  --device "${resolved_device}:/dev/mentor_pi_mcu:rwm" \
  --volume /run/udev:/run/udev:ro \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,mode=1777,size=256m \
  --env HOME=/tmp/mentor-pi-home \
  --env ROS_LOG_DIR=/tmp/mentor-pi-ros-log \
  --env "ROS_DOMAIN_ID=${ros_domain_id}" \
  --env "RRCLITE_RUNTIME_ACK=${required_ack}" \
  --env MENTOR_PI_DEVELOPMENT_RUNTIME=1 \
  --env MENTOR_PI_PROJECT_ROOT=/workspace \
  --env MENTOR_PI_FIRMWARE_VERIFIER=/workspace/tools/verify_firmware_artifact.sh \
  --env MENTOR_PI_RRCLITE_AUTORESET=1 \
  --env MENTOR_PI_HOST_PREFIX=/opt/mentor_pi/host \
  --env MENTOR_PI_AGENT_PREFIX=/opt/mentor_pi/micro_ros_agent \
  --env MENTOR_PI_AGENT_EXECUTABLE=/opt/mentor_pi/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent \
  "${docker_vehicle_arguments[@]}" \
  --volume "${PROJECT_ROOT}:/workspace:ro" \
  --volume "${host_prefix}:/opt/mentor_pi/host:ro" \
  --volume "${agent_prefix}:/opt/mentor_pi/micro_ros_agent:ro" \
  --workdir /workspace \
  --entrypoint /bin/bash \
  "${image}" -lc \
  'set -eo pipefail
   mkdir -p "${HOME}" "${ROS_LOG_DIR}"
   set +u
   source /opt/ros/humble/setup.bash
   source /opt/mentor_pi/micro_ros_agent/local_setup.bash
   source /opt/mentor_pi/host/setup.bash
   set -u
   if [[ -z "${1}" ]]; then
     exec ros2 launch mentor_pi_bringup controller.launch.py \
       serial_device:=/dev/mentor_pi_mcu \
       agent_executable:="${MENTOR_PI_AGENT_EXECUTABLE}"
   fi
   exec ros2 launch mentor_pi_hardwares vehicle.launch.py \
     vehicle_config:="${1}" \
     tracking_controller:="${2}" \
     serial_device:=/dev/mentor_pi_mcu \
     agent_executable:="${MENTOR_PI_AGENT_EXECUTABLE}"' \
  mentor-pi-runtime "${container_vehicle_config}" "${tracking_controller}"
