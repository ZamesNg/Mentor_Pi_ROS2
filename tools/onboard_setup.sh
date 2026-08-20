#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
readonly HARDWARE_PACKAGE="${REPO_ROOT}/ros2_ws/src/mentor_pi_hardwares"
readonly CUBE_PROGRAMMER_PACKAGE="${REPO_ROOT}/third_party/stm32cubeprogrammer_2.23.0_arm64.deb"
readonly CUBE_PROGRAMMER_SHA256=46b844fd135627290d2d0af3d3897debfa4247ec6af7d1cea3e0e9b0fb0bd31d
readonly FLASH_TOKEN=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
readonly MODE="${1:-}"
readonly MENTOR_PI_TYPE="${MENTOR_PI_TYPE:-}"
readonly MENTOR_PI_NAME="${MENTOR_PI_NAME:-}"
PORT="${PORT:-}"
agent_was_active=0

Fail() {
  echo "Mentor Pi onboarding error: $*" >&2
  exit 1
}

RestoreAgentOnFailure() {
  local status=$?
  if ((status != 0 && agent_was_active == 1)); then
    sudo systemctl restart mentor-pi-agent.service >/dev/null 2>&1 || true
  fi
  exit "${status}"
}
trap RestoreAgentOnFailure EXIT

[[ "${MODE}" == setup || "${MODE}" == configure ]] || \
  Fail "usage: onboard_setup.sh setup|configure"
[[ "$(id -u)" != 0 ]] || Fail "run as the normal operator, not root"
[[ "${MENTOR_PI_TYPE}" == mecanum || "${MENTOR_PI_TYPE}" == ackermann ]] || \
  Fail "export MENTOR_PI_TYPE=mecanum or ackermann"
[[ "${MENTOR_PI_NAME}" =~ ^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$ ]] || \
  Fail "export MENTOR_PI_NAME as a valid relative ROS namespace"
[[ -r /etc/os-release ]] || Fail "cannot identify the operating system"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "native Ubuntu 22.04 is required"
[[ ! -f /.dockerenv ]] || Fail "onboarding cannot run in a container"
[[ -r /opt/ros/humble/setup.zsh && -r /opt/ros/humble/setup.bash ]] || \
  Fail "ROS 2 Humble shell setup is missing"
command -v sudo >/dev/null || Fail "sudo is required"
command -v getent >/dev/null || Fail "getent is required"
readonly OPERATOR="$(id -un)"
readonly OPERATOR_HOME="$(getent passwd "${OPERATOR}" | awk -F: 'NR == 1 {print $6}')"
[[ -n "${OPERATOR_HOME}" && -d "${OPERATOR_HOME}" ]] || \
  Fail "could not resolve the operator home directory"

GenerateVehicleConfig() {
  "${SCRIPT_DIR}/generate_vehicle_config.py" \
    --type "${MENTOR_PI_TYPE}" --name "${MENTOR_PI_NAME}" \
    --package-root "${HARDWARE_PACKAGE}" >/dev/null
}

ResolveSerialPort() {
  if [[ -z "${PORT}" ]]; then
    if [[ -c /dev/mentor_pi_mcu ]]; then
      PORT=/dev/mentor_pi_mcu
    else
      PORT="$("${REPO_ROOT}/micro_ros_agent/tools/find_device.sh" --path)"
    fi
  fi
  [[ "${PORT}" =~ ^/dev/[A-Za-z0-9._/+:-]+$ && -c "${PORT}" ]] || \
    Fail "PORT must identify an existing explicit serial device"
  readonly PORT
}

SudoApt() {
  local -a environment=(DEBIAN_FRONTEND=noninteractive)
  local variable
  for variable in http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY; do
    [[ -z "${!variable:-}" ]] || environment+=("${variable}=${!variable}")
  done
  sudo env "${environment[@]}" apt-get "$@"
}

InstallOnboardPrerequisites() {
  local command
  for command in cmake colcon git ninja rosdep tar vcs xz; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      SudoApt update
      SudoApt install -y --no-install-recommends \
        build-essential ca-certificates cmake curl git ninja-build \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
        xz-utils zsh
      return
    fi
  done
}

InstallCubeProgrammer() {
  if [[ "$(dpkg-query -W -f='${Version}' stm32cubeprogrammer 2>/dev/null || true)" == \
        2.23.0 && -x "$(command -v STM32_Programmer_CLI 2>/dev/null || true)" ]]; then
    return
  fi
  [[ "$(dpkg --print-architecture)" == arm64 ]] || \
    Fail "the packaged STM32CubeProgrammer requires arm64"
  [[ -f "${CUBE_PROGRAMMER_PACKAGE}" && \
     ! -L "${CUBE_PROGRAMMER_PACKAGE}" ]] || \
    Fail "packaged STM32CubeProgrammer is missing"
  [[ "$(sha256sum "${CUBE_PROGRAMMER_PACKAGE}" | awk '{print $1}')" == \
     "${CUBE_PROGRAMMER_SHA256}" ]] || \
    Fail "packaged STM32CubeProgrammer checksum mismatch"

  SudoApt install -y "${CUBE_PROGRAMMER_PACKAGE}"
  [[ "$(dpkg-query -W -f='${Version}' stm32cubeprogrammer 2>/dev/null || true)" == \
        2.23.0 && -x "$(command -v STM32_Programmer_CLI 2>/dev/null || true)" ]] || \
    Fail "STM32CubeProgrammer installation did not validate"
}

RequestFlashAcknowledgement() {
  if [[ "${FLASH_ACK:-}" == "${FLASH_TOKEN}" ]]; then
    printf '%s' "${FLASH_TOKEN}"
    return
  fi
  [[ -t 0 ]] || Fail "export FLASH_ACK=${FLASH_TOKEN} for non-interactive use"
  printf '%s\n' \
    'Firmware flashing requires the STM32 ROM bootloader and disconnected motors.' \
    "Type exactly: ${FLASH_TOKEN}"
  local response
  IFS= read -r response
  [[ "${response}" == "${FLASH_TOKEN}" ]] || Fail "flash acknowledgement rejected"
  printf '%s' "${response}"
}

StopAgentForFlash() {
  if systemctl is-active --quiet mentor-pi-agent.service; then
    agent_was_active=1
    sudo systemctl stop mentor-pi-agent.service
  fi
}

BuildAndFlashFirmware() {
  local acknowledgement
  acknowledgement="$(RequestFlashAcknowledgement)"
  make --no-print-directory -C "${REPO_ROOT}/firmware" build verify package
  StopAgentForFlash
  make --no-print-directory -C "${REPO_ROOT}/firmware" flash \
    PORT="${PORT}" FLASH_ACK="${acknowledgement}"
}

BuildHardwarePackage() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  "${REPO_ROOT}/ros2_ws/tools/colcon.sh" build --symlink-install \
    --packages-select mentor_pi_hardwares
}

sudo -n true 2>/dev/null || sudo -v
InstallOnboardPrerequisites
InstallCubeProgrammer
ResolveSerialPort
GenerateVehicleConfig

if [[ "${MODE}" == configure ]]; then
  if pgrep -f 'ros2_control_node|vehicle.launch.py' \
      >/dev/null 2>&1; then
    Fail "stop vehicle bringup before changing its type or namespace"
  fi
  BuildAndFlashFirmware
  BuildHardwarePackage
  sudo systemctl restart mentor-pi-agent.service
  agent_was_active=0
  trap - EXIT
  echo "Onboard configuration updated. Relaunch vehicle.launch.py."
  exit 0
fi

make --no-print-directory -C "${REPO_ROOT}/firmware" setup test
BuildAndFlashFirmware
make --no-print-directory -C "${REPO_ROOT}/micro_ros_agent" setup build test
sudo make --no-print-directory -C "${REPO_ROOT}/micro_ros_agent" install-service \
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 \
  ROS_DISCOVERY_SERVER=192.168.2.191:11811 DEVICE="${PORT}"
agent_was_active=0
make --no-print-directory -C "${REPO_ROOT}/ros2_ws" deps build
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
"${REPO_ROOT}/ros2_ws/tools/colcon.sh" test \
  --packages-select mentor_pi_hardwares
"${REPO_ROOT}/ros2_ws/tools/colcon.sh" test-result --verbose
"${SCRIPT_DIR}/configure_operator_zsh.py" \
  --zshrc "${OPERATOR_HOME}/.zshrc" \
  --workspace-setup "${REPO_ROOT}/ros2_ws/install/setup.zsh" \
  --vehicle-type "${MENTOR_PI_TYPE}" \
  --vehicle-name "${MENTOR_PI_NAME}"
trap - EXIT
echo "Mentor Pi onboarding completed for ${MENTOR_PI_NAME} (${MENTOR_PI_TYPE})."
