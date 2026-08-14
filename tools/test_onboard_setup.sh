#!/usr/bin/env bash

set -euo pipefail
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

bash -n "${SCRIPT_DIR}/onboard_setup.sh"
python3 -m py_compile "${SCRIPT_DIR}/generate_vehicle_config.py" \
  "${SCRIPT_DIR}/configure_operator_zsh.py"
grep -Fq 'third_party/stm32cubeprogrammer_2.23.0_arm64.deb' \
  "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'PORT ?=' "${REPO_ROOT}/Makefile"
! grep -Fq 'PORT ?= /dev/mentor_pi_mcu' "${REPO_ROOT}/Makefile"
grep -Fq '$(if $(PORT),$(PORT),$(DEFAULT_PORT))' "${REPO_ROOT}/Makefile"
grep -Fq '46b844fd135627290d2d0af3d3897debfa4247ec6af7d1cea3e0e9b0fb0bd31d' \
  "${SCRIPT_DIR}/onboard_setup.sh"

package="${TEST_ROOT}/mentor_pi_hardwares"
mkdir -p "${package}/config/mecanum" "${package}/config/ackermann"
cp "${REPO_ROOT}/ros2_ws/src/mentor_pi_hardwares/config/mecanum/hardware.yaml" \
  "${package}/config/mecanum/hardware.yaml"
cp "${REPO_ROOT}/ros2_ws/src/mentor_pi_hardwares/config/ackermann/hardware.yaml" \
  "${package}/config/ackermann/hardware.yaml"
cp "${REPO_ROOT}/ros2_ws/src/mentor_pi_hardwares/config/mecanum/controllers.yaml" \
  "${package}/config/mecanum/controllers.yaml"
cp "${REPO_ROOT}/ros2_ws/src/mentor_pi_hardwares/config/ackermann/controllers.yaml" \
  "${package}/config/ackermann/controllers.yaml"
"${SCRIPT_DIR}/generate_vehicle_config.py" --type mecanum \
  --name fleet/robot_two --package-root "${package}" >/dev/null
grep -Fqx '  robot_name: fleet/robot_two' \
  "${package}/config/generated/vehicle.yaml"
grep -Fqx '  vehicle_type: mecanum' \
  "${package}/config/generated/vehicle.yaml"
grep -Fqx '    base_frame_id: fleet/robot_two/base_footprint' \
  "${package}/config/generated/controllers.yaml"
grep -Fqx '    odom_frame_id: fleet/robot_two/odom' \
  "${package}/config/generated/controllers.yaml"

zshrc="${TEST_ROOT}/operator/.zshrc"
mkdir -p "$(dirname "${zshrc}")"
cat >"${zshrc}" <<'EOF'
export MENTOR_PI_TYPE=ackermann
export MENTOR_PI_NAME=stale_robot
export ROS_DOMAIN_ID=0
# export ROS_DISCOVERY_SERVER=old:11811
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
source /opt/ros/humble/setup.zsh
source /srv/another_ws/ros2_ws/install/setup.zsh
EOF
workspace="${TEST_ROOT}/repo with spaces/ros2_ws/install/setup.zsh"
"${SCRIPT_DIR}/configure_operator_zsh.py" --zshrc "${zshrc}" \
  --workspace-setup "${workspace}" --vehicle-type mecanum \
  --vehicle-name robot_two
first="$(sha256sum "${zshrc}" | awk '{print $1}')"
"${SCRIPT_DIR}/configure_operator_zsh.py" --zshrc "${zshrc}" \
  --workspace-setup "${workspace}" --vehicle-type mecanum \
  --vehicle-name robot_two
[[ "${first}" == "$(sha256sum "${zshrc}" | awk '{print $1}')" ]]
grep -Fqx 'export MENTOR_PI_TYPE=mecanum' "${zshrc}"
grep -Fqx 'export MENTOR_PI_NAME=robot_two' "${zshrc}"
grep -Fqx 'source /srv/another_ws/ros2_ws/install/setup.zsh' "${zshrc}"
[[ "$(grep -c '^export ROS_DOMAIN_ID=42$' "${zshrc}")" == 1 ]]
[[ "$(grep -c '^export ROS_DISCOVERY_SERVER=192.168.2.191:11811$' "${zshrc}")" == 1 ]]
[[ "$(grep -Fxc 'eval "$(register-python-argcomplete3 ros2)"' "${zshrc}")" == 1 ]]
[[ "$(grep -Fxc 'eval "$(register-python-argcomplete3 colcon)"' "${zshrc}")" == 1 ]]
workspace_source_line="$(grep -nF "${workspace}" "${zshrc}" | cut -d: -f1)"
argcomplete_line="$(grep -nF 'eval "$(register-python-argcomplete3 ros2)"' "${zshrc}" | cut -d: -f1)"
[[ "${argcomplete_line}" -gt "${workspace_source_line}" ]]

grep -Fq '[[ "$(id -u)" != 0 ]]' "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'sudo -n true 2>/dev/null || sudo -v' \
  "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'InstallOnboardPrerequisites' "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'ResolveSerialPort' "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'micro_ros_agent/tools/find_device.sh' \
  "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'python3-colcon-common-extensions python3-rosdep python3-vcstool' \
  "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq 'sudo env "${environment[@]}" apt-get "$@"' \
  "${SCRIPT_DIR}/onboard_setup.sh"
grep -Fq "make --no-print-directory -C \"\${REPO_ROOT}/firmware\" flash" \
  "${SCRIPT_DIR}/onboard_setup.sh"
! grep -Eq '^[[:space:]]*MENTOR_PI_NAME=.*\\$' \
  "${SCRIPT_DIR}/onboard_setup.sh"
configure_block="$(sed -n '/if \[\[ "${MODE}" == configure \]\]/,/^fi$/p' \
  "${SCRIPT_DIR}/onboard_setup.sh")"
[[ "${configure_block}" != *'micro_ros_agent" setup'* ]]
[[ "${configure_block}" == *'BuildAndFlashFirmware'* ]]
