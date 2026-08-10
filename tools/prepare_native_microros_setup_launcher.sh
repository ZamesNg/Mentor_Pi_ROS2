#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Native micro-ROS launcher preparation error: $*" >&2
  exit 1
}

[[ "$#" == 3 ]] || \
  Fail "usage: prepare_native_microros_setup_launcher.sh PREFIX OUTPUT_DIR COLCON"

readonly MICROROS_SETUP_PREFIX="$1"
readonly OUTPUT_ROOT="$2"
readonly COLCON_EXECUTABLE="$3"
for path in "${MICROROS_SETUP_PREFIX}" "${OUTPUT_ROOT}" \
    "${COLCON_EXECUTABLE}"; do
  [[ "${path}" =~ ^/[A-Za-z0-9._/+:-]+$ ]] || \
    Fail "paths must be absolute and contain only safe characters: ${path}"
done
[[ -d "${MICROROS_SETUP_PREFIX}" && ! -L "${MICROROS_SETUP_PREFIX}" ]] || \
  Fail "micro_ros_setup prefix is missing or symbolic"
[[ -x "${COLCON_EXECUTABLE}" && ! -d "${COLCON_EXECUTABLE}" ]] || \
  Fail "colcon is not an executable file"

readonly SCRIPT_ROOT="${MICROROS_SETUP_PREFIX}/lib/micro_ros_setup"
readonly CREATE_FIRMWARE_SOURCE="${SCRIPT_ROOT}/create_firmware_ws.sh"
readonly CREATE_WORKSPACE_SOURCE="${SCRIPT_ROOT}/create_ws.sh"
readonly CLEAN_ENV_SOURCE="${SCRIPT_ROOT}/clean_env.sh"
[[ -x "${CREATE_FIRMWARE_SOURCE}" && -x "${CREATE_WORKSPACE_SOURCE}" &&
   -r "${CLEAN_ENV_SOURCE}" ]] || \
  Fail "pinned micro_ros_setup scripts are incomplete"

if [[ -e "${OUTPUT_ROOT}" || -L "${OUTPUT_ROOT}" ]]; then
  [[ -d "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
    Fail "output root exists but is not a regular directory"
else
  mkdir -p -- "${OUTPUT_ROOT}"
fi
readonly CREATE_FIRMWARE_OUTPUT="${OUTPUT_ROOT}/create_firmware_ws.sh"
readonly CLEAN_ENV_OUTPUT="${OUTPUT_ROOT}/clean_env.sh"
[[ ! -L "${CREATE_FIRMWARE_OUTPUT}" && ! -L "${CLEAN_ENV_OUTPUT}" ]] || \
  Fail "refusing symbolic launcher output"

install -m 0755 "${CREATE_FIRMWARE_SOURCE}" "${CREATE_FIRMWARE_OUTPUT}"
install -m 0755 "${CLEAN_ENV_SOURCE}" "${CLEAN_ENV_OUTPUT}"

# The upstream cleanup can remove /usr/bin when /usr is present in an RDK
# ament prefix. Restore only the standard executable path; library, Python,
# CMake, ament, and colcon prefix cleanup remains unchanged.
sed -i \
  '/export PATH=$(clean $PATH)/a\  export PATH="/usr/bin:/bin:${PATH}"' \
  "${CLEAN_ENV_OUTPUT}"

# The upstream launcher sources the private clean_env.sh beside itself. Use
# absolute, preflight-validated entry points for create_ws and colcon so neither
# depends on ROS executable paths that the cleanup intentionally removes.
sed -i \
  -e "s#ros2 run micro_ros_setup create_ws.sh#${CREATE_WORKSPACE_SOURCE}#g" \
  -e "s#        colcon build#        ${COLCON_EXECUTABLE} build#" \
  "${CREATE_FIRMWARE_OUTPUT}"

grep -Fq "${COLCON_EXECUTABLE} build" "${CREATE_FIRMWARE_OUTPUT}" || \
  Fail "pinned launcher does not contain the expected colcon call"
grep -Fq 'export PATH="/usr/bin:/bin:${PATH}"' "${CLEAN_ENV_OUTPUT}" || \
  Fail "pinned cleanup does not contain the native system PATH repair"

printf '%s\n' "${CREATE_FIRMWARE_OUTPUT}"
