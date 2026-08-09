#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly INSTALL_PREFIX="${MENTOR_PI_NATIVE_INSTALL_PREFIX:-${PROJECT_ROOT}/mentor_pi_ros2/install}"
readonly STATE_FILE="${INSTALL_PREFIX}/MENTOR-PI-NATIVE-BUILD.txt"

Fail() {
  echo "Onboard colcon state error: $*" >&2
  exit 1
}

[[ "$#" == 1 ]] || Fail "usage: ./tools/onboard_colcon_state.sh record|verify"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "the onboard host must be Ubuntu"
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  Fail "the onboard host must run Ubuntu 22.04"
case "$(uname -m)" in
  aarch64 | arm64) ;;
  *) Fail "the onboard host must be arm64" ;;
esac
[[ -r "${INSTALL_PREFIX}/setup.bash" ]] || \
  Fail "the conventional colcon install is missing: ${INSTALL_PREFIX}"
readonly CURRENT_SOURCE="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"

case "$1" in
  record)
    readonly TEMPORARY_STATE="${STATE_FILE}.tmp.$$"
    [[ ! -e "${TEMPORARY_STATE}" && ! -L "${TEMPORARY_STATE}" ]] || \
      Fail "unexpected temporary state exists: ${TEMPORARY_STATE}"
    printf '%s\n' \
      'schema=mentor-pi-native-colcon-v1' \
      'ubuntu=22.04' \
      'architecture=arm64' \
      'ros_distro=humble' \
      "source_sha256=${CURRENT_SOURCE}" \
      >"${TEMPORARY_STATE}"
    mv -- "${TEMPORARY_STATE}" "${STATE_FILE}"
    echo "Recorded verified onboard colcon source: ${CURRENT_SOURCE}"
    ;;
  verify)
    [[ -f "${STATE_FILE}" && ! -L "${STATE_FILE}" ]] || \
      Fail "record the successful colcon build and tests before runtime"
    grep -Fqx 'schema=mentor-pi-native-colcon-v1' "${STATE_FILE}" || \
      Fail "native colcon state has an unsupported schema"
    grep -Fqx 'ubuntu=22.04' "${STATE_FILE}" || \
      Fail "native colcon state targets another Ubuntu release"
    grep -Fqx 'architecture=arm64' "${STATE_FILE}" || \
      Fail "native colcon state targets another architecture"
    grep -Fqx 'ros_distro=humble' "${STATE_FILE}" || \
      Fail "native colcon state targets another ROS distribution"
    grep -Fqx "source_sha256=${CURRENT_SOURCE}" "${STATE_FILE}" || \
      Fail "tracked host inputs changed after colcon build; rebuild and retest"
    echo "Verified onboard colcon source: ${CURRENT_SOURCE}"
    ;;
  *) Fail "usage: ./tools/onboard_colcon_state.sh record|verify" ;;
esac
