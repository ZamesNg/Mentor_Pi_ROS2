#!/usr/bin/env bash

set -euo pipefail

os_release="/etc/os-release"
architecture=""
ros_setup="/opt/ros/humble/setup.bash"
check_tools="yes"

Usage() {
  cat >&2 <<'EOF'
Usage: verify_host_build_environment.sh [--os-release PATH]
  [--architecture amd64|arm64] [--ros-setup PATH]
  [--check-tools yes|no]
EOF
  exit 2
}

Fail() {
  echo "Host build environment error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --os-release) os_release="${2:-}"; shift 2 ;;
    --architecture) architecture="${2:-}"; shift 2 ;;
    --ros-setup) ros_setup="${2:-}"; shift 2 ;;
    --check-tools) check_tools="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${check_tools}" == "yes" || "${check_tools}" == "no" ]] || Usage
ResolveReadableFile() {
  local requested="$1"
  local description="$2"
  local resolved=""
  resolved="$(readlink -f "${requested}" 2>/dev/null || true)"
  [[ -n "${resolved}" && -f "${resolved}" && -r "${resolved}" ]] ||
    Fail "${description} is missing, dangling, non-regular, or unreadable: ${requested}"
  printf '%s' "${resolved}"
}

# Ubuntu commonly provides /etc/os-release as a relative symlink to the regular
# /usr/lib/os-release file. Resolve it once and read only the canonical target;
# dangling links and directory/device targets remain rejected.
os_release="$(ResolveReadableFile "${os_release}" "OS identity")"
ros_setup="$(ResolveReadableFile "${ros_setup}" "ROS 2 Humble setup")"

ReadOsReleaseValue() {
  local key="$1"
  local line=""
  line="$(grep -E "^${key}=" "${os_release}" || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] ||
    Fail "${os_release} must contain exactly one ${key}= entry"
  local value="${line#*=}"
  if [[ "${value}" == \"*\" && "${value}" == *\" ]]; then
    value="${value#\"}"
    value="${value%\"}"
  fi
  printf '%s' "${value}"
}

[[ "$(ReadOsReleaseValue ID)" == "ubuntu" ]] ||
  Fail "the host must be Ubuntu"
[[ "$(ReadOsReleaseValue VERSION_ID)" == "22.04" ]] ||
  Fail "the host must be Ubuntu 22.04"
if [[ -z "${architecture}" ]]; then
  command -v dpkg >/dev/null 2>&1 || Fail "dpkg is required"
  architecture="$(dpkg --print-architecture)"
fi
case "${architecture}" in
  amd64 | arm64) ;;
  *) Fail "only Ubuntu 22.04 amd64 and arm64 are supported" ;;
esac

check_script='set -eo pipefail
set +u
source "$1"
set -u
[[ "${ROS_DISTRO:-}" == humble ]] || {
  echo "ROS setup did not identify ROS_DISTRO=humble" >&2
  exit 1
}
if [[ "$2" == yes ]]; then
  for tool in colcon rosdep cmake c++ python3 sha256sum tar gzip; do
    command -v "${tool}" >/dev/null 2>&1 || {
      echo "required host build tool is missing: ${tool}" >&2
      exit 1
    }
  done
fi'
if ! bash -c "${check_script}" mentor-pi-host-check \
    "${ros_setup}" "${check_tools}"; then
  Fail "ROS identity or required host build tools are incomplete"
fi

echo "Verified Ubuntu 22.04 ${architecture}, ROS 2 Humble, and host tools=${check_tools}."
