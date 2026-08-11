#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "micro-ROS Agent environment error: $*" >&2
  exit 1
}

[[ "$(uname -s)" == Linux ]] || \
  Fail "use the repository VS Code Dev Container on macOS"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "native builds require Ubuntu 22.04; use the VS Code Dev Container elsewhere"
[[ -r /opt/ros/humble/setup.bash ]] || Fail "ROS 2 Humble is unavailable"
for command in cmake colcon git rosdep; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is unavailable"
done
case "$(dpkg --print-architecture)" in
  amd64 | arm64) ;;
  *) Fail "only native Ubuntu amd64 and arm64 builds are supported" ;;
esac
echo "Agent environment: Ubuntu 22.04 / Humble ($(dpkg --print-architecture))."
