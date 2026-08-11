#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "ROS workspace environment error: $*" >&2
  exit 1
}

[[ "$(uname -s)" == Linux ]] || \
  Fail "use the repository VS Code Dev Container on macOS"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "native builds require Ubuntu 22.04; use the VS Code Dev Container elsewhere"
[[ -r /opt/ros/humble/setup.bash ]] || \
  Fail "ROS 2 Humble is missing at /opt/ros/humble"
for command in colcon rosdep vcs; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is unavailable"
done
case "$(uname -m)" in
  x86_64 | amd64 | aarch64 | arm64) ;;
  *) Fail "only amd64 and arm64 Linux hosts are supported" ;;
esac
echo "ROS workspace environment: Ubuntu 22.04 / Humble ($(uname -m))."
