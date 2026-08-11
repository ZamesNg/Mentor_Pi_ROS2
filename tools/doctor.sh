#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() { echo "Mentor Pi doctor error: $*" >&2; exit 1; }
for command in git make cmake; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is unavailable"
done
[[ "$(uname -s)" == Linux ]] || \
  Fail "open this repository in its VS Code Dev Container on macOS"
[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  Fail "use the VS Code Dev Container outside Ubuntu 22.04"
case "$(uname -m)" in
  x86_64 | amd64 | aarch64 | arm64) ;;
  *) Fail "only amd64 and arm64 are supported" ;;
esac
[[ -d "${PROJECT_ROOT}/firmware" && -d "${PROJECT_ROOT}/micro_ros_agent" && \
   -d "${PROJECT_ROOT}/ros2_ws/src" ]] || Fail "component layout is incomplete"
if [[ -f /.dockerenv ]]; then
  environment="VS Code Dev Container"
else
  environment="native Ubuntu 22.04"
fi
printf '%s\n' \
  "Environment: ${environment}" \
  "Architecture: $(uname -m)" \
  'Firmware build: make -C firmware ...' \
  'Agent build: make -C micro_ros_agent ...' \
  'ROS build: make -C ros2_ws ...'
