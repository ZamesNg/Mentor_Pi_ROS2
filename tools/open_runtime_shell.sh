#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly HOST_BUILDER="${SCRIPT_DIR}/build_host.sh"
readonly AGENT_BUILDER="${SCRIPT_DIR}/build_agent.sh"
readonly CONTAINER_NAME="mentor-pi-runtime"

ros_domain_id=""

Fail() {
  echo "Adaptive Mentor Pi shell error: $*" >&2
  exit 1
}

if [[ "$#" == 2 && "$1" == "--ros-domain-id" ]]; then
  ros_domain_id="$2"
else
  echo "Usage: open_runtime_shell.sh --ros-domain-id 0..232" >&2
  exit 2
fi
[[ "${ros_domain_id}" =~ ^(0|[1-9][0-9]{0,2})$ ]] && \
  ((ros_domain_id <= 232)) || Fail "ROS domain ID must be in [0, 232]"

if [[ "$(docker container inspect "${CONTAINER_NAME}" \
    --format '{{.State.Running}}' 2>/dev/null || true)" == "true" ]]; then
  runtime_domain="$(docker container inspect "${CONTAINER_NAME}" \
    --format '{{range .Config.Env}}{{println .}}{{end}}' | \
    sed -n 's/^ROS_DOMAIN_ID=//p')"
  [[ "${runtime_domain}" == "${ros_domain_id}" ]] || \
    Fail "running container uses ROS_DOMAIN_ID=${runtime_domain}, not ${ros_domain_id}"
  exec docker exec --interactive --tty "${CONTAINER_NAME}" \
    /bin/bash -lc \
    'set -e
     source /opt/ros/humble/setup.bash
     source /opt/mentor_pi/micro_ros_agent/local_setup.bash
     source /opt/mentor_pi/host/setup.bash
     ros2 daemon stop >/dev/null 2>&1 || true
     export ZDOTDIR=/opt/mentor_pi/zsh
     exec /usr/bin/zsh -d -i'
fi

if command -v docker >/dev/null 2>&1 && \
    docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  Fail "runtime container exists but is not running; remove that exact stopped container"
fi
Fail "start make start in another terminal before opening the Docker ROS shell"
