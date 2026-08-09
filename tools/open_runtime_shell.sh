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
     export PS1="(mentor-pi-humble) \u@\h:\w\\$ "
     exec /bin/bash --noprofile --norc -i'
fi

if command -v docker >/dev/null 2>&1 && \
    docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  Fail "runtime container exists but is not running; remove that exact stopped container"
fi

grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "the host must be Ubuntu"
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  Fail "start make start in another terminal before opening a Docker ROS shell"
readonly host_prefix="$(${HOST_BUILDER} --print-output)"
readonly agent_prefix="$(${AGENT_BUILDER} --print-output)"
readonly agent_executable="${agent_prefix}/lib/micro_ros_agent/micro_ros_agent"
[[ -r "${host_prefix}/setup.bash" && -x "${agent_executable}" ]] || \
  Fail "run make host and make agent first"

set +u
source /opt/ros/humble/setup.bash
source "${agent_prefix}/local_setup.bash"
source "${host_prefix}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ros_domain_id}"
ros2 daemon stop >/dev/null 2>&1 || true
export MENTOR_PI_DEVELOPMENT_RUNTIME=1
export MENTOR_PI_PROJECT_ROOT="${PROJECT_ROOT}"
export MENTOR_PI_FIRMWARE_VERIFIER="${PROJECT_ROOT}/tools/verify_firmware_artifact.sh"
export MENTOR_PI_HOST_PREFIX="${host_prefix}"
export MENTOR_PI_AGENT_PREFIX="${agent_prefix}"
export MENTOR_PI_AGENT_EXECUTABLE="${agent_executable}"
export PS1="(mentor-pi-humble) ${PS1:-\\u@\\h:\\w\\$ }"
exec /bin/bash --noprofile --norc -i
