#!/usr/bin/env bash

set -euo pipefail

readonly -a CONTAINER_NAMES=(mentor-pi-runtime mentor-pi-production)

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

command -v docker >/dev/null 2>&1 || Fail "Docker is unavailable"

running_container=""
stopped_container=""
for container_name in "${CONTAINER_NAMES[@]}"; do
  container_state="$(docker container inspect "${container_name}" \
    --format '{{.State.Running}}' 2>/dev/null || true)"
  if [[ "${container_state}" == true ]]; then
    [[ -z "${running_container}" ]] || \
      Fail "both ${running_container} and ${container_name} are running; stop one runtime"
    running_container="${container_name}"
  elif docker container inspect "${container_name}" >/dev/null 2>&1; then
    stopped_container="${container_name}"
  fi
done

if [[ -n "${running_container}" ]]; then
  runtime_domain="$(docker container inspect "${running_container}" \
    --format '{{range .Config.Env}}{{println .}}{{end}}' | \
    sed -n 's/^ROS_DOMAIN_ID=//p')"
  [[ "${runtime_domain}" == "${ros_domain_id}" ]] || \
    Fail "running container uses ROS_DOMAIN_ID=${runtime_domain}, not ${ros_domain_id}"
  exec docker exec --interactive --tty "${running_container}" \
    /bin/bash -lc \
    'set -e
     source /opt/ros/humble/setup.bash
     source /opt/mentor_pi/micro_ros_agent/local_setup.bash
     source /opt/mentor_pi/host/setup.bash
     ros2 daemon stop >/dev/null 2>&1 || true
     export ZDOTDIR=/opt/mentor_pi/zsh
     exec /usr/bin/zsh -d -i'
fi

if [[ -n "${stopped_container}" ]]; then
  Fail "container ${stopped_container} exists but is not running; remove that exact stopped container"
fi
Fail "start make start or the RDK production target before opening the Docker ROS shell"
