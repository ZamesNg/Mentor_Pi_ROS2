#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly OH_MY_ZSH_COMMIT="97b27bb2ec0701330b18c2d3e340b22e742b3fa8"

Fail() {
  echo "Host runtime image test failed: $*" >&2
  exit 1
}

architecture=""
image=""
host_prefix=""
agent_prefix=""
prepared=0
if (($# > 0)); then
  while (($# > 0)); do
    case "$1" in
      --architecture) architecture="${2:-}"; shift 2 ;;
      --image) image="${2:-}"; shift 2 ;;
      --host-prefix) host_prefix="${2:-}"; shift 2 ;;
      --agent-prefix) agent_prefix="${2:-}"; shift 2 ;;
      *) Fail "usage: test_host_runtime_image.sh [--architecture amd64|arm64 --image IMAGE --host-prefix ABSOLUTE --agent-prefix ABSOLUTE]" ;;
    esac
  done
  [[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
    Fail "prepared runtime architecture must be amd64 or arm64"
  [[ -n "${image}" && "${host_prefix}" == /* && "${agent_prefix}" == /* ]] || \
    Fail "prepared runtime image and absolute prefixes are required"
  [[ -d "${host_prefix}" && ! -L "${host_prefix}" && \
     -d "${agent_prefix}" && ! -L "${agent_prefix}" ]] || \
    Fail "prepared runtime prefixes are missing or symbolic"
  prepared=1
fi

if ((prepared == 0)); then
case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
  *) Fail "the native architecture must be amd64 or arm64" ;;
esac

"${SCRIPT_DIR}/prepare_build_images.sh" --architecture "${architecture}"
"${SCRIPT_DIR}/build_host.sh" --runtime
"${SCRIPT_DIR}/build_agent.sh"

image="$(
  "${SCRIPT_DIR}/prepare_build_images.sh" \
    --architecture "${architecture}" --print project
)"
host_prefix="$(${SCRIPT_DIR}/build_host.sh --runtime --print-output)"
agent_prefix="$(${SCRIPT_DIR}/build_agent.sh --print-output)"
fi
readonly architecture image host_prefix agent_prefix

[[ "$(docker image inspect "${image}" --format '{{.Architecture}}')" == \
  "${architecture}" ]] || Fail "runtime image architecture is wrong"
[[ "$(docker image inspect "${image}" --format '{{json .Config.Cmd}}')" == \
  '["/usr/bin/zsh"]' ]] || Fail "runtime image default command is not zsh"

autosuggestions_line="$(
  grep -n '^source /usr/share/zsh-autosuggestions/' \
    "${SCRIPT_DIR}/docker/host-runtime.zshrc" | cut -d: -f1
)"
highlighting_line="$(
  grep -n '^source /usr/share/zsh-syntax-highlighting/' \
    "${SCRIPT_DIR}/docker/host-runtime.zshrc" | cut -d: -f1
)"
[[ -n "${autosuggestions_line}" && -n "${highlighting_line}" && \
  "${autosuggestions_line}" -lt "${highlighting_line}" ]] || \
  Fail "autosuggestions must load before syntax highlighting"

docker run --rm \
  --platform "linux/${architecture}" \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,noexec \
  --volume "${host_prefix}:/opt/mentor_pi/host:ro" \
  --volume "${agent_prefix}:/opt/mentor_pi/micro_ros_agent:ro" \
  --env HOME=/tmp/mentor-pi-home \
  --env ZDOTDIR=/opt/mentor_pi/zsh \
  --env "MENTOR_PI_EXPECTED_OMZ_COMMIT=${OH_MY_ZSH_COMMIT}" \
  --entrypoint /usr/bin/zsh \
  "${image}" -d -i -c '
    set -e
    [[ "$(cat /opt/mentor_pi/oh-my-zsh/MENTOR-PI-COMMIT)" == \
      "${MENTOR_PI_EXPECTED_OMZ_COMMIT}" ]]
    [[ "${ROS_DISTRO:-}" == humble ]]
    [[ "${AMENT_PREFIX_PATH:-}" == \
      /opt/mentor_pi/host:/opt/mentor_pi/micro_ros_agent:/opt/ros/humble ]]
    (( $+functions[_git] ))
    (( $+functions[_make] ))
    (( $+functions[_path_files] ))
    (( $+functions[_python_argcomplete] ))
    [[ "${_comps[ros2]-}" == *python_argcomplete* ]]
    (( $+functions[_zsh_autosuggest_start] ))
    (( $+functions[_zsh_highlight] ))
    command -v ros2 git make >/dev/null
  '

echo "Host runtime image, overlays, zsh, plugins, and completion passed."
