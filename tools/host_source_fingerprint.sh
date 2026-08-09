#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT="${1:-${DEFAULT_PROJECT_ROOT}}"

Fail() {
  echo "Host source fingerprint error: $*" >&2
  exit 1
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    Fail "neither sha256sum nor shasum is installed"
  fi
}

[[ "$#" -le 1 ]] || Fail "usage: host_source_fingerprint.sh [PROJECT_ROOT]"
[[ -d "${PROJECT_ROOT}" ]] || Fail "project root does not exist: ${PROJECT_ROOT}"

readonly PATHS="$(mktemp)"
readonly MANIFEST="$(mktemp)"
trap 'rm -f "${PATHS}" "${MANIFEST}"' EXIT

AppendDirectory() {
  local directory="$1"
  [[ -d "${directory}" && ! -L "${directory}" ]] ||
    Fail "required source directory is missing or symbolic: ${directory}"
  local first_link=""
  first_link="$(find "${directory}" -type l -print -quit)"
  [[ -z "${first_link}" ]] || Fail "source symlink is unsupported: ${first_link}"
  find "${directory}" -type f \
    ! -name '*.pyc' \
    ! -name '.DS_Store' \
    ! -path '*/__pycache__/*' \
    -print >>"${PATHS}"
}

AppendFile() {
  local file="$1"
  [[ -f "${file}" && ! -L "${file}" ]] ||
    Fail "required source file is missing or symbolic: ${file}"
  printf '%s\n' "${file}" >>"${PATHS}"
}

AppendDirectory "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces"
AppendDirectory "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup"
AppendDirectory "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_hardwares"
AppendDirectory "${PROJECT_ROOT}/docs/tutorials"
for file in \
    Makefile \
    tools/bootstrap_native_arm_toolchain.sh \
    tools/build_agent.sh \
    tools/build_host_handoff_container.sh \
    tools/build_microros_agent_from_lock.sh \
    tools/build_host.sh \
    tools/build_host_runtime_image.sh \
    tools/build_host_release.sh \
    tools/host_build_container_entrypoint.sh \
    tools/host_handoff_container_entrypoint.sh \
    tools/host_source_fingerprint.sh \
    tools/docker/host-runtime.Dockerfile \
    tools/install_microros_agent.sh \
    tools/install_onboard_stm32cubeprogrammer.sh \
    tools/microros_agent_source.lock \
    tools/open_runtime_shell.sh \
    tools/onboard_colcon_state.sh \
    tools/package_host_handoff.sh \
    tools/patches/micro_xrce_agent_rrclite_modem_lines.patch \
    tools/prepare_host_build_dependencies.sh \
    tools/require_microros_agent_install_idle.sh \
    tools/run_runtime.sh \
    tools/select_pinned_build_image.sh \
    tools/setup_onboard_ros_environment.sh \
    tools/verify_host_build_environment.sh \
    tools/verify_host_release_relocation.sh \
    tools/verify_microros_agent_build_container.sh \
    tools/verify_microros_agent_build_in_container.sh \
    tools/test_active_build_policy.sh \
    tools/test_ros_workspace_layout.sh \
    tools/verify_microros_agent_install_state.sh; do
  AppendFile "${PROJECT_ROOT}/${file}"
done

LC_ALL=C sort -u "${PATHS}" | while IFS= read -r source; do
  [[ "${source}" != *$'\n'* ]] || Fail "newline in source path is unsupported"
  relative="${source#"${PROJECT_ROOT}/"}"
  [[ "${relative}" != "${source}" ]] ||
    Fail "source is outside project root: ${source}"
  printf '%s  %s\n' "$(Sha256 "${source}")" "${relative}"
done >"${MANIFEST}"

Sha256 "${MANIFEST}"
