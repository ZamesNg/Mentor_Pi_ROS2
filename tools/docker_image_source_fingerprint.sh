#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

component="${1:-}"
base_image="${2:-}"
project_root="${3:-${DEFAULT_PROJECT_ROOT}}"

Fail() {
  echo "Docker image fingerprint error: $*" >&2
  exit 1
}

[[ "${component}" == project || "${component}" == quality ]] || \
  Fail "usage: docker_image_source_fingerprint.sh project|quality BASE_IMAGE [PROJECT_ROOT]"
[[ -n "${base_image}" && "${base_image}" != *$'\n'* ]] || \
  Fail "base image identity must be one non-empty line"
[[ "${project_root}" == /* && -d "${project_root}" ]] || \
  Fail "project root must be an absolute directory"
command -v sha256sum >/dev/null 2>&1 || Fail "sha256sum is unavailable"

if [[ "${component}" == project ]]; then
  readonly -a relative_inputs=(
    tools/docker/rrclite.Dockerfile
    tools/docker/host-runtime.zshrc
    tools/docker/ros-humble-packages.lock
    tools/altro_source.lock
    tools/microros_agent_source.lock
    firmware/mentor_pi_mcu/config/microros_sources.lock
  )
else
  readonly -a relative_inputs=(tools/docker/quality-tests.Dockerfile)
fi

for relative in "${relative_inputs[@]}"; do
  [[ -f "${project_root}/${relative}" ]] || Fail "missing input ${relative}"
done

{
  printf 'format=rrclite-docker-image-source-v1\n'
  printf 'component=%s\n' "${component}"
  printf 'base=%s\n' "${base_image}"
  for relative in "${relative_inputs[@]}"; do
    printf '%s  %s\n' \
      "$(sha256sum "${project_root}/${relative}" | awk '{print $1}')" \
      "${relative}"
  done
} | sha256sum | awk '{print $1}'
