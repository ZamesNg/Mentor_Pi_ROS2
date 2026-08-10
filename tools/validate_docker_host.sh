#!/usr/bin/env bash

set -euo pipefail

os_release=""
architecture=""

Fail() {
  echo "Docker host validation error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --os-release) os_release="${2:-}"; shift 2 ;;
    --architecture) architecture="${2:-}"; shift 2 ;;
    *) Fail "usage: validate_docker_host.sh --os-release PATH --architecture amd64|arm64" ;;
  esac
done
[[ -f "${os_release}" && -r "${os_release}" ]] || Fail "OS identity is unavailable"
[[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
  Fail "architecture must be amd64 or arm64"

ReadOsValue() {
  local key="$1"
  local count=""
  local value=""
  count="$(grep -Ec "^${key}=" "${os_release}" || true)"
  [[ "${count}" == 1 ]] || Fail "OS identity must contain exactly one ${key} entry"
  value="$(sed -n "s/^${key}=//p" "${os_release}")"
  value="${value#\"}"
  value="${value%\"}"
  printf '%s' "${value}"
}

[[ "$(ReadOsValue ID)" == ubuntu ]] || Fail "the host must be Ubuntu"
version="$(ReadOsValue VERSION_ID)"
[[ "${version}" == 22.04 || "${version}" == 24.04 ]] || \
  Fail "supported hosts are Ubuntu 22.04 and Ubuntu 24.04"

printf '%s\n' \
  "ubuntu=${version}" \
  "architecture=${architecture}" \
  'runtime=docker-humble' \
  'host_ros=unused'
