#!/usr/bin/env bash

set -euo pipefail

mode=""
path=""

Fail() {
  echo "RDK handoff selection error: $*" >&2
  exit 1
}

Usage() {
  echo "Usage: select_rdk_handoff.sh --latest-under ABSOLUTE_DIRECTORY | --verify ABSOLUTE_HANDOFF" >&2
  exit 2
}

[[ "$#" == 2 ]] || Usage
case "$1" in
  --latest-under | --verify) mode="$1"; path="$2" ;;
  *) Usage ;;
esac
[[ "${path}" == /* ]] || Fail "path must be absolute"

if [[ "${mode}" == --latest-under ]]; then
  [[ -d "${path}" && ! -L "${path}" ]] || \
    Fail "received-handoff directory is missing or symbolic: ${path}"
  mapfile -t candidates < <(find "${path}" -mindepth 1 -maxdepth 1 \
    -type d -name 'rdk-arm64-????????T??????Z' -print | LC_ALL=C sort)
  ((${#candidates[@]} > 0)) || \
    Fail "no timestamped RDK handoff exists under ${path}"
  path="${candidates[${#candidates[@]} - 1]}"
fi

[[ -d "${path}" && ! -L "${path}" ]] || \
  Fail "RDK handoff is missing or symbolic: ${path}"
path="$(cd "${path}" && pwd -P)"
readonly name="$(basename "${path}")"
[[ "${name}" =~ ^rdk-arm64-[0-9]{8}T[0-9]{6}Z$ ]] || \
  Fail "RDK handoff name is not timestamped: ${name}"
readonly metadata="${path}/RDK-HANDOFF.txt"
readonly manifest="${path}/SHA256SUMS"
[[ -f "${metadata}" && ! -L "${metadata}" && \
   -f "${manifest}" && ! -L "${manifest}" ]] || \
  Fail "RDK handoff metadata or checksum manifest is missing or symbolic"
(cd "${path}" && sha256sum --check SHA256SUMS >/dev/null) || \
  Fail "RDK handoff checksum verification failed"
for expected in \
    'package_format=rrclite-rdk-handoff-v1' \
    "release_id=${name}" \
    'build_execution=qemu-emulated' \
    'build_host_architecture=amd64' \
    'target_architecture=arm64' \
    'native_target_validated=0'; do
  grep -Fqx "${expected}" "${metadata}" || \
    Fail "RDK handoff metadata is missing ${expected}"
done
[[ -d "${path}/board-handoff" && ! -L "${path}/board-handoff" ]] || \
  Fail "RDK handoff board package is missing or symbolic"

printf '%s\n' "${path}"
