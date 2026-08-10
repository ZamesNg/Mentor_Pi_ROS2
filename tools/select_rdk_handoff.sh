#!/usr/bin/env bash

set -euo pipefail

mode=""
path=""

Fail() {
  echo "RDK handoff selection error: $*" >&2
  exit 1
}

Usage() {
  echo "Usage: select_rdk_handoff.sh --latest-under ABSOLUTE_DIRECTORY | --latest-received-under ABSOLUTE_DIRECTORY | --verify ABSOLUTE_HANDOFF" >&2
  exit 2
}

[[ "$#" == 2 ]] || Usage
case "$1" in
  --latest-under | --latest-received-under | --verify) mode="$1"; path="$2" ;;
  *) Usage ;;
esac
[[ "${path}" == /* ]] || Fail "path must be absolute"

if [[ "${mode}" == --latest-under || "${mode}" == --latest-received-under ]]; then
  [[ -d "${path}" && ! -L "${path}" ]] || \
    Fail "received-handoff directory is missing or symbolic: ${path}"
  mapfile -t candidates < <(find "${path}" -mindepth 1 -maxdepth 1 \
    -type d -name 'rdk-arm64-????????T??????Z' -print | LC_ALL=C sort)
  if [[ "${mode}" == --latest-received-under ]]; then
    mapfile -t received_archives < <(find "${path}" -mindepth 1 -maxdepth 1 \
      -type f -name 'rdk-arm64-????????T??????Z.tar' -print | LC_ALL=C sort)
    for archive in "${received_archives[@]}"; do
      candidates+=("${archive%.tar}")
    done
    mapfile -t candidates < <(printf '%s\n' "${candidates[@]}" | LC_ALL=C sort -u)
  fi
  ((${#candidates[@]} > 0)) || \
    Fail "no timestamped RDK handoff exists under ${path}"
  path="${candidates[${#candidates[@]} - 1]}"

  if [[ "${mode}" == --latest-received-under && ! -d "${path}" ]]; then
    readonly archive="${path}.tar"
    readonly archive_checksum="${archive}.sha256"
    readonly archive_name="$(basename "${archive}")"
    readonly handoff_name="$(basename "${path}")"
    [[ -f "${archive}" && ! -L "${archive}" && \
       -f "${archive_checksum}" && ! -L "${archive_checksum}" ]] || \
      Fail "received RDK handoff archive or checksum is missing or symbolic: ${archive}"
    [[ "$(wc -l <"${archive_checksum}")" == 1 ]] || \
      Fail "received RDK handoff archive checksum is malformed: ${archive_checksum}"
    read -r checksum_entry <"${archive_checksum}"
    checksum_digest="${checksum_entry%%  *}"
    [[ "${checksum_digest}" =~ ^[0-9a-f]{64}$ && \
       "${checksum_entry}" == "${checksum_digest}  ${archive_name}" ]] || \
      Fail "received RDK handoff archive checksum names the wrong file"
    (cd "$(dirname "${archive}")" && \
      sha256sum --check --strict "$(basename "${archive_checksum}")" >/dev/null) || \
      Fail "received RDK handoff archive checksum verification failed"
    tar -tf "${archive}" >/dev/null || \
      Fail "received RDK handoff archive cannot be listed"
    while IFS= read -r member; do
      [[ "${member}" == "${handoff_name}" || \
         "${member}" == "${handoff_name}/"* ]] || \
        Fail "received RDK handoff archive contains an unexpected path: ${member}"
      [[ "/${member}/" != *'/../'* && "/${member}/" != *'/./'* ]] || \
        Fail "received RDK handoff archive contains an unsafe path: ${member}"
    done < <(tar -tf "${archive}")

    staging_root="$(mktemp -d "$(dirname "${archive}")/.rdk-receive.XXXXXX")"
    cleanup_staging=1
    CleanupStaging() {
      if [[ "${cleanup_staging}" == 1 ]]; then
        rm -rf -- "${staging_root}"
      fi
    }
    trap CleanupStaging EXIT
    tar -xpf "${archive}" -C "${staging_root}"
    "${BASH_SOURCE[0]}" --verify "${staging_root}/${handoff_name}" >/dev/null
    mv "${staging_root}/${handoff_name}" "${path}"
    cleanup_staging=0
    rmdir "${staging_root}"
    trap - EXIT
    echo "Verified and extracted received RDK handoff: ${path}" >&2
  fi
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
