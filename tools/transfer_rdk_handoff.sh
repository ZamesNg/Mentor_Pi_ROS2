#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SELECTOR="${SCRIPT_DIR}/select_rdk_handoff.sh"

login=""
handoff=""
remote_directory="/home/sunrise/Mentor_Pi/build/received-handoffs"

Fail() {
  echo "RDK handoff transfer error: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: transfer_rdk_handoff.sh --login USER@HOST
  [--handoff ABSOLUTE_RDK_HANDOFF] [--remote-directory ABSOLUTE_PATH]
EOF
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --login) login="${2:-}"; shift 2 ;;
    --handoff) handoff="${2:-}"; shift 2 ;;
    --remote-directory) remote_directory="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${login}" =~ ^[A-Za-z0-9._-]+@[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || \
  Fail "--login must be USER@HOST (use SSH config for ports or advanced routing)"
[[ "${remote_directory}" == /* && \
   "${remote_directory}" =~ ^/[A-Za-z0-9._/-]+$ && \
   "${remote_directory}" != *'/../'* && \
   "${remote_directory}" != */.. && \
   "${remote_directory}" != *'/./'* ]] || \
  Fail "remote directory must be a normalized absolute path without spaces"
for command in ssh scp tar sha256sum; do
  command -v "${command}" >/dev/null 2>&1 || Fail "${command} is required"
done
[[ -x "${SELECTOR}" ]] || Fail "RDK handoff selector is unavailable"

if [[ -n "${handoff}" ]]; then
  [[ "${handoff}" == /* ]] || Fail "--handoff must be an absolute path"
  bundle="$("${SELECTOR}" --verify "${handoff}")"
else
  bundle="$("${SELECTOR}" --latest-under \
    "${PROJECT_ROOT}/build/rdk-handoff")"
fi
readonly bundle
readonly bundle_parent="$(dirname "${bundle}")"
readonly bundle_name="$(basename "${bundle}")"
readonly archive="${bundle}.tar"
readonly checksum="${archive}.sha256"

reuse_archive=0
if [[ -e "${archive}" || -L "${archive}" || \
      -e "${checksum}" || -L "${checksum}" ]]; then
  [[ -f "${archive}" && ! -L "${archive}" && \
     -f "${checksum}" && ! -L "${checksum}" ]] || \
    Fail "existing archive output is incomplete or symbolic: ${archive}"
  if (cd "${bundle_parent}" && \
      sha256sum --check "${bundle_name}.tar.sha256" >/dev/null) && \
      tar -xOf "${archive}" "${bundle_name}/SHA256SUMS" | \
        cmp -s - "${bundle}/SHA256SUMS"; then
    reuse_archive=1
    echo "Reusing verified RDK handoff archive: ${archive}"
  else
    echo "Replacing stale RDK handoff archive: ${archive}"
  fi
fi
if [[ "${reuse_archive}" == 0 ]]; then
  archive_staging="$(mktemp "${bundle_parent}/.${bundle_name}.tar.XXXXXX")"
  checksum_staging="$(mktemp "${bundle_parent}/.${bundle_name}.sha256.XXXXXX")"
  cleanup=1
  Cleanup() {
    if [[ "${cleanup}" == 1 ]]; then
      rm -f -- "${archive_staging}" "${checksum_staging}"
    fi
  }
  trap Cleanup EXIT
  tar -C "${bundle_parent}" -cpf "${archive_staging}" "${bundle_name}"
  "${SELECTOR}" --verify "${bundle}" >/dev/null
  digest="$(sha256sum "${archive_staging}" | awk '{print $1}')"
  printf '%s  %s\n' "${digest}" "${bundle_name}.tar" >"${checksum_staging}"
  mv -f "${archive_staging}" "${archive}"
  mv -f "${checksum_staging}" "${checksum}"
  cleanup=0
  trap - EXIT
  echo "Created checksummed RDK handoff archive: ${archive}"
fi

ssh -- "${login}" "mkdir -p -- '${remote_directory}'"
scp -- "${archive}" "${checksum}" "${login}:${remote_directory}/"
echo "Transferred ${bundle_name} to ${login}:${remote_directory}/"
