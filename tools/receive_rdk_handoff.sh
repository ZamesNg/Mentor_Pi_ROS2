#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly SELECTOR="${SCRIPT_DIR}/select_rdk_handoff.sh"
readonly DEFAULT_SEARCH_ROOT="${PROJECT_ROOT}/build/received-handoffs"

handoff=""

Fail() {
  echo "RDK handoff receipt error: $*" >&2
  exit 1
}

Usage() {
  echo "Usage: receive_rdk_handoff.sh [--handoff ABSOLUTE_RDK_HANDOFF]" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --handoff) handoff="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

case "$(uname -m)" in
  aarch64 | arm64) ;;
  *) Fail "received production handoffs require the arm64 RDK X5" ;;
esac
grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "RDK host must run Ubuntu"
grep -Eq '^VERSION_ID=\"?22[.]04\"?$' /etc/os-release || \
  Fail "RDK host must run Ubuntu 22.04"
"${SCRIPT_DIR}/detect_host_profile.sh" | grep -Fqx 'profile=rdk-x5' || \
  Fail "received production handoffs require the detected RDK X5 profile"
command -v docker >/dev/null 2>&1 || Fail "Docker is unavailable"
docker info >/dev/null 2>&1 || Fail "Docker Engine is unavailable"
[[ -x "${SELECTOR}" ]] || Fail "RDK handoff selector is unavailable"

if [[ -n "${handoff}" ]]; then
  [[ "${handoff}" == /* ]] || Fail "--handoff must be an absolute path"
  handoff="$("${SELECTOR}" --verify "${handoff}")"
else
  handoff="$("${SELECTOR}" --latest-received-under \
    "${DEFAULT_SEARCH_ROOT}")"
fi

[[ "$(dirname "${handoff}")" == "${DEFAULT_SEARCH_ROOT}" ]] || \
  Fail "verified handoff must be directly under ${DEFAULT_SEARCH_ROOT}"
readonly receipt="${DEFAULT_SEARCH_ROOT}/VERIFIED-RDK-HANDOFF.txt"
readonly manifest_sha="$(sha256sum "${handoff}/SHA256SUMS" | awk '{print $1}')"
receipt_staging="$(mktemp "${DEFAULT_SEARCH_ROOT}/.verified-handoff.XXXXXX")"
printf 'handoff_name=%s\nmanifest_sha256=%s\n' \
  "$(basename "${handoff}")" "${manifest_sha}" >"${receipt_staging}"
chmod 0644 "${receipt_staging}"
mv -f "${receipt_staging}" "${receipt}"

echo "Verified received RDK handoff: ${handoff}" >&2
printf '%s\n' "${handoff}"
