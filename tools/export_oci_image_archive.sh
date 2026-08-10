#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CONVERTER="${SCRIPT_DIR}/export_oci_image_archive.py"

Fail() {
  echo "OCI image export error: $*" >&2
  exit 1
}

print_runtime_id=0
if [[ "${1:-}" == "--print-runtime-id" ]]; then
  print_runtime_id=1
  shift
fi
[[ "$#" -eq 2 ]] || \
  Fail "usage: export_oci_image_archive.sh [--print-runtime-id] IMAGE OUTPUT.tar"
readonly IMAGE="$1"
readonly OUTPUT="$2"
[[ "${OUTPUT}" == /* ]] || Fail "output path must be absolute"
[[ ! -e "${OUTPUT}" && ! -L "${OUTPUT}" ]] || \
  Fail "output path already exists: ${OUTPUT}"
command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
command -v python3 >/dev/null 2>&1 || Fail "Python 3 is required"
[[ -x "${CONVERTER}" ]] || Fail "OCI archive converter is unavailable"

readonly IMAGE_ID="$(docker image inspect "${IMAGE}" --format '{{.Id}}' \
  2>/dev/null || true)"
readonly IMAGE_OS="$(docker image inspect "${IMAGE}" --format '{{.Os}}' \
  2>/dev/null || true)"
readonly IMAGE_ARCHITECTURE="$(docker image inspect "${IMAGE}" \
  --format '{{.Architecture}}' 2>/dev/null || true)"
IMAGE_REFERENCE="$(docker image inspect "${IMAGE}" \
  --format '{{index .RepoTags 0}}' 2>/dev/null || true)"
if [[ -z "${IMAGE_REFERENCE}" || "${IMAGE_REFERENCE}" == '<no value>' ]]; then
  IMAGE_REFERENCE="mentor-pi/runtime-handoff:${IMAGE_ID#sha256:}"
fi
readonly IMAGE_REFERENCE
[[ "${IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ && \
   "${IMAGE_OS}" == linux && \
   ("${IMAGE_ARCHITECTURE}" == amd64 || \
    "${IMAGE_ARCHITECTURE}" == arm64) && \
   "${IMAGE_REFERENCE}" == *:* ]] || \
  Fail "image identity or platform is invalid"

docker_archive="$(mktemp "${TMPDIR:-/tmp}/mentor-pi-docker-image.XXXXXX.tar")"
cleanup_output=1
Cleanup() {
  rm -f -- "${docker_archive}"
  if [[ "${cleanup_output}" == 1 ]]; then
    rm -f -- "${OUTPUT}"
  fi
}
trap Cleanup EXIT

docker save --output "${docker_archive}" "${IMAGE}"
runtime_image_id="$(python3 "${CONVERTER}" \
  --docker-archive "${docker_archive}" \
  --output "${OUTPUT}" \
  --image-id "${IMAGE_ID}" \
  --os "${IMAGE_OS}" \
  --architecture "${IMAGE_ARCHITECTURE}" \
  --reference "${IMAGE_REFERENCE}")"
[[ "${runtime_image_id}" =~ ^sha256:[0-9a-f]{64}$ && \
   "${runtime_image_id}" != *$'\n'* ]] || \
  Fail "converter reported an invalid exported runtime image ID"
tar -tf "${OUTPUT}" oci-layout >/dev/null 2>&1 || \
  Fail "exported archive lacks the OCI layout marker"
tar -tf "${OUTPUT}" index.json >/dev/null 2>&1 || \
  Fail "exported archive lacks the OCI index"

cleanup_output=0
if [[ "${print_runtime_id}" == 1 ]]; then
  printf '%s\n' "${runtime_image_id}"
else
  echo "Exported OCI runtime image ${runtime_image_id} (source ${IMAGE_ID}): ${OUTPUT}"
fi
