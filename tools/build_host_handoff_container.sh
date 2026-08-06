#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"

architecture=""
output_directory=""
release_id=""
image=""

Usage() {
  cat >&2 <<'EOF'
Usage: build_host_handoff_container.sh --architecture amd64|arm64
  --release-id SAFE_ID --output-directory PATH [--image PINNED_IMAGE@sha256:DIGEST]
       build_host_handoff_container.sh --print-default-image \
         --architecture amd64|arm64

The exact image must already be present locally. The build runs without network
access and does not install into /opt, contact systemd, or access hardware.
EOF
  exit 2
}

if [[ "$#" -eq 3 && "$1" == "--print-default-image" && \
  "$2" == "--architecture" ]]; then
  "${IMAGE_SELECTOR}" host "$3"
  exit 0
fi

Fail() {
  echo "Host container build error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --architecture) architecture="${2:-}"; shift 2 ;;
    --release-id) release_id="${2:-}"; shift 2 ;;
    --output-directory) output_directory="${2:-}"; shift 2 ;;
    --image) image="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${architecture}" == "amd64" || "${architecture}" == "arm64" ]] || Usage
[[ -x "${IMAGE_SELECTOR}" ]] || Fail "pinned image selector is unavailable"
if [[ -z "${image}" ]]; then
  image="${MENTOR_PI_HOST_BUILDER_IMAGE:-$(
    "${IMAGE_SELECTOR}" host "${architecture}"
  )}"
fi
[[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] ||
  Fail "release ID must contain 1-64 safe filename characters"
[[ -n "${output_directory}" ]] || Usage
[[ "${image}" =~ @sha256:[0-9a-f]{64}$ ]] ||
  Fail "builder image must be pinned by a sha256 manifest digest"
command -v docker >/dev/null 2>&1 || Fail "docker is not installed"
docker image inspect "${image}" >/dev/null 2>&1 ||
  Fail "pinned image is not local; explicitly run: docker pull ${image}"
readonly IMAGE_ARCH="$(docker image inspect "${image}" \
  --format '{{.Architecture}}')"
[[ "${IMAGE_ARCH}" == "${architecture}" ]] || \
  Fail "pinned image architecture ${IMAGE_ARCH} does not match ${architecture}"

if [[ "${output_directory}" == /* ]]; then
  output_candidate="${output_directory}"
else
  output_candidate="${PROJECT_ROOT}/${output_directory}"
fi
mkdir -p "$(dirname "${output_candidate}")"
output_parent="$(cd "$(dirname "${output_candidate}")" && pwd -P)"
output_name="$(basename "${output_candidate}")"
readonly OUTPUT_ROOT="${output_parent}/${output_name}"
case "${OUTPUT_ROOT}/" in
  "${PROJECT_ROOT}/"*) ;;
  *) Fail "container output must remain inside the project workspace" ;;
esac
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] ||
  Fail "refusing to replace existing output ${OUTPUT_ROOT}"
readonly OUTPUT_RELATIVE="${OUTPUT_ROOT#"${PROJECT_ROOT}/"}"

readonly WORK_RELATIVE="build/host-handoff-work/${release_id}-${architecture}"
readonly WORK_ROOT="${PROJECT_ROOT}/${WORK_RELATIVE}"
[[ ! -e "${WORK_ROOT}" && ! -L "${WORK_ROOT}" ]] ||
  Fail "refusing to replace existing work directory ${WORK_ROOT}"
readonly PREFIX_RELATIVE="${WORK_RELATIVE}/prefix"
readonly BUILD_RELATIVE="${WORK_RELATIVE}/colcon"
readonly CONTAINER_PLATFORM="linux/${architecture}"
readonly CALLER_UID="$(id -u)"
readonly CALLER_GID="$(id -g)"

docker run --rm --network=none \
  --platform "${CONTAINER_PLATFORM}" \
  --env SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-0}" \
  --env MENTOR_PI_CALLER_UID="${CALLER_UID}" \
  --env MENTOR_PI_CALLER_GID="${CALLER_GID}" \
  --env MENTOR_PI_HOST_BUILDER_IMAGE="${image}" \
  --env MENTOR_PI_OUTPUT_RELATIVE="${OUTPUT_RELATIVE}" \
  --env MENTOR_PI_PREFIX_RELATIVE="${PREFIX_RELATIVE}" \
  --env MENTOR_PI_BUILD_RELATIVE="${BUILD_RELATIVE}" \
  --env MENTOR_PI_RELEASE_ID="${release_id}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${image}" \
  /workspace/tools/host_handoff_container_entrypoint.sh

echo "Host handoff completed without network or hardware access: ${OUTPUT_ROOT}"
