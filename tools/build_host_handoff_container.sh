#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly HOST_RUNTIME_BUILDER="${SCRIPT_DIR}/build_host_runtime_image.sh"
readonly HOST_RUNTIME_DOCKERFILE="${SCRIPT_DIR}/docker/host-runtime.Dockerfile"

architecture=""
output_directory=""
release_id=""
image=""

Usage() {
  cat >&2 <<'EOF'
Usage: build_host_handoff_container.sh --architecture amd64|arm64
  --release-id SAFE_ID --output-directory PATH [--image PREPARED_IMAGE]
       build_host_handoff_container.sh --print-default-image \
         --architecture amd64|arm64

The prepared Humble/ros2_control image must already be present locally. The
build runs without network access and does not install into /opt, contact
systemd, or access hardware.
EOF
  exit 2
}

if [[ "$#" -eq 3 && "$1" == "--print-default-image" && \
  "$2" == "--architecture" ]]; then
  "${HOST_RUNTIME_BUILDER}" --architecture "$3" --print-output
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
[[ -x "${HOST_RUNTIME_BUILDER}" && -f "${HOST_RUNTIME_DOCKERFILE}" ]] || \
  Fail "Humble host runtime image tooling is unavailable"
readonly DEFAULT_RUNTIME_IMAGE="$(
  "${HOST_RUNTIME_BUILDER}" --architecture "${architecture}" --print-output
)"
readonly PINNED_BASE_IMAGE="$("${IMAGE_SELECTOR}" host "${architecture}")"
if [[ -z "${image}" ]]; then
  image="${MENTOR_PI_HOST_RUNTIME_IMAGE:-${DEFAULT_RUNTIME_IMAGE}}"
fi
[[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] ||
  Fail "release ID must contain 1-64 safe filename characters"
[[ -n "${output_directory}" ]] || Usage
command -v docker >/dev/null 2>&1 || Fail "docker is not installed"
docker image inspect "${image}" >/dev/null 2>&1 ||
  Fail "prepared image is not local; run make setup"
readonly IMAGE_ARCH="$(docker image inspect "${image}" \
  --format '{{.Architecture}}')"
[[ "${IMAGE_ARCH}" == "${architecture}" ]] || \
  Fail "prepared image architecture ${IMAGE_ARCH} does not match ${architecture}"
builder_identity=""
if [[ "${image}" == "${DEFAULT_RUNTIME_IMAGE}" ]]; then
  readonly RUNTIME_DOCKERFILE_SHA="$(sha256sum \
    "${HOST_RUNTIME_DOCKERFILE}" | awk '{print $1}')"
  [[ "$(docker image inspect "${image}" \
      --format '{{index .Config.Labels "org.mentor-pi.host-runtime.base"}}')" == \
      "${PINNED_BASE_IMAGE}" ]] || \
    Fail "prepared runtime image has the wrong pinned base"
  [[ "$(docker image inspect "${image}" \
      --format '{{index .Config.Labels "org.mentor-pi.host-runtime.dockerfile-sha256"}}')" == \
      "${RUNTIME_DOCKERFILE_SHA}" ]] || \
    Fail "prepared runtime image has the wrong Dockerfile fingerprint"
  builder_identity="${PINNED_BASE_IMAGE}"
elif [[ "${image}" =~ @sha256:[0-9a-f]{64}$ ]]; then
  builder_identity="${image}"
else
  Fail "custom prepared images must be pinned by a sha256 manifest digest"
fi
readonly builder_identity

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
  --env MENTOR_PI_HOST_BUILDER_IMAGE="${builder_identity}" \
  --env MENTOR_PI_OUTPUT_RELATIVE="${OUTPUT_RELATIVE}" \
  --env MENTOR_PI_PREFIX_RELATIVE="${PREFIX_RELATIVE}" \
  --env MENTOR_PI_BUILD_RELATIVE="${BUILD_RELATIVE}" \
  --env MENTOR_PI_RELEASE_ID="${release_id}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${image}" \
  /workspace/tools/host_handoff_container_entrypoint.sh

echo "Host handoff completed without network or hardware access: ${OUTPUT_ROOT}"
