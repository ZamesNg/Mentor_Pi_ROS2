#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly -a ORIGINAL_ARGUMENTS=("$@")

architecture=""
print_component=""
include_quality=0

Fail() {
  echo "Build-image preparation error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --architecture) architecture="${2:-}"; shift 2 ;;
    --include-quality) include_quality=1; shift ;;
    --print) print_component="${2:-}"; shift 2 ;;
    *) Fail "usage: prepare_build_images.sh --architecture amd64|arm64 [--include-quality] [--print firmware|microros|quality]" ;;
  esac
done
[[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
  Fail "architecture must be amd64 or arm64"
[[ -z "${print_component}" || "${print_component}" == firmware || \
   "${print_component}" == microros || "${print_component}" == quality ]] || \
  Fail "unsupported image component"

ImageName() {
  local component="$1"
  local dockerfile=""
  local image_repository=""
  case "${component}" in
    firmware)
      dockerfile="${SCRIPT_DIR}/docker/firmware-builder.Dockerfile"
      image_repository=rrclite-firmware-builder
      ;;
    microros)
      dockerfile="${SCRIPT_DIR}/docker/microros-builder.Dockerfile"
      image_repository=micro-ros-static-library-builder
      ;;
    quality)
      dockerfile="${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
      image_repository=rrclite-quality-tests
      ;;
    *) Fail "unsupported image component: ${component}" ;;
  esac
  local source_sha=""
  source_sha="$(sha256sum "${dockerfile}" | awk '{print $1}')"
  printf 'mentor-pi/%s:docker-%s-%s\n' \
    "${image_repository}" "${architecture}" "${source_sha:0:16}"
}

if [[ -n "${print_component}" ]]; then
  ImageName "${print_component}"
  exit 0
fi
if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "${ORIGINAL_ARGUMENTS[@]}"
fi

case "$(uname -m)" in
  x86_64 | amd64) native_architecture=amd64 ;;
  aarch64 | arm64) native_architecture=arm64 ;;
  *) Fail "unsupported native architecture: $(uname -m)" ;;
esac
[[ "${architecture}" == "${native_architecture}" ]] || \
  Fail "cross-architecture image builds are forbidden; requested ${architecture} on ${native_architecture}"

command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker is not running or accessible"

components=(firmware microros)
((include_quality == 0)) || components+=(quality)
for component in "${components[@]}"; do
  if [[ "${component}" == firmware || "${component}" == quality ]]; then
    dockerfile="${SCRIPT_DIR}/docker/firmware-builder.Dockerfile"
    base_image="$("${IMAGE_SELECTOR}" ubuntu "${architecture}")"
    [[ "${component}" != quality ]] || \
      dockerfile="${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
  else
    dockerfile="${SCRIPT_DIR}/docker/microros-builder.Dockerfile"
    base_image="$("${IMAGE_SELECTOR}" microros "${architecture}")"
  fi
  dockerfile_sha="$(sha256sum "${dockerfile}" | awk '{print $1}')"
  image="$(ImageName "${component}")"
  if [[ "$(docker image inspect "${image}" --format \
      '{{index .Config.Labels "org.mentor-pi.build-image.base"}}' \
      2>/dev/null || true)" == "${base_image}" && \
        "$(docker image inspect "${image}" --format \
      '{{index .Config.Labels "org.mentor-pi.build-image.dockerfile-sha256"}}' \
      2>/dev/null || true)" == "${dockerfile_sha}" && \
        "$(docker image inspect "${image}" --format '{{.Os}}/{{.Architecture}}' \
      2>/dev/null || true)" == "linux/${architecture}" ]]; then
    echo "Reusing verified ${component} image: ${image}"
    continue
  fi
  build_command=(
    docker build --platform "linux/${architecture}"
    --label "org.mentor-pi.build-image.base=${base_image}"
    --label "org.mentor-pi.build-image.dockerfile-sha256=${dockerfile_sha}"
    --file "${dockerfile}" --tag "${image}"
  )
  if [[ "${component}" == firmware || "${component}" == microros ]]; then
    # BuildKit supplies TARGETARCH automatically, but the legacy Docker builder
    # does not. Pass the already validated native architecture explicitly so
    # both builders select the same pinned Arm GNU toolchain archive.
    build_command+=(--build-arg "TARGETARCH=${architecture}")
  fi
  use_host_network=0
  for proxy_variable in HTTP_PROXY HTTPS_PROXY NO_PROXY \
      http_proxy https_proxy no_proxy; do
    if [[ -n "${!proxy_variable:-}" ]]; then
      proxy_value="${!proxy_variable}"
      if [[ "${proxy_value}" == *127.0.0.1* || \
            "${proxy_value}" == *localhost* ]]; then
        use_host_network=1
      fi
      build_command+=(--build-arg "${proxy_variable}=${proxy_value}")
    fi
  done
  ((use_host_network == 0)) || build_command+=(--network host)
  build_command+=("${PROJECT_ROOT}/tools/docker")
  "${build_command[@]}"
  [[ "$(docker image inspect "${image}" --format '{{.Os}}/{{.Architecture}}')" == \
    "linux/${architecture}" ]] || Fail "built ${component} image has the wrong platform"
done

echo "Pinned Docker build images are ready for linux/${architecture}."
