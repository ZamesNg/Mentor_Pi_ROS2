#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly HOST_RUNTIME_BUILDER="${SCRIPT_DIR}/build_host_runtime_image.sh"
readonly -a BUILD_DOCKERFILE_COMPONENTS=(
  "ubuntu:${SCRIPT_DIR}/docker/firmware-builder.Dockerfile"
  "microros:${SCRIPT_DIR}/docker/microros-builder.Dockerfile"
  "ubuntu:${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
)

architecture=""
dry_run=0

Fail() {
  echo "RRCLite pinned-image setup failed: $*" >&2
  exit 1
}

PullImage() {
  local image="$1"
  local source="$2"
  [[ "${image}" =~ ^[^[:space:]@]+@sha256:[0-9a-f]{64}$ ]] || \
    Fail "${source} does not identify an image pinned by a sha256 digest"
  if [[ "${dry_run}" == "1" ]]; then
    printf 'docker pull --platform %q %q # %s\n' \
      "linux/${architecture}" "${image}" "${source}"
    return
  fi
  echo "Pulling ${image} for linux/${architecture} (${source})."
  docker pull --platform "linux/${architecture}" "${image}"
}

while (($# > 0)); do
  case "$1" in
    --architecture)
      (($# >= 2)) || \
        Fail "--architecture requires amd64 or arm64"
      architecture="${2:-}"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *)
      Fail "usage: ./tools/pull_pinned_build_images.sh --architecture amd64|arm64 [--dry-run]"
      ;;
  esac
done
[[ "${architecture}" == "amd64" || "${architecture}" == "arm64" ]] || \
  Fail "architecture must be amd64 or arm64"
if [[ "${dry_run}" == "0" ]]; then
  command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
  docker info >/dev/null 2>&1 || \
    Fail "Docker Desktop/Engine is not running or is not accessible"
fi
[[ -x "${IMAGE_SELECTOR}" ]] || \
  Fail "pinned image selector is missing or not executable"

host_image="$(${IMAGE_SELECTOR} host "${architecture}")" || \
  Fail "could not read the pinned Humble host image"
PullImage "${host_image}" "host build"

for component_and_dockerfile in "${BUILD_DOCKERFILE_COMPONENTS[@]}"; do
  component="${component_and_dockerfile%%:*}"
  dockerfile="${component_and_dockerfile#*:}"
  [[ -f "${dockerfile}" ]] || Fail "Dockerfile is missing: ${dockerfile}"
  base_image="$(awk '$1 == "FROM" {print $2; exit}' "${dockerfile}")"
  [[ -n "${base_image}" ]] || \
    Fail "Dockerfile has no base image: ${dockerfile}"
  case "${component}" in
    ubuntu)
      expected_index='ubuntu:24.04@sha256:561618e2c15bf2397621dd04f96926663a3b5616c189cf7e38db7e82f5c538ea'
      ;;
    microros)
      expected_index='microros/micro_ros_static_library_builder:humble@sha256:e291f74890e81b31eb1d70731cb79b2d767dd585269325031effc72952b24b9d'
      ;;
    *) Fail "unsupported Dockerfile image component: ${component}" ;;
  esac
  [[ "${base_image}" == "${expected_index}" ]] || \
    Fail "Dockerfile base differs from its reviewed multi-platform index: ${dockerfile}"
  child_image="$("${IMAGE_SELECTOR}" "${component}" "${architecture}")"
  relative_dockerfile="${dockerfile#"${PROJECT_ROOT}/"}"
  PullImage "${child_image}" "${relative_dockerfile}"
done

if [[ "${dry_run}" == "1" ]]; then
  printf './tools/build_host_runtime_image.sh --architecture %q\n' \
    "${architecture}"
  echo "Pinned RRCLite image pull plan is valid for linux/${architecture}."
else
  "${HOST_RUNTIME_BUILDER}" --architecture "${architecture}"
  echo "Pinned RRCLite build images are present locally for linux/${architecture}."
fi
