#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly PROJECT_DOCKERFILE="${SCRIPT_DIR}/docker/rrclite.Dockerfile"
readonly QUALITY_DOCKERFILE="${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
readonly ZSHRC="${SCRIPT_DIR}/docker/host-runtime.zshrc"
readonly ROS_LOCK="${SCRIPT_DIR}/docker/ros-humble-packages.lock"
readonly ALTO_LOCK="${SCRIPT_DIR}/altro_source.lock"
readonly AGENT_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly MICROROS_LOCK="${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_sources.lock"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly IMAGE_FINGERPRINT="${SCRIPT_DIR}/docker_image_source_fingerprint.sh"
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
    *) Fail "usage: prepare_build_images.sh --architecture amd64|arm64 [--include-quality] [--print project|quality]" ;;
  esac
done
[[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
  Fail "architecture must be amd64 or arm64"
[[ -z "${print_component}" || "${print_component}" == project || \
   "${print_component}" == quality ]] || Fail "unsupported image component"
[[ -f "${PROJECT_DOCKERFILE}" && -f "${QUALITY_DOCKERFILE}" && \
   -f "${ZSHRC}" && -f "${ROS_LOCK}" && -f "${ALTO_LOCK}" && \
   -f "${AGENT_LOCK}" && -f "${MICROROS_LOCK}" ]] || \
  Fail "Docker image inputs are missing"
[[ -x "${IMAGE_FINGERPRINT}" ]] || Fail "image fingerprint tool is missing"

# The lock is repository-owned data containing assignments only. Reject any
# other syntax before loading it so package arguments cannot execute code.
if grep -Ev '^[A-Z0-9_]+=[A-Za-z0-9.+:~-]+$|^$' "${ROS_LOCK}" | grep -q .; then
  Fail "ROS package lock contains unsupported syntax"
fi
# shellcheck disable=SC1090
source "${ROS_LOCK}"

LockedValue() {
  local suffix="$1"
  local prefix="${architecture^^}"
  local variable="${prefix}_${suffix}"
  local value="${!variable:-}"
  [[ -n "${value}" ]] || Fail "ROS package lock has no ${variable} entry"
  printf '%s\n' "${value}"
}

ProjectSourceSha() {
  local base_image="$1"
  "${IMAGE_FINGERPRINT}" project "${base_image}" "${PROJECT_ROOT}"
}

QualitySourceSha() {
  local base_image="$1"
  "${IMAGE_FINGERPRINT}" quality "${base_image}" "${PROJECT_ROOT}"
}

ImageName() {
  local component="$1"
  if [[ "${component}" == project ]]; then
    local base_image
    base_image="$("${IMAGE_SELECTOR}" microros "${architecture}")"
    local source_sha
    source_sha="$(ProjectSourceSha "${base_image}")"
    printf 'mentor-pi/rrclite:humble-%s-%s\n' \
      "${architecture}" "${source_sha:0:16}"
  elif [[ "${component}" == quality ]]; then
    local base_image
    base_image="$("${IMAGE_SELECTOR}" ubuntu "${architecture}")"
    local source_sha
    source_sha="$(QualitySourceSha "${base_image}")"
    printf 'mentor-pi/rrclite-quality:noble-%s-%s\n' \
      "${architecture}" "${source_sha:0:16}"
  else
    Fail "unsupported image component: ${component}"
  fi
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

components=(project)
((include_quality == 0)) || components+=(quality)
for component in "${components[@]}"; do
  if [[ "${component}" == project ]]; then
    dockerfile="${PROJECT_DOCKERFILE}"
    base_image="$("${IMAGE_SELECTOR}" microros "${architecture}")"
    source_sha="$(ProjectSourceSha "${base_image}")"
  else
    dockerfile="${QUALITY_DOCKERFILE}"
    base_image="$("${IMAGE_SELECTOR}" ubuntu "${architecture}")"
    source_sha="$(QualitySourceSha "${base_image}")"
  fi
  image="$(ImageName "${component}")"
  if [[ "$(docker image inspect "${image}" --format \
      '{{index .Config.Labels "org.mentor-pi.image.base"}}' \
      2>/dev/null || true)" == "${base_image}" && \
        "$(docker image inspect "${image}" --format \
      '{{index .Config.Labels "org.mentor-pi.image.source-sha256"}}' \
      2>/dev/null || true)" == "${source_sha}" && \
        "$(docker image inspect "${image}" --format '{{.Os}}/{{.Architecture}}' \
      2>/dev/null || true)" == "linux/${architecture}" ]]; then
    echo "Reusing verified ${component} image: ${image}"
    continue
  fi

  build_command=(
    docker build --platform "linux/${architecture}"
    --build-arg "BASE_IMAGE=${base_image}"
    --label "org.mentor-pi.image.base=${base_image}"
    --label "org.mentor-pi.image.source-sha256=${source_sha}"
    --file "${dockerfile}" --tag "${image}"
  )
  if [[ "${component}" == project ]]; then
    build_command+=(
      --build-arg "TARGETARCH=${architecture}"
      --build-arg "ROS_SNAPSHOT_DATE=${ROS_SNAPSHOT_DATE}"
      --build-arg "ROS2_CONTROL_VERSION=$(LockedValue ROS2_CONTROL_VERSION)"
      --build-arg "ROS2_CONTROLLERS_VERSION=$(LockedValue ROS2_CONTROLLERS_VERSION)"
      --build-arg "MECANUM_CONTROLLER_VERSION=$(LockedValue MECANUM_CONTROLLER_VERSION)"
      --build-arg "ACKERMANN_CONTROLLER_VERSION=$(LockedValue ACKERMANN_CONTROLLER_VERSION)"
      --build-arg "FOXGLOVE_BRIDGE_VERSION=$(LockedValue FOXGLOVE_BRIDGE_VERSION)"
      --build-arg "XACRO_VERSION=$(LockedValue XACRO_VERSION)"
    )
  fi
  use_host_network=0
  for proxy_variable in HTTP_PROXY HTTPS_PROXY NO_PROXY \
      http_proxy https_proxy no_proxy; do
    if [[ -n "${!proxy_variable:-}" ]]; then
      proxy_value="${!proxy_variable}"
      [[ "${proxy_value}" != *127.0.0.1* && \
         "${proxy_value}" != *localhost* ]] || use_host_network=1
      build_command+=(--build-arg "${proxy_variable}=${proxy_value}")
    fi
  done
  ((use_host_network == 0)) || build_command+=(--network host)
  build_command+=("${PROJECT_ROOT}")
  "${build_command[@]}"
  [[ "$(docker image inspect "${image}" --format '{{.Os}}/{{.Architecture}}')" == \
    "linux/${architecture}" ]] || Fail "built ${component} image has the wrong platform"
done

echo "Pinned RRCLite Docker images are ready for linux/${architecture}."
