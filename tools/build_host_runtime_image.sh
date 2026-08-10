#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly DOCKERFILE="${SCRIPT_DIR}/docker/host-runtime.Dockerfile"
readonly ZSHRC="${SCRIPT_DIR}/docker/host-runtime.zshrc"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly -a ORIGINAL_ARGUMENTS=("$@")

architecture=""
print_output=0

Fail() {
  echo "Host runtime image build error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --architecture) architecture="${2:-}"; shift 2 ;;
    --print-output) print_output=1; shift ;;
    *) Fail "usage: build_host_runtime_image.sh --architecture amd64|arm64 [--print-output]" ;;
  esac
done
[[ "${architecture}" == "amd64" || "${architecture}" == "arm64" ]] || \
  Fail "architecture must be amd64 or arm64"

readonly base_image="$(${IMAGE_SELECTOR} host "${architecture}")"
readonly dockerfile_sha="$(sha256sum "${DOCKERFILE}" | awk '{print $1}')"
readonly zshrc_sha="$(sha256sum "${ZSHRC}" | awk '{print $1}')"
readonly runtime_source_sha="$(
  printf '%s\n%s\n' "${dockerfile_sha}" "${zshrc_sha}" | \
    sha256sum | awk '{print $1}'
)"
readonly image="mentor-pi/rrclite-host-runtime:humble-${architecture}-${runtime_source_sha:0:16}"
if ((print_output == 1)); then
  printf '%s\n' "${image}"
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
if [[ "$(docker image inspect "${image}" \
    --format '{{index .Config.Labels "org.mentor-pi.host-runtime.base"}}' \
    2>/dev/null || true)" == "${base_image}" && \
      "$(docker image inspect "${image}" \
    --format '{{index .Config.Labels "org.mentor-pi.host-runtime.dockerfile-sha256"}}' \
    2>/dev/null || true)" == "${dockerfile_sha}" && \
      "$(docker image inspect "${image}" \
    --format '{{index .Config.Labels "org.mentor-pi.host-runtime.zshrc-sha256"}}' \
    2>/dev/null || true)" == "${zshrc_sha}" && \
      "$(docker image inspect "${image}" \
    --format '{{.Os}}/{{.Architecture}}' 2>/dev/null || true)" == \
      "linux/${architecture}" ]]; then
  echo "Reusing verified Mentor Pi Humble host runtime image: ${image}"
  exit 0
fi

declare -a build_command=(
  docker build --platform "linux/${architecture}"
  --build-arg "BASE_IMAGE=${base_image}"
  --label "org.mentor-pi.host-runtime.base=${base_image}"
  --label "org.mentor-pi.host-runtime.dockerfile-sha256=${dockerfile_sha}"
  --label "org.mentor-pi.host-runtime.zshrc-sha256=${zshrc_sha}"
  --file "${DOCKERFILE}" --tag "${image}"
)
use_host_network=0
for proxy_variable in HTTP_PROXY HTTPS_PROXY NO_PROXY \
    http_proxy https_proxy no_proxy; do
  if [[ -n "${!proxy_variable:-}" ]]; then
    proxy_value="${!proxy_variable}"
    if [[ "${proxy_value}" == *127.0.0.1* ||
          "${proxy_value}" == *localhost* ]]; then
      use_host_network=1
    fi
    build_command+=(--build-arg "${proxy_variable}=${proxy_value}")
  fi
done
if ((use_host_network == 1)); then
  build_command+=(--network host)
fi
build_command+=("${PROJECT_ROOT}")
"${build_command[@]}"

[[ "$(docker image inspect "${image}" --format '{{.Os}}/{{.Architecture}}')" == \
  "linux/${architecture}" ]] || Fail "built image platform does not match"
echo "Built Mentor Pi Humble host runtime image: ${image}"
