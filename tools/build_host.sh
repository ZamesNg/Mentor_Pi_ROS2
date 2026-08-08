#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_TOOL="${SCRIPT_DIR}/build_host_release.sh"
readonly CONTAINER_ENTRYPOINT="${SCRIPT_DIR}/host_build_container_entrypoint.sh"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly HOST_RUNTIME_BUILDER="${SCRIPT_DIR}/build_host_runtime_image.sh"

print_output=0
runtime_build=0

Fail() {
  echo "Adaptive host build error: $*" >&2
  exit 1
}

ReadSingleValue() {
  local metadata_file="$1"
  local key="$2"
  local count=""
  local line=""
  count="$(grep -Ec "^${key}=" "${metadata_file}" || true)"
  [[ "${count}" == "1" ]] || \
    Fail "${metadata_file} must contain exactly one ${key}= entry"
  line="$(grep -E "^${key}=" "${metadata_file}")"
  printf '%s' "${line#*=}"
}

ReadOsValue() {
  local key="$1"
  local line=""
  local value=""
  line="$(grep -E "^${key}=" /etc/os-release || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] || \
    Fail "/etc/os-release must contain exactly one ${key}= entry"
  value="${line#*=}"
  value="${value#\"}"
  value="${value%\"}"
  printf '%s' "${value}"
}

Usage() {
  echo "Usage: build_host.sh [--runtime] [--print-output]" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --print-output) print_output=1; shift ;;
    --runtime) runtime_build=1; shift ;;
    -h | --help) Usage ;;
    *) Usage ;;
  esac
done

[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
[[ "$(ReadOsValue ID)" == "ubuntu" ]] || Fail "the host must be Ubuntu"
readonly ubuntu_version="$(ReadOsValue VERSION_ID)"
case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
  *) Fail "the host architecture must be amd64 or arm64" ;;
esac
readonly architecture

readonly source_sha="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
if ((runtime_build == 1)); then
  readonly build_key="${architecture}-${source_sha:0:16}-runtime"
else
  readonly build_key="${architecture}-${source_sha:0:16}"
fi
readonly runtime_root="${PROJECT_ROOT}/build/runtime/${build_key}"
readonly output_prefix="${runtime_root}/host"
readonly work_directory="${runtime_root}/host-work"
readonly metadata="${output_prefix}/HOST-BUILD-METADATA.txt"

if ((print_output == 1)); then
  printf '%s\n' "${output_prefix}"
  exit 0
fi

if [[ -f "${metadata}" && ! -L "${metadata}" ]]; then
  [[ "$(ReadSingleValue "${metadata}" source_sha256)" == "${source_sha}" && \
    "$(ReadSingleValue "${metadata}" architecture)" == "${architecture}" && \
    "$(ReadSingleValue "${metadata}" ubuntu)" == "22.04" && \
    "$(ReadSingleValue "${metadata}" ros_distro)" == "humble" && \
    -r "${output_prefix}/setup.bash" ]] || \
    Fail "cached host build metadata is inconsistent: ${output_prefix}"
  echo "Reusing verified Humble host build: ${output_prefix}"
  exit 0
fi
[[ ! -e "${output_prefix}" && ! -L "${output_prefix}" && \
  ! -e "${work_directory}" && ! -L "${work_directory}" ]] || \
  Fail "incomplete build cache exists at ${runtime_root}; move that exact generated directory aside and retry"

if [[ "${ubuntu_version}" == "22.04" ]]; then
  echo "Building the host natively on Ubuntu 22.04/ROS 2 Humble."
  build_arguments=(
    --project-root "${PROJECT_ROOT}" \
    --output-prefix "${output_prefix}" \
    --work-directory "${work_directory}"
  )
  ((runtime_build == 0)) || build_arguments+=(--skip-tests)
  exec "${BUILD_TOOL}" "${build_arguments[@]}"
fi

command -v docker >/dev/null 2>&1 || \
  Fail "Docker is required on Ubuntu ${ubuntu_version}"
docker info >/dev/null 2>&1 || \
  Fail "Docker is not running or accessible"
"${HOST_RUNTIME_BUILDER}" --architecture "${architecture}"
readonly image="$(${HOST_RUNTIME_BUILDER} --architecture "${architecture}" --print-output)"
readonly base_image="$(${IMAGE_SELECTOR} host "${architecture}")"
readonly image_architecture="$(docker image inspect "${image}" \
  --format '{{.Architecture}}' 2>/dev/null || true)"
[[ "${image_architecture}" == "${architecture}" ]] || \
  Fail "the pinned Humble host image is not local; run make setup"

readonly output_relative="${output_prefix#"${PROJECT_ROOT}/"}"
readonly work_relative="${work_directory#"${PROJECT_ROOT}/"}"
[[ "${output_relative}" != "${output_prefix}" && \
  "${work_relative}" != "${work_directory}" ]] || \
  Fail "generated runtime paths must remain inside the repository"

echo "Building the host in pinned Ubuntu 22.04/ROS 2 Humble Docker on Ubuntu ${ubuntu_version}."
docker run --rm --network=none \
  --platform "linux/${architecture}" \
  --env MENTOR_PI_CALLER_UID="$(id -u)" \
  --env MENTOR_PI_CALLER_GID="$(id -g)" \
  --env MENTOR_PI_HOST_BUILDER_IMAGE="${base_image}" \
  --env MENTOR_PI_OUTPUT_RELATIVE="${output_relative}" \
  --env MENTOR_PI_WORK_RELATIVE="${work_relative}" \
  --env MENTOR_PI_SKIP_TESTS="${runtime_build}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  --entrypoint /workspace/tools/host_build_container_entrypoint.sh \
  "${image}"

[[ -f "${metadata}" && \
  "$(ReadSingleValue "${metadata}" source_sha256)" == "${source_sha}" ]] || \
  Fail "container host build did not produce matching metadata"
echo "Adaptive Humble host build ready: ${output_prefix}"
