#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_HELPER="${SCRIPT_DIR}/build_microros_agent_from_lock.sh"
readonly SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly XRCE_AGENT_PATCH="${SCRIPT_DIR}/patches/micro_xrce_agent_rrclite_modem_lines.patch"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly JOB_SELECTOR="${SCRIPT_DIR}/select_build_jobs.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly -a ORIGINAL_ARGUMENTS=("$@")

print_output=0

Fail() {
  echo "Adaptive micro-ROS Agent build error: $*" >&2
  exit 1
}

Sha256() {
  sha256sum "$1" | awk '{print $1}'
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
  echo "Usage: build_agent.sh [--print-output]" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --print-output) print_output=1; shift ;;
    -h | --help) Usage ;;
    *) Usage ;;
  esac
done

if ((print_output == 0)) && [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "${ORIGINAL_ARGUMENTS[@]}"
fi

[[ -r /etc/os-release ]] || Fail "/etc/os-release is unavailable"
[[ "$(ReadOsValue ID)" == "ubuntu" ]] || Fail "the host must be Ubuntu"
case "$(uname -m)" in
  x86_64 | amd64) architecture=amd64 ;;
  aarch64 | arm64) architecture=arm64 ;;
  *) Fail "the host architecture must be amd64 or arm64" ;;
esac
readonly architecture
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"
[[ -f "${XRCE_AGENT_PATCH}" && ! -L "${XRCE_AGENT_PATCH}" ]] || \
  Fail "RRCLite Agent patch is missing or symbolic"
readonly lock_sha="$(Sha256 "${SOURCE_LOCK}")"
readonly patch_sha="$(Sha256 "${XRCE_AGENT_PATCH}")"

command -v docker >/dev/null 2>&1 || Fail "Docker is required"
docker info >/dev/null 2>&1 || Fail "Docker is not running or accessible"
image="$(${IMAGE_SELECTOR} microros "${architecture}")"
image_id="$(docker image inspect "${image}" --format '{{.Id}}' \
  2>/dev/null || true)"
image_architecture="$(docker image inspect "${image}" \
  --format '{{.Architecture}}' 2>/dev/null || true)"
[[ "${image_id}" =~ ^sha256:[0-9a-f]{64}$ && \
  "${image_architecture}" == "${architecture}" ]] || \
  Fail "the pinned Humble Agent image is not local; run make setup"
readonly image image_id

cache_identity="${image_id#sha256:}"
cache_identity="${cache_identity//[^A-Za-z0-9._-]/-}"
readonly cache_key="agent-${architecture}-${lock_sha:0:12}-${patch_sha:0:12}-${cache_identity:0:16}"
readonly work_root="${PROJECT_ROOT}/build/runtime/${cache_key}"
readonly install_root="${work_root}/install"
readonly executable="${install_root}/lib/micro_ros_agent/micro_ros_agent"
readonly metadata="${install_root}/AGENT-BUILD-METADATA.txt"

if ((print_output == 1)); then
  printf '%s\n' "${install_root}"
  exit 0
fi

if [[ -f "${metadata}" && ! -L "${metadata}" ]]; then
  [[ "$(ReadSingleValue "${metadata}" architecture)" == "${architecture}" && \
    "$(ReadSingleValue "${metadata}" source_lock_sha256)" == "${lock_sha}" && \
    "$(ReadSingleValue "${metadata}" rrclite_patch_sha256)" == "${patch_sha}" && \
    "$(ReadSingleValue "${metadata}" builder_identity)" == "${image_id}" && \
    -x "${executable}" && ! -L "${executable}" && \
    "$(ReadSingleValue "${metadata}" executable_sha256)" == \
      "$(Sha256 "${executable}")" ]] || \
    Fail "cached Agent build metadata is inconsistent: ${work_root}"
  echo "Reusing verified pinned micro-ROS Agent: ${executable}"
  exit 0
fi
[[ ! -e "${metadata}" && ! -L "${metadata}" ]] || \
  Fail "Agent metadata path is not a regular file: ${metadata}"
mkdir -p "${work_root}" "${work_root}/home"

readonly caller_uid="$(id -u)"
readonly caller_gid="$(id -g)"
echo "Fetching pinned Agent sources in Docker."
docker run --rm \
  --platform "linux/${architecture}" \
  --network bridge \
  --user "${caller_uid}:${caller_gid}" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env HOME=/work/home \
  --volume "${PROJECT_ROOT}:/project:ro" \
  --volume "${work_root}:/work" \
  --entrypoint /bin/bash \
  "${image}" -lc \
  'source /opt/ros/humble/setup.bash && exec /project/tools/build_microros_agent_from_lock.sh fetch --work-root /work'

echo "Building the pinned Agent in Docker with networking disabled."
docker run --rm \
  --platform "linux/${architecture}" \
  --network none \
  --user "${caller_uid}:${caller_gid}" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env HOME=/work/home \
  --env "CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}" \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --volume "${PROJECT_ROOT}:/project:ro" \
  --volume "${work_root}:/work" \
  --entrypoint /bin/bash \
  "${image}" -lc \
  'source /opt/ros/humble/setup.bash && exec /project/tools/build_microros_agent_from_lock.sh build --work-root /work --dependency-mode preinstalled'

[[ -x "${executable}" && ! -L "${executable}" ]] || \
  Fail "Agent build did not produce its executable"
metadata_temporary="${metadata}.tmp.$$"
printf '%s\n' \
  'format=rrclite-adaptive-agent-v1' \
  "ubuntu_target=22.04" \
  'ros_distro=humble' \
  "architecture=${architecture}" \
  "source_lock_sha256=${lock_sha}" \
  "rrclite_patch_sha256=${patch_sha}" \
  "builder_identity=${image_id}" \
  "executable_sha256=$(Sha256 "${executable}")" \
  >"${metadata_temporary}"
mv "${metadata_temporary}" "${metadata}"
echo "Adaptive pinned Agent build ready: ${executable}"
