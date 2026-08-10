#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly BUILD_IMAGE_PREPARER="${SCRIPT_DIR}/prepare_build_images.sh"
readonly PROJECT_DOCKERFILE="${SCRIPT_DIR}/docker/rrclite.Dockerfile"
readonly IMAGE_FINGERPRINT="${SCRIPT_DIR}/docker_image_source_fingerprint.sh"
readonly PROJECT_ZSHRC="${SCRIPT_DIR}/docker/host-runtime.zshrc"
readonly ROS_LOCK="${SCRIPT_DIR}/docker/ros-humble-packages.lock"
readonly ALTO_LOCK="${SCRIPT_DIR}/altro_source.lock"
readonly AGENT_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly MICROROS_LOCK="${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_sources.lock"
readonly AGENT_BUILDER="${SCRIPT_DIR}/build_agent.sh"
readonly JOB_SELECTOR="${SCRIPT_DIR}/select_build_jobs.sh"
readonly OCI_EXPORTER="${SCRIPT_DIR}/export_oci_image_archive.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"

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
  "${BUILD_IMAGE_PREPARER}" --architecture "$3" --print project
  exit 0
fi

if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
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
case "$(uname -m)" in
  x86_64 | amd64) native_architecture=amd64 ;;
  aarch64 | arm64) native_architecture=arm64 ;;
  *) Fail "unsupported native architecture: $(uname -m)" ;;
esac
[[ "${architecture}" == "${native_architecture}" ]] || \
  Fail "cross-architecture handoff builds are forbidden; requested ${architecture} on ${native_architecture}"
[[ -x "${IMAGE_SELECTOR}" ]] || Fail "pinned image selector is unavailable"
[[ -x "${BUILD_IMAGE_PREPARER}" && -x "${IMAGE_FINGERPRINT}" && \
   -f "${PROJECT_DOCKERFILE}" && \
   -f "${PROJECT_ZSHRC}" && -f "${ROS_LOCK}" && -f "${ALTO_LOCK}" && \
   -f "${AGENT_LOCK}" && -f "${MICROROS_LOCK}" ]] || \
  Fail "unified Humble image tooling is unavailable"
readonly DEFAULT_RUNTIME_IMAGE="$(
  "${BUILD_IMAGE_PREPARER}" --architecture "${architecture}" --print project
)"
readonly PINNED_BASE_IMAGE="$("${IMAGE_SELECTOR}" microros "${architecture}")"
if [[ -z "${image}" ]]; then
  image="${MENTOR_PI_HOST_RUNTIME_IMAGE:-${DEFAULT_RUNTIME_IMAGE}}"
fi
[[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] ||
  Fail "release ID must contain 1-64 safe filename characters"
[[ -n "${output_directory}" ]] || Usage
command -v docker >/dev/null 2>&1 || Fail "docker is not installed"
[[ -x "${OCI_EXPORTER}" ]] || Fail "OCI image exporter is unavailable"
docker image inspect "${image}" >/dev/null 2>&1 ||
  Fail "prepared image is not local; run make setup"
readonly IMAGE_PLATFORM="$(docker image inspect "${image}" \
  --format '{{.Os}}/{{.Architecture}}')"
[[ "${IMAGE_PLATFORM}" == "linux/${architecture}" ]] || \
  Fail "prepared image platform ${IMAGE_PLATFORM} does not match linux/${architecture}"
builder_identity=""
if [[ "${image}" == "${DEFAULT_RUNTIME_IMAGE}" ]]; then
  readonly PROJECT_SOURCE_SHA="$(
    "${IMAGE_FINGERPRINT}" project "${PINNED_BASE_IMAGE}" "${PROJECT_ROOT}"
  )"
  [[ "$(docker image inspect "${image}" \
      --format '{{index .Config.Labels "org.mentor-pi.image.base"}}')" == \
      "${PINNED_BASE_IMAGE}" ]] || \
    Fail "prepared runtime image has the wrong pinned base"
  [[ "$(docker image inspect "${image}" \
      --format '{{index .Config.Labels "org.mentor-pi.image.source-sha256"}}')" == \
      "${PROJECT_SOURCE_SHA}" ]] || \
    Fail "prepared runtime image has the wrong unified source fingerprint"
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
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"
"${AGENT_BUILDER}"
readonly AGENT_PREFIX="$(${AGENT_BUILDER} --print-output)"
readonly AGENT_RELATIVE="${AGENT_PREFIX#"${PROJECT_ROOT}/"}"
[[ "${AGENT_RELATIVE}" != "${AGENT_PREFIX}" && \
   -x "${AGENT_PREFIX}/lib/micro_ros_agent/micro_ros_agent" ]] || \
  Fail "verified Agent prefix is unavailable"
readonly RUNTIME_IMAGE_ID="$(docker image inspect "${image}" --format '{{.Id}}')"
readonly IMAGE_ARCHIVE_RELATIVE="build/host-handoff-images/${release_id}-${architecture}.tar"
readonly IMAGE_ARCHIVE="${PROJECT_ROOT}/${IMAGE_ARCHIVE_RELATIVE}"
mkdir -p "$(dirname "${IMAGE_ARCHIVE}")"
[[ ! -e "${IMAGE_ARCHIVE}" && ! -L "${IMAGE_ARCHIVE}" ]] || \
  Fail "runtime image archive path already exists"
cleanup_archive=1
CleanupArchive() {
  if [[ "${cleanup_archive}" == 1 && -f "${IMAGE_ARCHIVE}" ]]; then
    rm -f -- "${IMAGE_ARCHIVE}"
  fi
}
trap CleanupArchive EXIT
"${OCI_EXPORTER}" "${image}" "${IMAGE_ARCHIVE}"

docker run --rm --network=none \
  --platform "${CONTAINER_PLATFORM}" \
  --env SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-0}" \
  --env MENTOR_PI_CALLER_UID="${CALLER_UID}" \
  --env MENTOR_PI_CALLER_GID="${CALLER_GID}" \
  --env MENTOR_PI_HOST_BUILDER_IMAGE="${builder_identity}" \
  --env MENTOR_PI_HOST_BUILDER_IMAGE_ID="${RUNTIME_IMAGE_ID}" \
  --env MENTOR_PI_OUTPUT_RELATIVE="${OUTPUT_RELATIVE}" \
  --env MENTOR_PI_PREFIX_RELATIVE="${PREFIX_RELATIVE}" \
  --env MENTOR_PI_BUILD_RELATIVE="${BUILD_RELATIVE}" \
  --env MENTOR_PI_RELEASE_ID="${release_id}" \
  --env MENTOR_PI_AGENT_RELATIVE="${AGENT_RELATIVE}" \
  --env MENTOR_PI_RUNTIME_IMAGE_ARCHIVE_RELATIVE="${IMAGE_ARCHIVE_RELATIVE}" \
  --env MENTOR_PI_RUNTIME_IMAGE_ID="${RUNTIME_IMAGE_ID}" \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${image}" \
  /workspace/tools/host_handoff_container_entrypoint.sh

rm -f -- "${IMAGE_ARCHIVE}"
cleanup_archive=0
trap - EXIT

echo "Host handoff completed without network or hardware access: ${OUTPUT_ROOT}"
