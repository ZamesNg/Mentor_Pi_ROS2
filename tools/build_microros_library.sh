#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FIRMWARE_ROOT="${PROJECT_ROOT}/firmware/mentor_pi_mcu"
readonly INTERFACE_ROOT="${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces"
readonly SOURCE_UTILS="${FIRMWARE_ROOT}/third_party/micro_ros_stm32cubemx_utils"
readonly SOURCE_UTILS_REPOSITORY="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git"
readonly SOURCE_UTILS_COMMIT="bd531b273c1bcd070b3143c5642128ec75a6f04e"
readonly BUILD_ROOT="${FIRMWARE_ROOT}/build/microros"
readonly BUILD_UTILS="${BUILD_ROOT}/micro_ros_stm32cubemx_utils"
readonly CAPTURE_IMAGE="microros/micro_ros_static_library_builder:humble@sha256:e291f74890e81b31eb1d70731cb79b2d767dd585269325031effc72952b24b9d"
readonly DOCKERFILE="${PROJECT_ROOT}/tools/docker/microros-builder.Dockerfile"
readonly SOURCE_LOCK="${FIRMWARE_ROOT}/config/microros_sources.lock"
readonly ARTIFACT_HASH="${FIRMWARE_ROOT}/config/microros_artifact.sha256"
readonly ARTIFACT_TREE_HASH="${FIRMWARE_ROOT}/config/microros_artifact_tree.sha256"
readonly FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/firmware_source_fingerprint.sh"
readonly MICROROS_FINGERPRINT_TOOL="${PROJECT_ROOT}/tools/microros_artifact_fingerprint.sh"
readonly DEPENDENCY_BOOTSTRAP="${PROJECT_ROOT}/tools/bootstrap_firmware_dependencies.sh"
readonly BUILD_IMAGE_PREPARER="${PROJECT_ROOT}/tools/prepare_build_images.sh"
readonly JOB_SELECTOR="${PROJECT_ROOT}/tools/select_build_jobs.sh"
readonly CACHE_VALIDATOR="${PROJECT_ROOT}/tools/verify_microros_cache.sh"
readonly GEOMETRY2_COMMIT="65c620c308920f558c7b5d3fb852941bd6d8fced"
readonly LIBYAML_REPOSITORY="https://github.com/yaml/libyaml"
readonly LIBYAML_COMMIT="2c891fc7a770e8ba2fec34fc6b545c672beb37e6"
readonly SOURCE_LOCK_CANDIDATE="${FIRMWARE_ROOT}/build/microros_sources.humble.candidate.lock"
readonly ARTIFACT_HASH_CANDIDATE="${FIRMWARE_ROOT}/build/microros_artifact.humble.candidate.sha256"
readonly ARTIFACT_TREE_HASH_CANDIDATE="${FIRMWARE_ROOT}/build/microros_artifact_tree.humble.candidate.sha256"
readonly CACHE_METADATA="${BUILD_ROOT}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/MENTOR-PI-CACHE.txt"
readonly BUILD_LOCK="${PROJECT_ROOT}/tools/run_with_build_lock.sh"

if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi

# Docker does not inherit shell proxy variables into builds or containers.
# Forward only the standard proxy variables when the caller explicitly sets
# them; a normal direct-network build adds no proxy arguments.
declare -a docker_run_command=(docker run --rm)
for proxy_variable in HTTP_PROXY HTTPS_PROXY NO_PROXY \
    http_proxy https_proxy no_proxy; do
  if [[ -n "${!proxy_variable:-}" ]]; then
    if [[ "${proxy_variable}" != "NO_PROXY" &&
          "${proxy_variable}" != "no_proxy" ]]; then
      case "${!proxy_variable}" in
        *://127.0.0.1:*|*://localhost:*|*://\[::1\]:*)
          echo "Not forwarding host-loopback ${proxy_variable} into Docker."
          continue
          ;;
      esac
    fi
    docker_run_command+=(
      --env "${proxy_variable}=${!proxy_variable}"
    )
  fi
done
readonly -a docker_run_command

capture_source_lock=0
capture_artifact_hashes=0
if [[ "$#" -eq 1 && "$1" == "--capture-source-lock" ]]; then
  capture_source_lock=1
elif [[ "$#" -eq 1 && "$1" == "--capture-artifact-hashes" ]]; then
  capture_artifact_hashes=1
elif [[ "$#" -ne 0 ]]; then
  echo "Usage: $0 [--capture-source-lock|--capture-artifact-hashes]" >&2
  exit 2
fi

Sha256() {
  if command -v sha256sum >/dev/null; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "Neither sha256sum nor shasum is installed." >&2
    return 1
  fi
}

GenerationInputFingerprint() {
  local image_identity="$1"
  {
    printf 'image=%s\n' "${image_identity}"
    for input in \
        "${SOURCE_LOCK}" \
        "${FIRMWARE_ROOT}/config/microros_colcon.meta" \
        "${FIRMWARE_ROOT}/config/microros_toolchain.cmake" \
        "${FIRMWARE_ROOT}/config/microros_library_generation.sh" \
        "${DOCKERFILE}" \
        "${PROJECT_ROOT}/tools/apply_microros_source_lock.sh" \
        "${PROJECT_ROOT}/tools/build_microros_library.sh" \
        "${CACHE_VALIDATOR}"; do
      printf '%s  %s\n' "$(Sha256 "${input}")" "${input#"${PROJECT_ROOT}/"}"
    done
  } | sha256sum | awk '{print $1}'
}

RemoveBuildRoot() {
  [[ "${BUILD_ROOT}" == "${FIRMWARE_ROOT}/build/microros" &&
     "${BUILD_ROOT}" != "/" ]] || {
    echo "Refusing to remove unexpected micro-ROS build root: ${BUILD_ROOT}" >&2
    return 1
  }
  rm -rf -- "${BUILD_ROOT}"
}

if [[ ! -d "${SOURCE_UTILS}" ]]; then
  echo "Run tools/bootstrap_firmware_dependencies.sh first." >&2
  exit 1
fi
if [[ ! -f "${INTERFACE_ROOT}/package.xml" ]]; then
  echo "mentor_pi_interfaces is missing: ${INTERFACE_ROOT}" >&2
  exit 1
fi
if [[ ! -f "${SOURCE_LOCK}" || \
    ("${capture_source_lock}" == "0" && \
     "${capture_artifact_hashes}" == "0" && \
     (! -f "${ARTIFACT_HASH}" || ! -f "${ARTIFACT_TREE_HASH}")) ]]; then
  echo "micro-ROS source/artifact lock is missing under config/." >&2
  exit 1
fi
if [[ ! -x "${FINGERPRINT_TOOL}" || ! -x "${CACHE_VALIDATOR}" || \
    ! -x "${MICROROS_FINGERPRINT_TOOL}" || \
    ! -x "${DEPENDENCY_BOOTSTRAP}" || \
    ! -x "${BUILD_IMAGE_PREPARER}" || \
    ! -x "${JOB_SELECTOR}" ]]; then
  echo "Firmware artifact fingerprint tooling is missing or not executable." >&2
  exit 1
fi
"${DEPENDENCY_BOOTSTRAP}" --verify-existing \
  "${SOURCE_UTILS_REPOSITORY}" "${SOURCE_UTILS_COMMIT}" "${SOURCE_UTILS}"
readonly INTERFACE_FINGERPRINT_BEFORE="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"

case "$(uname -m)" in
  x86_64 | amd64) readonly ARCHITECTURE=amd64 ;;
  aarch64 | arm64) readonly ARCHITECTURE=arm64 ;;
  *) echo "Unsupported host architecture: $(uname -m)" >&2; exit 1 ;;
esac
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"
command -v docker >/dev/null 2>&1 || {
  echo "Docker is required for micro-ROS generation." >&2
  exit 1
}
docker info >/dev/null 2>&1 || {
  echo "Docker Engine is not running or accessible." >&2
  exit 1
}

selected_image=""
if [[ "${capture_source_lock}" == "1" ]]; then
  selected_image="${CAPTURE_IMAGE}"
  if ! docker image inspect "${selected_image}" >/dev/null 2>&1; then
    docker pull --platform "linux/${ARCHITECTURE}" "${selected_image}"
  fi
else
  "${BUILD_IMAGE_PREPARER}" --architecture "${ARCHITECTURE}"
  selected_image="$("${BUILD_IMAGE_PREPARER}" \
    --architecture "${ARCHITECTURE}" --print microros)"
fi
readonly selected_image
readonly IMAGE_ID="$(docker image inspect "${selected_image}" --format '{{.Id}}')"
readonly IMAGE_ARCHITECTURE="$(docker image inspect "${selected_image}" --format '{{.Architecture}}')"
[[ "${IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ && \
   "${IMAGE_ARCHITECTURE}" == "${ARCHITECTURE}" ]] || {
  echo "The micro-ROS image identity or architecture is invalid." >&2
  exit 1
}
readonly GENERATION_INPUT_SHA="$(GenerationInputFingerprint "${IMAGE_ID}")"

if [[ "${capture_source_lock}" == 0 && "${capture_artifact_hashes}" == 0 && \
      -e "${CACHE_METADATA}" ]]; then
  cached_tree="$("${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}" 2>/dev/null || true)"
  if "${CACHE_VALIDATOR}" \
      --metadata "${CACHE_METADATA}" \
      --image-id "${IMAGE_ID}" \
      --architecture "${ARCHITECTURE}" \
      --generation-input-sha256 "${GENERATION_INPUT_SHA}" \
      --interfaces-sha256 "${INTERFACE_FINGERPRINT_BEFORE}" \
      --archive "${CACHE_METADATA%/*}/libmicroros.a" \
      --expected-archive-sha256 "$(tr -d '[:space:]' <"${ARTIFACT_HASH}")" \
      --tree-sha256 "${cached_tree}" \
      --expected-tree-sha256 "$(tr -d '[:space:]' <"${ARTIFACT_TREE_HASH}")" \
      >/dev/null; then
    echo "Reusing verified Humble micro-ROS library cache: ${CACHE_METADATA%/*}"
    exit 0
  fi
  echo "Discarding stale or corrupt micro-ROS cache."
fi

RemoveBuildRoot
mkdir -p "${BUILD_UTILS}"
cp -a "${SOURCE_UTILS}/." "${BUILD_UTILS}/"
# The dependency checkout is provenance-verified immediately above. Its Git
# object database is not an input to static-library generation, and copying it
# into a bind-mounted build tree prevents Docker Desktop from restoring file
# ownership. Keep only the reviewed checkout contents in the build tree.
rm -rf -- "${BUILD_UTILS}/.git"
if [[ -n "$(find "${BUILD_ROOT}" -type d -name .git -print -quit)" ]]; then
  echo "Unexpected nested Git metadata in the micro-ROS build inputs." >&2
  exit 1
fi

# The upstream generator resolves custom packages from BASE_PATH/../../, where
# BASE_PATH is BUILD_UTILS/microros_static_library_ide.  Therefore the
# microros_component directory must be a sibling of BUILD_UTILS, not a child.
readonly EXTRA_PACKAGES="${BUILD_ROOT}/microros_component/extra_packages"
mkdir -p "${EXTRA_PACKAGES}/mentor_pi_interfaces"
cp -a "${INTERFACE_ROOT}/." \
  "${EXTRA_PACKAGES}/mentor_pi_interfaces/"
cp \
  "${FIRMWARE_ROOT}/config/microros_colcon.meta" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/colcon.meta"
cp \
  "${FIRMWARE_ROOT}/config/microros_toolchain.cmake" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/toolchain.cmake"
cp \
  "${FIRMWARE_ROOT}/config/microros_library_generation.sh" \
  "${BUILD_UTILS}/microros_static_library_ide/library_generation/library_generation.sh"

# Select the sole MCU type-support backend at generation time so desktop
# backends are not retained in the embedded archive.
if [[ "${capture_source_lock}" == "1" ]]; then
  rm -f -- "${SOURCE_LOCK_CANDIDATE}"
fi
if [[ "${capture_artifact_hashes}" == "1" ]]; then
  rm -f -- "${ARTIFACT_HASH_CANDIDATE}" \
    "${ARTIFACT_TREE_HASH_CANDIDATE}"
fi

cat >"${BUILD_ROOT}/colcon-defaults.yaml" <<EOF
build:
  executor: parallel
  parallel-workers: ${BUILD_JOBS}
EOF
"${docker_run_command[@]}" \
  --platform "linux/${ARCHITECTURE}" \
  --volume "${FIRMWARE_ROOT}:/project" \
  --volume "${PROJECT_ROOT}/tools:/rrclite_tools:ro" \
  --env COLCON_DEFAULTS_FILE=/project/build/microros/colcon-defaults.yaml \
  --env CMAKE_BUILD_PARALLEL_LEVEL=1 \
  --env MAKEFLAGS=-j1 \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --env MICROROS_LIBRARY_FOLDER=build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide \
  --env STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_microxrcedds_c \
  --env MICROROS_GEOMETRY2_COMMIT="${GEOMETRY2_COMMIT}" \
  --env MICROROS_LIBYAML_REPOSITORY="${LIBYAML_REPOSITORY}" \
  --env MICROROS_LIBYAML_COMMIT="${LIBYAML_COMMIT}" \
  --env MICROROS_CAPTURE_SOURCE_LOCK="${capture_source_lock}" \
  --env MICROROS_SOURCE_LOCK_CANDIDATE=build/microros_sources.humble.candidate.lock \
  --env MICROROS_CALLER_UID="$(id -u)" \
  --env MICROROS_CALLER_GID="$(id -g)" \
  --env PYTHONHASHSEED=0 \
  "${selected_image}"

if [[ "${capture_source_lock}" == "1" ]]; then
  test -s "${SOURCE_LOCK_CANDIDATE}"
  echo "Captured candidate Humble micro-ROS source lock: ${SOURCE_LOCK_CANDIDATE}"
  echo "Review and apply it to ${SOURCE_LOCK}, then run this command without arguments."
  exit 0
fi

readonly LIBRARY_DIR="${BUILD_UTILS}/microros_static_library_ide/libmicroros"
test -f "${LIBRARY_DIR}/libmicroros.a"
test -d "${LIBRARY_DIR}/include"
grep -Fqx 'humble' "${LIBRARY_DIR}/ros_distro"
test -f \
  "${LIBRARY_DIR}/include/mentor_pi_interfaces/msg/motor_command.h"
test -f \
  "${LIBRARY_DIR}/include/mentor_pi_interfaces/srv/set_motor_model.h"
test -f \
  "${LIBRARY_DIR}/include/mentor_pi_interfaces/srv/set_motor_pid.h"
grep -q '^mentor_pi_interfaces/MotorCommand.msg$' \
  "${LIBRARY_DIR}/available_ros2_types"
grep -q '^mentor_pi_interfaces/SetMotorModel.srv$' \
  "${LIBRARY_DIR}/available_ros2_types"
grep -q '^mentor_pi_interfaces/SetMotorPid.srv$' \
  "${LIBRARY_DIR}/available_ros2_types"
readonly NORMALIZED_EXPECTED="$(mktemp)"
readonly NORMALIZED_ACTUAL="$(mktemp)"
trap 'rm -f "${NORMALIZED_EXPECTED}" "${NORMALIZED_ACTUAL}"' EXIT
sed '/^[[:space:]]*#/d; /^[[:space:]]*$/d' "${SOURCE_LOCK}" | sort \
  >"${NORMALIZED_EXPECTED}"
while read -r repository_url repository_commit extra; do
  [[ -n "${repository_url}" && -n "${repository_commit}" && \
      -z "${extra:-}" ]] || {
    echo "Invalid generated built_packages row." >&2
    exit 1
  }
  repository_url="${repository_url%/}"
  repository_url="${repository_url%.git}"
  printf '%s %s\n' "${repository_url}" "${repository_commit}"
done <"${LIBRARY_DIR}/built_packages" | sort >"${NORMALIZED_ACTUAL}"
if ! diff -u "${NORMALIZED_EXPECTED}" "${NORMALIZED_ACTUAL}"; then
  echo "Generated micro-ROS package revisions differ from the lock." >&2
  exit 1
fi
readonly ACTUAL_ARCHIVE_HASH="$(Sha256 "${LIBRARY_DIR}/libmicroros.a")"
readonly ACTUAL_TREE_HASH="$(
  "${MICROROS_FINGERPRINT_TOOL}" "${PROJECT_ROOT}"
)"
readonly INTERFACE_FINGERPRINT_AFTER="$(
  "${FINGERPRINT_TOOL}" interfaces "${PROJECT_ROOT}"
)"
if [[ "${INTERFACE_FINGERPRINT_BEFORE}" != \
    "${INTERFACE_FINGERPRINT_AFTER}" ]]; then
  echo "mentor_pi_interfaces changed while the library was generated." >&2
  exit 1
fi
printf '%s\n' "${INTERFACE_FINGERPRINT_AFTER}" \
  >"${LIBRARY_DIR}/mentor_pi_interfaces.source.sha256"

if [[ "${capture_artifact_hashes}" == "1" ]]; then
  printf '%s\n' "${ACTUAL_ARCHIVE_HASH}" >"${ARTIFACT_HASH_CANDIDATE}"
  printf '%s\n' "${ACTUAL_TREE_HASH}" >"${ARTIFACT_TREE_HASH_CANDIDATE}"
  echo "Captured candidate Humble micro-ROS artifact hashes:"
  echo "  archive=${ACTUAL_ARCHIVE_HASH}"
  echo "  tree=${ACTUAL_TREE_HASH}"
  echo "Review and apply the candidate files under ${FIRMWARE_ROOT}/build."
  exit 0
fi

readonly EXPECTED_ARCHIVE_HASH="$(tr -d '[:space:]' <"${ARTIFACT_HASH}")"
if [[ "${ACTUAL_ARCHIVE_HASH}" != "${EXPECTED_ARCHIVE_HASH}" ]]; then
  echo "Generated libmicroros.a hash differs from the reviewed artifact." >&2
  echo "Expected: ${EXPECTED_ARCHIVE_HASH}" >&2
  echo "Actual:   ${ACTUAL_ARCHIVE_HASH}" >&2
  exit 1
fi
readonly EXPECTED_TREE_HASH="$(tr -d '[:space:]' \
  <"${ARTIFACT_TREE_HASH}")"
if [[ "${ACTUAL_TREE_HASH}" != "${EXPECTED_TREE_HASH}" ]]; then
  echo "Generated micro-ROS archive/header tree differs from the reviewed artifact." >&2
  echo "Expected: ${EXPECTED_TREE_HASH}" >&2
  echo "Actual:   ${ACTUAL_TREE_HASH}" >&2
  exit 1
fi
cat >"${CACHE_METADATA}.tmp" <<EOF
format=mentor-pi-microros-cache-v1
image_id=${IMAGE_ID}
architecture=${ARCHITECTURE}
generation_input_sha256=${GENERATION_INPUT_SHA}
interfaces_sha256=${INTERFACE_FINGERPRINT_AFTER}
archive_sha256=${ACTUAL_ARCHIVE_HASH}
tree_sha256=${ACTUAL_TREE_HASH}
EOF
mv "${CACHE_METADATA}.tmp" "${CACHE_METADATA}"
echo "Generated Humble micro-ROS library: ${LIBRARY_DIR}"
