#!/usr/bin/env bash

# The only approved cross-architecture path. Do not add an architecture flag:
# ordinary commands must stay native, and this target is fixed amd64 -> arm64.
set -euo pipefail

if [[ "${RRCLITE_RDK_RUNNER_SNAPSHOT:-0}" != 1 ]]; then
  initial_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  initial_project_root="$(cd "${initial_script_dir}/.." && pwd)"
  initial_runner_root="${initial_project_root}/build/rdk-handoff-runner"
  mkdir -p "${initial_runner_root}"
  runner_snapshot="$(mktemp "${initial_runner_root}/rdk-handoff.XXXXXX.sh")"
  cp -- "${BASH_SOURCE[0]}" "${runner_snapshot}"
  chmod 0755 "${runner_snapshot}"
  exec "${initial_script_dir}/run_with_build_lock.sh" env \
    RRCLITE_RDK_RUNNER_SNAPSHOT=1 \
    RRCLITE_RDK_PROJECT_ROOT="${initial_project_root}" \
    RRCLITE_RDK_RUNNER_PATH="${runner_snapshot}" \
    "${runner_snapshot}" "$@"
fi

readonly PROJECT_ROOT="${RRCLITE_RDK_PROJECT_ROOT:?missing snapshotted project root}"
readonly SCRIPT_DIR="${PROJECT_ROOT}/tools"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly JOB_SELECTOR="${SCRIPT_DIR}/select_rdk_handoff_jobs.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
readonly FIRMWARE_FINGERPRINT="${SCRIPT_DIR}/firmware_source_fingerprint.sh"
readonly HOST_FINGERPRINT="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly IMAGE_FINGERPRINT="${SCRIPT_DIR}/docker_image_source_fingerprint.sh"
readonly OCI_ARCHIVE_VERIFIER="${SCRIPT_DIR}/verify_oci_image_archive.py"
readonly RUNNER_SNAPSHOT="${RRCLITE_RDK_RUNNER_PATH:?missing runner snapshot path}"
readonly -a ORIGINAL_ARGUMENTS=("$@")

CleanupRunnerSnapshot() {
  case "${RUNNER_SNAPSHOT}" in
    "${PROJECT_ROOT}/build/rdk-handoff-runner/"rdk-handoff.*.sh)
      [[ ! -f "${RUNNER_SNAPSHOT}" || -L "${RUNNER_SNAPSHOT}" ]] || \
        rm -f -- "${RUNNER_SNAPSHOT}"
      ;;
  esac
}
trap CleanupRunnerSnapshot EXIT

Fail() { echo "RDK handoff error: $*" >&2; exit 1; }
Sha256() { sha256sum "$1" | awk '{print $1}'; }
RequireLine() {
  local file="$1"
  local expected="$2"
  [[ -f "${file}" && ! -L "${file}" ]] || Fail "required metadata is missing: ${file}"
  grep -Fqx "${expected}" "${file}" || \
    Fail "required metadata '${expected}' is missing from ${file}"
}
RequireUniqueField() {
  local file="$1"
  local key="$2"
  local expected="$3"
  [[ "$(awk -F= -v key="${key}" '$1 == key {count++} END {print count + 0}' \
    "${file}")" == 1 ]] || Fail "metadata field ${key} is missing or duplicated in ${file}"
  RequireLine "${file}" "${key}=${expected}"
}
RelativeInputsFingerprint() {
  local root="$1"
  shift
  {
    local relative
    for relative in "$@"; do
      [[ -f "${root}/${relative}" && ! -L "${root}/${relative}" ]] || \
        Fail "stage input is missing or symbolic: ${root}/${relative}"
      printf '%s  %s\n' "$(Sha256 "${root}/${relative}")" "${relative}"
    done
  } | sha256sum | awk '{print $1}'
}
CombineFingerprint() {
  printf '%s\n' "$@" | sha256sum | awk '{print $1}'
}

[[ "$#" == 0 ]] || Fail "make rdk-handoff accepts no arguments"
[[ "${RRCLITE_BUILD_LOCK_HELD:-0}" == 1 ]] || Fail "runner snapshot does not own the build lock"
[[ "$(Sha256 "${RUNNER_SNAPSHOT}")" == "$(Sha256 "${SCRIPT_DIR}/rdk_handoff.sh")" ]] || \
  Fail "handoff runner changed while its immutable snapshot was starting; retry"
[[ "${RDK_HANDOFF_FRESH:-0}" == 0 || "${RDK_HANDOFF_FRESH:-0}" == 1 ]] || \
  Fail "RDK_HANDOFF_FRESH must be 0 or 1"
case "$(uname -m)" in
  x86_64|amd64) ;; *) Fail "make rdk-handoff is supported only on an amd64 host" ;; esac
command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker is not running or accessible"
command -v rsync >/dev/null 2>&1 || Fail "rsync is required to isolate the arm64 build"
command -v sha256sum >/dev/null 2>&1 || Fail "sha256sum is required"
command -v python3 >/dev/null 2>&1 || Fail "Python 3 is required"
[[ -x "${OCI_ARCHIVE_VERIFIER}" ]] || Fail "OCI archive verifier is unavailable"

# This executes an arm64 ELF before any build. Docker reports a missing binfmt
# handler instead of silently executing an amd64 image when emulation is absent.
readonly ARM64_BASE_IMAGE="$("${IMAGE_SELECTOR}" microros arm64)"
if ! docker run --rm --platform linux/arm64 --entrypoint /bin/true \
    "${ARM64_BASE_IMAGE}"; then
  Fail "linux/arm64 QEMU/binfmt preflight failed; install/register binfmt explicitly, then retry (this command never registers it)"
fi

readonly CPU_COUNT="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
[[ "${CPU_COUNT}" =~ ^[1-9][0-9]*$ ]] || Fail "online CPU count is unavailable"
SELECTED_BUILD_JOBS="$(RRCLITE_CPU_COUNT="${CPU_COUNT}" "${JOB_SELECTOR}")"
readonly SELECTED_BUILD_JOBS
export RRCLITE_BUILD_JOBS="${SELECTED_BUILD_JOBS}"
export RRCLITE_QEMU_EMULATED_TESTS=1
echo "RDK handoff QEMU package workers: ${RRCLITE_BUILD_JOBS}"

readonly PROJECT_IMAGE_SOURCE_SHA="$(
  "${IMAGE_FINGERPRINT}" project "${ARM64_BASE_IMAGE}" "${PROJECT_ROOT}"
)"
readonly FIRMWARE_INPUT_SHA="$("${FIRMWARE_FINGERPRINT}" firmware "${PROJECT_ROOT}")"
readonly HOST_INPUT_SHA="$("${HOST_FINGERPRINT}" --package-content \
  "${PROJECT_ROOT}")"
readonly HOST_PACKAGE_PAYLOAD_SHA="$("${HOST_FINGERPRINT}" --package-payload \
  "${PROJECT_ROOT}")"
readonly HOST_BUILD_INPUT_SHA="$("${HOST_FINGERPRINT}" --compile "${PROJECT_ROOT}")"
readonly MICROROS_STAGE_SOURCE_SHA="$(RelativeInputsFingerprint "${PROJECT_ROOT}" \
  firmware/mentor_pi_mcu/config/microros_sources.lock \
  firmware/mentor_pi_mcu/config/microros_colcon.meta \
  firmware/mentor_pi_mcu/config/microros_toolchain.cmake \
  firmware/mentor_pi_mcu/config/microros_library_generation.sh \
  tools/apply_microros_source_lock.sh tools/build_microros_library.sh \
  tools/microros_artifact_fingerprint.sh tools/verify_microros_cache.sh)"
readonly AGENT_STAGE_SOURCE_SHA="$(RelativeInputsFingerprint "${PROJECT_ROOT}" \
  tools/build_agent.sh tools/build_microros_agent_from_lock.sh \
  tools/microros_agent_source.lock \
  tools/patches/micro_xrce_agent_rrclite_modem_lines.patch)"
readonly IMAGE_STAGE_INPUT_SHA="$(CombineFingerprint image \
  "${PROJECT_IMAGE_SOURCE_SHA}" "${ARM64_BASE_IMAGE}")"
readonly MICROROS_STAGE_INPUT_SHA="$(CombineFingerprint microros \
  "${IMAGE_STAGE_INPUT_SHA}" \
  "$("${FIRMWARE_FINGERPRINT}" interfaces "${PROJECT_ROOT}")" \
  "${MICROROS_STAGE_SOURCE_SHA}")"
readonly PID_STAGE_INPUT_SHA="$(CombineFingerprint pid "${IMAGE_STAGE_INPUT_SHA}" \
  "${FIRMWARE_INPUT_SHA}" "$(Sha256 "${SCRIPT_DIR}/package_board_handoff.sh")")"
readonly AGENT_STAGE_INPUT_SHA="$(CombineFingerprint agent \
  "${IMAGE_STAGE_INPUT_SHA}" "${AGENT_STAGE_SOURCE_SHA}")"
readonly HOST_BUILD_STAGE_INPUT_SHA="$(CombineFingerprint host-build \
  "${IMAGE_STAGE_INPUT_SHA}" "${HOST_BUILD_INPUT_SHA}")"
readonly HOST_PACKAGE_STAGE_INPUT_SHA="$(CombineFingerprint host-package \
  "${HOST_INPUT_SHA}")"
readonly CURRENT_IDENTITY="$(
  {
    printf 'format=rrclite-rdk-input-v1\n'
    printf 'firmware=%s\n' "${FIRMWARE_INPUT_SHA}"
    printf 'host=%s\n' "${HOST_INPUT_SHA}"
    printf 'host_build=%s\n' "${HOST_BUILD_INPUT_SHA}"
    printf 'image_source=%s\n' "${PROJECT_IMAGE_SOURCE_SHA}"
    printf 'arm64_base=%s\n' "${ARM64_BASE_IMAGE}"
    printf 'image_stage=%s\n' "${IMAGE_STAGE_INPUT_SHA}"
    printf 'microros_stage=%s\n' "${MICROROS_STAGE_INPUT_SHA}"
    printf 'pid_stage=%s\n' "${PID_STAGE_INPUT_SHA}"
    printf 'agent_stage=%s\n' "${AGENT_STAGE_INPUT_SHA}"
    printf 'host_build_stage=%s\n' "${HOST_BUILD_STAGE_INPUT_SHA}"
    printf 'host_package_stage=%s\n' "${HOST_PACKAGE_STAGE_INPUT_SHA}"
    printf 'build_jobs=%s\n' "${RRCLITE_BUILD_JOBS}"
  } | sha256sum | awk '{print $1}'
)"
readonly STATE_ROOT="${PROJECT_ROOT}/build/rdk-handoff-state"
readonly ACTIVE_FILE="${STATE_ROOT}/ACTIVE"
readonly WORK_PARENT="${PROJECT_ROOT}/build/rdk-handoff-work"
readonly OUTPUT_PARENT="${PROJECT_ROOT}/build/rdk-handoff"
mkdir -p "${STATE_ROOT}" "${WORK_PARENT}" "${OUTPUT_PARENT}"

release_id=""
work_root=""
checkpoint_manifest=""

LoadActiveCheckpoint() {
  [[ -f "${ACTIVE_FILE}" && ! -L "${ACTIVE_FILE}" ]] || \
    Fail "active handoff state is missing or symbolic"
  [[ "$(awk 'END {print NR}' "${ACTIVE_FILE}")" == 1 ]] || \
    Fail "active handoff state must contain exactly one release ID"
  read -r release_id <"${ACTIVE_FILE}"
  [[ "${release_id}" =~ ^rdk-arm64-[0-9]{8}T[0-9]{6}Z$ ]] || \
    Fail "active handoff release ID is malformed"
  work_root="${WORK_PARENT}/${release_id}"
  checkpoint_manifest="${work_root}/.rdk-handoff/CHECKPOINT.txt"
  [[ -d "${work_root}" && ! -L "${work_root}" && \
     -f "${checkpoint_manifest}" && ! -L "${checkpoint_manifest}" ]] || \
    Fail "active handoff work or checkpoint manifest is missing or symbolic"
  awk -F= '
    $1 ~ /^(format|release_id|input_identity|target_architecture|build_execution|build_host_architecture|native_target_validated|build_jobs|firmware_source_sha256|host_source_sha256|host_build_source_sha256|image_source_sha256|arm64_base_image|image_stage_input_sha256|microros_stage_input_sha256|pid_stage_input_sha256|agent_stage_input_sha256|host_build_stage_input_sha256|host_package_stage_input_sha256|completed_stage)$/ {next}
    $1 ~ /^stage_(image-preparation|microros-library|pid-firmware|agent-build|host-build-tests|host-oci-packaging|runtime-smoke|final-bundle)_sha256$/ {next}
    {exit 1}
  ' "${checkpoint_manifest}" || Fail "active handoff checkpoint contains unknown data"
  RequireUniqueField "${checkpoint_manifest}" format rrclite-rdk-checkpoint-v1
  RequireUniqueField "${checkpoint_manifest}" release_id "${release_id}"
  RequireUniqueField "${checkpoint_manifest}" target_architecture arm64
  RequireUniqueField "${checkpoint_manifest}" build_execution qemu-emulated
  RequireUniqueField "${checkpoint_manifest}" build_host_architecture amd64
  RequireUniqueField "${checkpoint_manifest}" native_target_validated 0
  for hash_field in input_identity firmware_source_sha256 host_source_sha256 \
      image_source_sha256; do
    hash_value="$(sed -n "s/^${hash_field}=//p" "${checkpoint_manifest}")"
    [[ "$(awk -F= -v key="${hash_field}" '$1 == key {count++} END {print count + 0}' \
        "${checkpoint_manifest}")" == 1 && "${hash_value}" =~ ^[0-9a-f]{64}$ ]] || \
      Fail "active handoff checkpoint ${hash_field} is malformed or duplicated"
  done
  build_jobs_value="$(sed -n 's/^build_jobs=//p' "${checkpoint_manifest}")"
  [[ "$(awk -F= '$1 == "build_jobs" {count++} END {print count + 0}' \
      "${checkpoint_manifest}")" == 1 && \
     "${build_jobs_value}" =~ ^[1-9][0-9]*$ ]] || \
    Fail "active handoff checkpoint build jobs are malformed or duplicated"
  base_image_value="$(sed -n 's/^arm64_base_image=//p' "${checkpoint_manifest}")"
  [[ "$(awk -F= '$1 == "arm64_base_image" {count++} END {print count + 0}' \
      "${checkpoint_manifest}")" == 1 && \
     "${base_image_value}" =~ @sha256:[0-9a-f]{64}$ ]] || \
    Fail "active handoff checkpoint base image is malformed or duplicated"
  completed_value="$(sed -n 's/^completed_stage=//p' "${checkpoint_manifest}")"
  [[ "$(awk -F= '$1 == "completed_stage" {count++} END {print count + 0}' \
      "${checkpoint_manifest}")" == 1 && \
     "${completed_value}" =~ ^(none|image-preparation|microros-library|pid-firmware|agent-build|host-build-tests|host-oci-packaging|runtime-smoke|final-bundle)$ ]] || \
    Fail "active handoff checkpoint completed stage is malformed or duplicated"
  for stage_name in image-preparation microros-library pid-firmware agent-build \
      host-build-tests host-oci-packaging runtime-smoke final-bundle; do
    stage_count="$(awk -F= -v key="stage_${stage_name}_sha256" \
      '$1 == key {count++} END {print count + 0}' "${checkpoint_manifest}")"
    ((stage_count <= 1)) || Fail "active checkpoint duplicates ${stage_name} checksum"
    if ((stage_count == 1)); then
      stage_hash="$(sed -n "s/^stage_${stage_name}_sha256=//p" \
        "${checkpoint_manifest}")"
      [[ "${stage_hash}" =~ ^[0-9a-f]{64}$ ]] || \
        Fail "active checkpoint has malformed ${stage_name} checksum"
    fi
  done
  for stage_input_field in image_stage_input_sha256 \
      microros_stage_input_sha256 pid_stage_input_sha256 \
      agent_stage_input_sha256 host_build_stage_input_sha256 \
      host_package_stage_input_sha256; do
    stage_input_count="$(awk -F= -v key="${stage_input_field}" \
      '$1 == key {count++} END {print count + 0}' "${checkpoint_manifest}")"
    ((stage_input_count <= 1)) || \
      Fail "active checkpoint duplicates ${stage_input_field}"
    if ((stage_input_count == 1)); then
      stage_input_value="$(sed -n "s/^${stage_input_field}=//p" \
        "${checkpoint_manifest}")"
      [[ "${stage_input_value}" =~ ^[0-9a-f]{64}$ ]] || \
        Fail "active checkpoint has malformed ${stage_input_field}"
    fi
  done
}

RemoveActiveCheckpoint() {
  LoadActiveCheckpoint
  case "${work_root}" in
    "${WORK_PARENT}"/rdk-arm64-*) ;;
    *) Fail "refusing to remove unexpected handoff work path: ${work_root}" ;;
  esac
  generated_output="${OUTPUT_PARENT}/${release_id}"
  if [[ -e "${generated_output}" || -L "${generated_output}" ]]; then
    [[ -d "${generated_output}" && ! -L "${generated_output}" ]] || \
      Fail "refusing to remove unexpected handoff output: ${generated_output}"
    RequireLine "${generated_output}/RDK-HANDOFF.txt" \
      'package_format=rrclite-rdk-handoff-v1'
    RequireLine "${generated_output}/RDK-HANDOFF.txt" "release_id=${release_id}"
    case "${generated_output}" in
      "${OUTPUT_PARENT}"/rdk-arm64-*) rm -rf -- "${generated_output}" ;;
      *) Fail "refusing to remove unexpected handoff output path" ;;
    esac
  fi
  rm -rf -- "${work_root}"
  rm -f -- "${ACTIVE_FILE}"
}

FindReusableCompletedHandoff() {
  local project_image project_image_id project_image_platform project_image_source
  local agent_lock_sha agent_patch_sha candidate metadata release_name
  local runtime_id runtime_source_id stage_field_count
  project_image="mentor-pi/rrclite:humble-arm64-${PROJECT_IMAGE_SOURCE_SHA:0:16}"
  project_image_id="$(docker image inspect "${project_image}" \
    --format '{{.Id}}' 2>/dev/null || true)"
  project_image_platform="$(docker image inspect "${project_image}" \
    --format '{{.Os}}/{{.Architecture}}' 2>/dev/null || true)"
  project_image_source="$(docker image inspect "${project_image}" \
    --format '{{index .Config.Labels "org.mentor-pi.image.source-sha256"}}' \
    2>/dev/null || true)"
  [[ "${project_image_id}" =~ ^sha256:[0-9a-f]{64}$ && \
     "${project_image_platform}" == linux/arm64 && \
     "${project_image_source}" == "${PROJECT_IMAGE_SOURCE_SHA}" ]] || return 1
  agent_lock_sha="$(Sha256 "${SCRIPT_DIR}/microros_agent_source.lock")"
  agent_patch_sha="$(Sha256 \
    "${SCRIPT_DIR}/patches/micro_xrce_agent_rrclite_modem_lines.patch")"

  while IFS= read -r candidate; do
    [[ -d "${candidate}" && ! -L "${candidate}" ]] || continue
    release_name="$(basename "${candidate}")"
    [[ "${release_name}" =~ ^rdk-arm64-[0-9]{8}T[0-9]{6}Z$ ]] || continue
    metadata="${candidate}/RDK-HANDOFF.txt"
    [[ -f "${metadata}" && ! -L "${metadata}" && \
       -f "${candidate}/SHA256SUMS" && \
       ! -L "${candidate}/SHA256SUMS" ]] || continue
    (cd "${candidate}" && sha256sum --check SHA256SUMS >/dev/null 2>&1) || \
      continue
    for expected in \
        'package_format=rrclite-rdk-handoff-v1' \
        "release_id=${release_name}" \
        'build_execution=qemu-emulated' \
        'build_host_architecture=amd64' \
        'target_architecture=arm64' \
        'native_target_validated=0' \
        "source_sha256=${HOST_INPUT_SHA}"; do
      grep -Fqx "${expected}" "${metadata}" || continue 2
    done
    grep -Fqx "source_sha256=${FIRMWARE_INPUT_SHA}" \
      "${candidate}/board-handoff/firmware-pid-release/BUILD-METADATA.txt" || \
      continue
    grep -Fqx "source_sha256=${HOST_BUILD_INPUT_SHA}" \
      "${candidate}/host-handoff/host/HOST-BUILD-METADATA.txt" || continue
    grep -Fqx "source_lock_sha256=${agent_lock_sha}" \
      "${candidate}/host-handoff/agent/AGENT-BUILD-METADATA.txt" || continue
    grep -Fqx "rrclite_patch_sha256=${agent_patch_sha}" \
      "${candidate}/host-handoff/agent/AGENT-BUILD-METADATA.txt" || continue
    grep -Fqx "builder_image_id=${project_image_id}" \
      "${candidate}/host-handoff/HOST-HANDOFF.txt" || continue
    grep -Fqx "runtime_image_source_id=${project_image_id}" \
      "${candidate}/host-handoff/HOST-HANDOFF.txt" || continue

    # New bundles bind every stage input directly. Field-free older bundles
    # use the artifact identities checked above.
    stage_field_count="$(awk -F= '
      $1 ~ /^(image_stage_input_sha256|pid_stage_input_sha256|agent_stage_input_sha256|host_build_stage_input_sha256|host_package_stage_input_sha256)$/ {count++}
      END {print count + 0}
    ' "${metadata}")"
    if [[ "${stage_field_count}" != 0 ]]; then
      [[ "${stage_field_count}" == 5 ]] || continue
      for expected in \
          "image_stage_input_sha256=${IMAGE_STAGE_INPUT_SHA}" \
          "pid_stage_input_sha256=${PID_STAGE_INPUT_SHA}" \
          "agent_stage_input_sha256=${AGENT_STAGE_INPUT_SHA}" \
          "host_build_stage_input_sha256=${HOST_BUILD_STAGE_INPUT_SHA}" \
          "host_package_stage_input_sha256=${HOST_PACKAGE_STAGE_INPUT_SHA}"; do
        grep -Fqx "${expected}" "${metadata}" || continue 2
      done
    fi

    runtime_id="$(sed -n 's/^runtime_image_id=//p' \
      "${candidate}/host-handoff/HOST-HANDOFF.txt")"
    runtime_source_id="$(sed -n 's/^runtime_image_source_id=//p' \
      "${candidate}/host-handoff/HOST-HANDOFF.txt")"
    [[ "${runtime_source_id}" == "${project_image_id}" ]] || continue
    python3 "${OCI_ARCHIVE_VERIFIER}" \
      --archive "${candidate}/host-handoff/runtime-image/mentor-pi-runtime.tar" \
      --image-id "${runtime_id}" --os linux --architecture arm64 \
      >/dev/null 2>&1 || continue
    printf '%s\n' "${candidate}"
    return 0
  done < <(find "${OUTPUT_PARENT}" -mindepth 1 -maxdepth 1 -type d \
    -name 'rdk-arm64-????????T??????Z' -print | LC_ALL=C sort -r)
  return 1
}

if [[ "${RDK_HANDOFF_FRESH:-0}" == 0 ]]; then
  reusable_handoff="$(FindReusableCompletedHandoff || true)"
  if [[ -n "${reusable_handoff}" ]]; then
    echo "Reusing compatible completed RDK handoff: ${reusable_handoff}"
    if [[ -e "${ACTIVE_FILE}" || -L "${ACTIVE_FILE}" ]]; then
      LoadActiveCheckpoint
      redundant_work="${work_root}"
      RemoveActiveCheckpoint
      echo "Removed redundant incomplete RDK handoff work: ${redundant_work}"
    fi
    echo "Checksummed arm64 RDK handoff: ${reusable_handoff}"
    exit 0
  fi
fi

resume=0
refresh_checkpoint=0
if [[ -e "${ACTIVE_FILE}" || -L "${ACTIVE_FILE}" ]]; then
  LoadActiveCheckpoint
  recorded_identity="$(sed -n 's/^input_identity=//p' "${checkpoint_manifest}")"
  [[ "${recorded_identity}" =~ ^[0-9a-f]{64}$ ]] || \
    Fail "active handoff input identity is malformed"
  if [[ "${RDK_HANDOFF_FRESH:-0}" == 1 ]]; then
    echo "Deleting incompatible generated handoff checkpoint: ${work_root}"
    RemoveActiveCheckpoint
    release_id=""
    work_root=""
    checkpoint_manifest=""
  elif [[ "${recorded_identity}" != "${CURRENT_IDENTITY}" ]]; then
    resume=1
    refresh_checkpoint=1
    echo "Refreshing changed inputs while preserving unaffected RDK stages: ${release_id}"
  else
    resume=1
    echo "Resuming compatible RDK handoff: ${release_id}"
  fi
fi

if ((resume == 0)); then
  readonly STAMP="$(date -u '+%Y%m%dT%H%M%SZ')"
  release_id="rdk-arm64-${STAMP}"
  work_root="${WORK_PARENT}/${release_id}"
  checkpoint_manifest="${work_root}/.rdk-handoff/CHECKPOINT.txt"
  [[ ! -e "${work_root}" && ! -L "${work_root}" ]] || \
    Fail "generated handoff work path already exists"
fi
readonly RELEASE_ID="${release_id}"
readonly WORK_ROOT="${work_root}"
readonly CHECKPOINT_MANIFEST="${checkpoint_manifest}"
readonly OUTPUT_ROOT="${OUTPUT_PARENT}/${RELEASE_ID}"
readonly MICROROS_CACHE_ROOT="${PROJECT_ROOT}/build/rdk-handoff-cache/arm64/microros"
readonly MICROROS_WORK_ROOT="${WORK_ROOT}/firmware/mentor_pi_mcu/build/microros"
readonly MICROROS_LIBRARY="${MICROROS_WORK_ROOT}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a"
readonly CHECKPOINT_ROOT="${WORK_ROOT}/.rdk-handoff/checkpoints"
readonly HOST_HANDOFF="${WORK_ROOT}/build/host-handoff/${RELEASE_ID}"
readonly HOST_WORK="${WORK_ROOT}/build/host-handoff-work/${RELEASE_ID}-arm64"
readonly HOST_PREFIX="${HOST_WORK}/prefix"
readonly BOARD_HANDOFF="${WORK_ROOT}/build/board-handoff/${RELEASE_ID}"
staging_root=""
cache_staging=""
cache_previous=""
current_stage=initialization
completed_successfully=0
Cleanup() {
  status="$?"
  [[ -z "${staging_root}" || ! -d "${staging_root}" ]] || rm -rf -- "${staging_root}"
  [[ -z "${cache_staging}" || ! -d "${cache_staging}" ]] || rm -rf -- "${cache_staging}"
  if [[ -n "${cache_previous}" && -d "${cache_previous}" ]]; then
    if [[ ! -e "${MICROROS_CACHE_ROOT}" ]]; then
      mv "${cache_previous}" "${MICROROS_CACHE_ROOT}"
    else
      rm -rf -- "${cache_previous}"
    fi
  fi
  CleanupRunnerSnapshot
  if ((completed_successfully == 0)); then
    echo "RDK handoff stopped during stage '${current_stage}'." >&2
    echo "Preserved resumable work: ${WORK_ROOT}" >&2
    echo "Retry with: make rdk-handoff" >&2
  fi
  return "${status}"
}
trap Cleanup EXIT

WriteCheckpointManifest() {
  manifest_tmp="${CHECKPOINT_MANIFEST}.tmp.$$"
  cat >"${manifest_tmp}" <<EOF
format=rrclite-rdk-checkpoint-v1
release_id=${RELEASE_ID}
input_identity=${CURRENT_IDENTITY}
target_architecture=arm64
build_execution=qemu-emulated
build_host_architecture=amd64
native_target_validated=0
build_jobs=${RRCLITE_BUILD_JOBS}
firmware_source_sha256=${FIRMWARE_INPUT_SHA}
host_source_sha256=${HOST_INPUT_SHA}
host_build_source_sha256=${HOST_BUILD_INPUT_SHA}
image_source_sha256=${PROJECT_IMAGE_SOURCE_SHA}
arm64_base_image=${ARM64_BASE_IMAGE}
image_stage_input_sha256=${IMAGE_STAGE_INPUT_SHA}
microros_stage_input_sha256=${MICROROS_STAGE_INPUT_SHA}
pid_stage_input_sha256=${PID_STAGE_INPUT_SHA}
agent_stage_input_sha256=${AGENT_STAGE_INPUT_SHA}
host_build_stage_input_sha256=${HOST_BUILD_STAGE_INPUT_SHA}
host_package_stage_input_sha256=${HOST_PACKAGE_STAGE_INPUT_SHA}
completed_stage=none
EOF
  mv "${manifest_tmp}" "${CHECKPOINT_MANIFEST}"
}

RequireCheckpointIdentity() {
  for expected in \
      'format=rrclite-rdk-checkpoint-v1' \
      "release_id=${RELEASE_ID}" \
      "input_identity=${CURRENT_IDENTITY}" \
      'target_architecture=arm64' \
      'build_execution=qemu-emulated' \
      'build_host_architecture=amd64' \
      'native_target_validated=0' \
      "build_jobs=${RRCLITE_BUILD_JOBS}" \
      "firmware_source_sha256=${FIRMWARE_INPUT_SHA}" \
      "host_source_sha256=${HOST_INPUT_SHA}" \
      "host_build_source_sha256=${HOST_BUILD_INPUT_SHA}" \
      "image_source_sha256=${PROJECT_IMAGE_SOURCE_SHA}" \
      "arm64_base_image=${ARM64_BASE_IMAGE}"; do
    RequireUniqueField "${CHECKPOINT_MANIFEST}" "${expected%%=*}" "${expected#*=}"
  done
  for expected in \
      "image_stage_input_sha256=${IMAGE_STAGE_INPUT_SHA}" \
      "microros_stage_input_sha256=${MICROROS_STAGE_INPUT_SHA}" \
      "pid_stage_input_sha256=${PID_STAGE_INPUT_SHA}" \
      "agent_stage_input_sha256=${AGENT_STAGE_INPUT_SHA}" \
      "host_build_stage_input_sha256=${HOST_BUILD_STAGE_INPUT_SHA}" \
      "host_package_stage_input_sha256=${HOST_PACKAGE_STAGE_INPUT_SHA}"; do
    RequireUniqueField "${CHECKPOINT_MANIFEST}" "${expected%%=*}" "${expected#*=}"
  done
  completed_stage="$(sed -n 's/^completed_stage=//p' "${CHECKPOINT_MANIFEST}")"
  [[ "$(awk -F= '$1 == "completed_stage" {count++} END {print count + 0}' \
      "${CHECKPOINT_MANIFEST}")" == 1 && \
     "${completed_stage}" =~ ^(none|image-preparation|microros-library|pid-firmware|agent-build|host-build-tests|host-oci-packaging|runtime-smoke|final-bundle)$ ]] || \
    Fail "active handoff completed stage is missing, duplicated, or malformed"
  [[ "$("${FIRMWARE_FINGERPRINT}" firmware "${WORK_ROOT}")" == \
      "${FIRMWARE_INPUT_SHA}" ]] || \
    Fail "resumable firmware source copy was changed"
  [[ "$("${HOST_FINGERPRINT}" --package-content "${WORK_ROOT}")" == \
      "${HOST_INPUT_SHA}" ]] || \
    Fail "resumable host source copy was changed"
}

StageMarker() { printf '%s/%s.complete\n' "${CHECKPOINT_ROOT}" "$1"; }
StageDone() {
  marker="$(StageMarker "$1")"
  [[ -f "${marker}" && ! -L "${marker}" ]] || return 1
  [[ "$(awk 'END {print NR}' "${marker}")" == 4 ]] || return 1
  grep -Fqx "stage=$1" "${marker}" && \
    grep -Fqx "input_identity=${CURRENT_IDENTITY}" "${marker}" && \
    grep -Eq '^output_sha256=[0-9a-f]{64}$' "${marker}" && \
    grep -Eq '^completed_utc=[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$' \
      "${marker}"
}
CompleteStage() {
  [[ "$#" == 2 && "$2" =~ ^[0-9a-f]{64}$ ]] || \
    Fail "stage $1 produced an invalid output checksum"
  marker="$(StageMarker "$1")"
  marker_tmp="${marker}.tmp.$$"
  cat >"${marker_tmp}" <<EOF
stage=$1
input_identity=${CURRENT_IDENTITY}
output_sha256=$2
completed_utc=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
EOF
  mv "${marker_tmp}" "${marker}"
  checkpoint_tmp="${CHECKPOINT_MANIFEST}.tmp.$$"
  sed '/^completed_stage=/d' "${CHECKPOINT_MANIFEST}" >"${checkpoint_tmp}"
  printf 'completed_stage=%s\nstage_%s_sha256=%s\n' "$1" "$1" "$2" \
    >>"${checkpoint_tmp}"
  mv "${checkpoint_tmp}" "${CHECKPOINT_MANIFEST}"
  echo "Completed RDK handoff stage: $1"
}
VerifyStageChecksum() {
  [[ "$#" == 2 && "$2" =~ ^[0-9a-f]{64}$ ]] || return 1
  RequireLine "$(StageMarker "$1")" "output_sha256=$2"
  RequireLine "${CHECKPOINT_MANIFEST}" "stage_$1_sha256=$2"
}
BeginStage() {
  current_stage="$1"
  if StageDone "$1"; then
    echo "Validating completed RDK handoff stage: $1"
    return 1
  fi
  marker="$(StageMarker "$1")"
  [[ ! -e "${marker}" && ! -L "${marker}" ]] || \
    Fail "stage checkpoint is malformed, symbolic, or tampered: ${marker}"
  echo "Starting RDK handoff stage: $1"
  return 0
}

RefreshChangedInputs() {
  local old_identity old_image old_firmware old_host old_host_build
  local old_image_stage old_microros_stage old_pid_stage old_agent_stage
  local old_host_build_stage old_host_package_stage
  local old_host_package_payload refresh_package_docs=0
  local old_micro_source old_agent_source
  local package_source package_manifest package_checksum
  local runtime_marker runtime_checksum file relative
  old_identity="$(sed -n 's/^input_identity=//p' "${CHECKPOINT_MANIFEST}")"
  old_image="$(sed -n 's/^image_source_sha256=//p' "${CHECKPOINT_MANIFEST}")"
  old_firmware="$(sed -n 's/^firmware_source_sha256=//p' "${CHECKPOINT_MANIFEST}")"
  old_host="$("${HOST_FINGERPRINT}" --package-content "${WORK_ROOT}")"
  old_host_package_payload="$("${HOST_FINGERPRINT}" --package-payload \
    "${WORK_ROOT}")"
  old_host_build="$("${HOST_FINGERPRINT}" --compile "${WORK_ROOT}")"
  old_image_stage="$(sed -n 's/^image_stage_input_sha256=//p' \
    "${CHECKPOINT_MANIFEST}")"
  [[ -n "${old_image_stage}" ]] || old_image_stage="$(CombineFingerprint image \
    "${old_image}" "$(sed -n 's/^arm64_base_image=//p' "${CHECKPOINT_MANIFEST}")")"
  old_microros_stage="$(sed -n 's/^microros_stage_input_sha256=//p' \
    "${CHECKPOINT_MANIFEST}")"
  if [[ -z "${old_microros_stage}" ]]; then
    old_micro_source="$(RelativeInputsFingerprint "${WORK_ROOT}" \
      firmware/mentor_pi_mcu/config/microros_sources.lock \
      firmware/mentor_pi_mcu/config/microros_colcon.meta \
      firmware/mentor_pi_mcu/config/microros_toolchain.cmake \
      firmware/mentor_pi_mcu/config/microros_library_generation.sh \
      tools/apply_microros_source_lock.sh tools/build_microros_library.sh \
      tools/microros_artifact_fingerprint.sh tools/verify_microros_cache.sh)"
    old_microros_stage="$(CombineFingerprint microros "${old_image_stage}" \
      "$("${FIRMWARE_FINGERPRINT}" interfaces "${WORK_ROOT}")" \
      "${old_micro_source}")"
  fi
  old_pid_stage="$(sed -n 's/^pid_stage_input_sha256=//p' "${CHECKPOINT_MANIFEST}")"
  [[ -n "${old_pid_stage}" ]] || old_pid_stage="$(CombineFingerprint pid \
    "${old_image_stage}" "${old_firmware}" \
    "$(Sha256 "${WORK_ROOT}/tools/package_board_handoff.sh")")"
  old_agent_stage="$(sed -n 's/^agent_stage_input_sha256=//p' \
    "${CHECKPOINT_MANIFEST}")"
  if [[ -z "${old_agent_stage}" ]]; then
    old_agent_source="$(RelativeInputsFingerprint "${WORK_ROOT}" \
      tools/build_agent.sh tools/build_microros_agent_from_lock.sh \
      tools/microros_agent_source.lock \
      tools/patches/micro_xrce_agent_rrclite_modem_lines.patch)"
    old_agent_stage="$(CombineFingerprint agent "${old_image_stage}" \
      "${old_agent_source}")"
  fi
  # Recompute this value from the copied compile inputs. This also migrates
  # checkpoints created before the build/package fingerprint split without
  # invalidating an already compiled host prefix.
  old_host_build_stage="$(CombineFingerprint host-build "${old_image_stage}" \
    "${old_host_build}")"
  # Recompute from files that can change the package bytes. This migrates old
  # broad checkpoints and prevents validator-only edits from repackaging a
  # completed OCI archive.
  old_host_package_stage="$(CombineFingerprint host-package "${old_host}")"

  local image_same=0 microros_same=0 pid_same=0 agent_same=0
  local host_build_same=0 host_package_same=0
  [[ "${old_image_stage}" != "${IMAGE_STAGE_INPUT_SHA}" ]] || image_same=1
  [[ "${old_microros_stage}" != "${MICROROS_STAGE_INPUT_SHA}" ]] || microros_same=1
  [[ "${old_pid_stage}" != "${PID_STAGE_INPUT_SHA}" ]] || pid_same=1
  [[ "${old_agent_stage}" != "${AGENT_STAGE_INPUT_SHA}" ]] || agent_same=1
  [[ "${old_host_build_stage}" != "${HOST_BUILD_STAGE_INPUT_SHA}" ]] || host_build_same=1
  [[ "${old_host_package_stage}" != "${HOST_PACKAGE_STAGE_INPUT_SHA}" ]] || host_package_same=1
  if ((host_package_same == 0)) && \
      [[ "${old_host_package_payload}" == "${HOST_PACKAGE_PAYLOAD_SHA}" && \
         -f "$(StageMarker host-oci-packaging)" && \
         ! -L "$(StageMarker host-oci-packaging)" && \
         -d "${HOST_HANDOFF}" && ! -L "${HOST_HANDOFF}" ]] && \
      (cd "${HOST_HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null); then
    host_package_same=1
    refresh_package_docs=1
    echo "Refreshing only packaged tutorials; preserving the OCI archive and prefixes."
  fi

  declare -A preserve=()
  preserve[image-preparation]="${image_same}"
  preserve[microros-library]="${microros_same}"
  preserve[pid-firmware]="${pid_same}"
  preserve[agent-build]="${agent_same}"
  preserve[host-build-tests]="${host_build_same}"
  preserve[host-oci-packaging]=$((host_package_same && pid_same && agent_same && host_build_same))
  preserve[runtime-smoke]="${preserve[host-oci-packaging]}"
  preserve[final-bundle]=$((preserve[host-oci-packaging] && ! refresh_package_docs))

  local -a stages=(image-preparation microros-library pid-firmware agent-build \
    host-build-tests host-oci-packaging runtime-smoke final-bundle)
  local stage marker checksum latest=none
  declare -A preserved_checksum=()
  for stage in "${stages[@]}"; do
    marker="$(StageMarker "${stage}")"
    if [[ -e "${marker}" || -L "${marker}" ]]; then
      [[ -f "${marker}" && ! -L "${marker}" && \
         "$(awk 'END {print NR}' "${marker}")" == 4 ]] || \
        Fail "cannot refresh malformed or symbolic stage marker: ${marker}"
      RequireLine "${marker}" "stage=${stage}"
      RequireLine "${marker}" "input_identity=${old_identity}"
      checksum="$(sed -n 's/^output_sha256=//p' "${marker}")"
      [[ "${checksum}" =~ ^[0-9a-f]{64}$ ]] || \
        Fail "cannot refresh malformed stage checksum: ${stage}"
      grep -Eq '^completed_utc=[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$' \
        "${marker}" || Fail "cannot refresh malformed stage timestamp: ${stage}"
      if [[ "${preserve[${stage}]}" == 1 ]]; then
        sed "s/^input_identity=.*/input_identity=${CURRENT_IDENTITY}/" \
          "${marker}" >"${marker}.tmp.$$"
        mv "${marker}.tmp.$$" "${marker}"
        preserved_checksum[${stage}]="${checksum}"
        latest="${stage}"
        echo "Preserving unaffected completed RDK stage: ${stage}"
      else
        rm -f -- "${marker}"
        echo "Invalidating affected RDK stage: ${stage}"
      fi
    fi
  done

  if ((pid_same == 0)); then
    if [[ -e "${BOARD_HANDOFF}" || -L "${BOARD_HANDOFF}" ]]; then
      [[ -d "${BOARD_HANDOFF}" && ! -L "${BOARD_HANDOFF}" ]] || \
        Fail "refusing to invalidate unexpected board handoff path"
      rm -rf -- "${BOARD_HANDOFF}"
    fi
  fi
  if ((host_build_same == 0)); then
    if [[ -e "${HOST_WORK}" || -L "${HOST_WORK}" ]]; then
      [[ -d "${HOST_WORK}" && ! -L "${HOST_WORK}" ]] || \
        Fail "refusing to refresh unexpected host work path"
      rm -f -- "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt"
    fi
  fi
  if ((image_same == 0)); then
    if [[ -e "${HOST_WORK}" || -L "${HOST_WORK}" ]]; then
      [[ -d "${HOST_WORK}" && ! -L "${HOST_WORK}" ]] || \
        Fail "refusing to invalidate unexpected host work path"
      rm -rf -- "${HOST_WORK}"
    fi
  fi
  if [[ "${preserve[host-oci-packaging]}" == 0 ]]; then
    if [[ -e "${HOST_HANDOFF}" || -L "${HOST_HANDOFF}" ]]; then
      [[ -d "${HOST_HANDOFF}" && ! -L "${HOST_HANDOFF}" ]] || \
        Fail "refusing to invalidate unexpected host handoff path"
      rm -rf -- "${HOST_HANDOFF}"
    fi
  fi
  if [[ "${preserve[final-bundle]}" == 0 && -e "${OUTPUT_ROOT}" ]]; then
    [[ -d "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
      Fail "refusing to remove unexpected final handoff output"
    rm -rf -- "${OUTPUT_ROOT}"
  fi

  rsync -a --delete --exclude '/build/' --exclude 'build/' \
    --exclude '/.git/' --exclude '/docs/reference/' \
    --exclude '/.rdk-handoff/' --exclude '/.rdk-handoff-bin/' \
    "${PROJECT_ROOT}/" "${WORK_ROOT}/"
  [[ "$("${FIRMWARE_FINGERPRINT}" firmware "${WORK_ROOT}")" == \
      "${FIRMWARE_INPUT_SHA}" ]] || Fail "refreshed firmware context differs"
  [[ "$("${HOST_FINGERPRINT}" --package-content "${WORK_ROOT}")" == \
      "${HOST_INPUT_SHA}" ]] || \
    Fail "refreshed host context differs"
  [[ "$("${HOST_FINGERPRINT}" --compile "${WORK_ROOT}")" == \
      "${HOST_BUILD_INPUT_SHA}" ]] || Fail "refreshed host build context differs"

  if ((refresh_package_docs == 1)); then
    [[ -d "${HOST_HANDOFF}/docs/tutorials" && \
       ! -L "${HOST_HANDOFF}/docs/tutorials" ]] || \
      Fail "packaged tutorial directory is missing or symbolic"
    rm -rf -- "${HOST_HANDOFF}/docs/tutorials"
    mkdir -p "${HOST_HANDOFF}/docs/tutorials"
    cp -a "${WORK_ROOT}/docs/tutorials/." \
      "${HOST_HANDOFF}/docs/tutorials/"
    package_source="$(${HOST_FINGERPRINT} "${WORK_ROOT}")"
    [[ "$(awk -F= '$1 == "source_sha256" {count++} END {print count + 0}' \
      "${HOST_HANDOFF}/HOST-HANDOFF.txt")" == 1 ]] || \
      Fail "packaged host source fingerprint is missing or duplicated"
    sed -i "s/^source_sha256=.*/source_sha256=${package_source}/" \
      "${HOST_HANDOFF}/HOST-HANDOFF.txt"
    RequireLine "${HOST_HANDOFF}/HOST-HANDOFF.txt" \
      "source_sha256=${package_source}"
    package_manifest="${HOST_HANDOFF}/SHA256SUMS"
    : >"${package_manifest}.tmp.$$"
    while IFS= read -r file; do
      relative="${file#"${HOST_HANDOFF}/"}"
      printf '%s  %s\n' "$(Sha256 "${file}")" "${relative}" \
        >>"${package_manifest}.tmp.$$"
    done < <(find "${HOST_HANDOFF}" -type f ! -name SHA256SUMS \
      ! -name 'SHA256SUMS.tmp.*' -print | LC_ALL=C sort)
    mv "${package_manifest}.tmp.$$" "${package_manifest}"
    (cd "${HOST_HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null)
    package_checksum="$(Sha256 "${package_manifest}")"
    marker="$(StageMarker host-oci-packaging)"
    sed -i "s/^output_sha256=.*/output_sha256=${package_checksum}/" "${marker}"
    preserved_checksum[host-oci-packaging]="${package_checksum}"
    runtime_marker="$(StageMarker runtime-smoke)"
    if [[ -f "${runtime_marker}" && ! -L "${runtime_marker}" ]]; then
      runtime_checksum="$(printf 'runtime-smoke:%s\n' \
        "${package_checksum}" | sha256sum | awk '{print $1}')"
      sed -i "s/^output_sha256=.*/output_sha256=${runtime_checksum}/" \
        "${runtime_marker}"
      preserved_checksum[runtime-smoke]="${runtime_checksum}"
    fi
  fi

  WriteCheckpointManifest
  checkpoint_tmp="${CHECKPOINT_MANIFEST}.tmp.$$"
  sed '/^completed_stage=/d' "${CHECKPOINT_MANIFEST}" >"${checkpoint_tmp}"
  printf 'completed_stage=%s\n' "${latest}" >>"${checkpoint_tmp}"
  for stage in "${stages[@]}"; do
    [[ -z "${preserved_checksum[${stage}]:-}" ]] || \
      printf 'stage_%s_sha256=%s\n' "${stage}" \
        "${preserved_checksum[${stage}]}" >>"${checkpoint_tmp}"
  done
  mv "${checkpoint_tmp}" "${CHECKPOINT_MANIFEST}"
}

if ((resume == 0)); then
  rsync -a --exclude '/build/' --exclude 'build/' --exclude '/.git/' \
    --exclude '/docs/reference/' "${PROJECT_ROOT}/" "${WORK_ROOT}/"
  [[ "$("${FIRMWARE_FINGERPRINT}" firmware "${PROJECT_ROOT}")" == \
        "${FIRMWARE_INPUT_SHA}" && \
      "$("${FIRMWARE_FINGERPRINT}" firmware "${WORK_ROOT}")" == \
        "${FIRMWARE_INPUT_SHA}" ]] || \
    Fail "firmware source changed while the immutable build context was copied"
  [[ "$("${HOST_FINGERPRINT}" --package-content "${PROJECT_ROOT}")" == \
        "${HOST_INPUT_SHA}" && \
      "$("${HOST_FINGERPRINT}" --package-content "${WORK_ROOT}")" == \
        "${HOST_INPUT_SHA}" ]] || \
    Fail "host source changed while the immutable build context was copied"
  [[ "$("${HOST_FINGERPRINT}" --compile "${PROJECT_ROOT}")" == \
        "${HOST_BUILD_INPUT_SHA}" && \
      "$("${HOST_FINGERPRINT}" --compile "${WORK_ROOT}")" == \
        "${HOST_BUILD_INPUT_SHA}" ]] || \
    Fail "host build source changed while the immutable context was copied"
  mkdir -p "${CHECKPOINT_ROOT}" "${WORK_ROOT}/.rdk-handoff-bin"
  WriteCheckpointManifest
  active_tmp="${ACTIVE_FILE}.tmp.$$"
  printf '%s\n' "${RELEASE_ID}" >"${active_tmp}"
  mv "${active_tmp}" "${ACTIVE_FILE}"
else
  if ((refresh_checkpoint == 1)); then
    RefreshChangedInputs
  fi
  RequireCheckpointIdentity
fi

cat >"${WORK_ROOT}/.rdk-handoff-bin/uname" <<'EOF'
#!/usr/bin/env bash
[[ "${1:-}" == -m ]] && { printf '%s\n' aarch64; exit 0; }
exec /usr/bin/uname "$@"
EOF
chmod 0755 "${WORK_ROOT}/.rdk-handoff-bin/uname"

export PATH="${WORK_ROOT}/.rdk-handoff-bin:${PATH}"
cd "${WORK_ROOT}"

if BeginStage image-preparation; then
  ./tools/prepare_build_images.sh --architecture arm64
  prepared_image="$(./tools/prepare_build_images.sh --architecture arm64 --print project)"
  image_output_sha="$(docker image inspect "${prepared_image}" --format '{{.Id}}' | sed 's/^sha256://')"
  CompleteStage image-preparation "${image_output_sha}"
else
  prepared_image="$(./tools/prepare_build_images.sh --architecture arm64 --print project)"
  image_output_sha="$(docker image inspect "${prepared_image}" --format '{{.Id}}' | sed 's/^sha256://')"
  VerifyStageChecksum image-preparation "${image_output_sha}"
fi

if BeginStage microros-library; then
  if [[ -e "${MICROROS_CACHE_ROOT}" || -L "${MICROROS_CACHE_ROOT}" ]]; then
    [[ -d "${MICROROS_CACHE_ROOT}" && ! -L "${MICROROS_CACHE_ROOT}" ]] || \
      Fail "persistent arm64 micro-ROS cache is not a regular directory"
    mkdir -p "${MICROROS_WORK_ROOT}"
    rsync -a "${MICROROS_CACHE_ROOT}/" "${MICROROS_WORK_ROOT}/"
    echo "Restored persistent arm64 micro-ROS cache for validation."
  fi
  ./tools/build_microros_library.sh
  mkdir -p "$(dirname "${MICROROS_CACHE_ROOT}")"
  cache_staging="$(mktemp -d "$(dirname "${MICROROS_CACHE_ROOT}")/.microros.XXXXXX")"
  rsync -a "${MICROROS_WORK_ROOT}/" "${cache_staging}/"
  if [[ -e "${MICROROS_CACHE_ROOT}" ]]; then
    cache_previous="${MICROROS_CACHE_ROOT}.previous.$$"
    [[ ! -e "${cache_previous}" ]] || Fail "stale cache backup exists: ${cache_previous}"
    mv "${MICROROS_CACHE_ROOT}" "${cache_previous}"
  fi
  mv "${cache_staging}" "${MICROROS_CACHE_ROOT}"
  cache_staging=""
  if [[ -n "${cache_previous}" ]]; then rm -rf -- "${cache_previous}"; cache_previous=""; fi
  microros_output_sha="$(Sha256 "${MICROROS_LIBRARY}")"
  CompleteStage microros-library "${microros_output_sha}"
else
  [[ -f "${MICROROS_LIBRARY}" && ! -L "${MICROROS_LIBRARY}" ]] || \
    Fail "completed micro-ROS stage has no library"
  VerifyStageChecksum microros-library "$(Sha256 "${MICROROS_LIBRARY}")"
fi

if BeginStage pid-firmware; then
  if [[ -d "${BOARD_HANDOFF}" && ! -L "${BOARD_HANDOFF}" ]] && \
      (cd "${BOARD_HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null); then
    echo "Adopting verified PID board package from the interrupted stage."
  else
    [[ ! -e "${BOARD_HANDOFF}" && ! -L "${BOARD_HANDOFF}" ]] || \
      Fail "unfinished PID board package exists but does not validate"
    ./tools/build_firmware.sh
    ./tools/package_board_handoff.sh --verified-build "build/board-handoff/${RELEASE_ID}"
  fi
  CompleteStage pid-firmware "$(Sha256 "${BOARD_HANDOFF}/SHA256SUMS")"
fi
RequireLine "${BOARD_HANDOFF}/HANDOFF.txt" 'target=STM32F407VET6'
RequireLine "${BOARD_HANDOFF}/firmware-pid-release/BUILD-MODE.txt" \
  'classification=NORMAL_CLOSED_LOOP_DEFAULT'
(
  cd "${BOARD_HANDOFF}"
  sha256sum --check SHA256SUMS >/dev/null
)
VerifyStageChecksum pid-firmware "$(Sha256 "${BOARD_HANDOFF}/SHA256SUMS")"

if BeginStage agent-build; then
  ./tools/build_agent.sh
  agent_prefix="$(./tools/build_agent.sh --print-output)"
  CompleteStage agent-build "$(Sha256 "${agent_prefix}/AGENT-BUILD-METADATA.txt")"
else
  ./tools/build_agent.sh
fi
agent_prefix="$(./tools/build_agent.sh --print-output)"
[[ -x "${agent_prefix}/lib/micro_ros_agent/micro_ros_agent" ]] || \
  Fail "completed Agent stage has no executable"
VerifyStageChecksum agent-build "$(Sha256 "${agent_prefix}/AGENT-BUILD-METADATA.txt")"

if BeginStage host-build-tests; then
  if [[ -f "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" && \
        ! -L "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" ]] && \
      grep -Fqx 'architecture=arm64' "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" && \
      grep -Fqx 'tests=qemu-passed-native-launch-deferred' \
        "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" && \
      grep -Fqx 'native_configuration_supervisor_launch_test=deferred' \
        "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" && \
      grep -Fqx 'tests=qemu-passed-native-launch-deferred' \
        "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" && \
      grep -Fqx 'native_configuration_supervisor_launch_test=deferred' \
        "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" && \
      grep -Fqx "source_sha256=${HOST_BUILD_INPUT_SHA}" \
        "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" && \
      grep -Fqx "metadata_sha256=$(Sha256 "${HOST_PREFIX}/HOST-BUILD-METADATA.txt")" \
        "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt"; then
    echo "Adopting verified host prefix from the interrupted stage."
  else
    host_args=(--architecture arm64 --release-id "${RELEASE_ID}" --build-only)
    [[ ! -d "${HOST_WORK}" ]] || host_args+=(--resume)
    ./tools/build_host_handoff_container.sh "${host_args[@]}"
  fi
  RequireLine "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" 'architecture=arm64'
  RequireLine "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" \
    'tests=qemu-passed-native-launch-deferred'
  RequireLine "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" \
    'native_configuration_supervisor_launch_test=deferred'
  RequireLine "${HOST_PREFIX}/HOST-BUILD-METADATA.txt" \
    "source_sha256=${HOST_BUILD_INPUT_SHA}"
  RequireLine "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" \
    'tests=qemu-passed-native-launch-deferred'
  RequireLine "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt" \
    'native_configuration_supervisor_launch_test=deferred'
  CompleteStage host-build-tests "$(Sha256 "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt")"
fi
VerifyStageChecksum host-build-tests "$(Sha256 "${HOST_PREFIX}/HOST-BUILD-COMPLETE.txt")"

if BeginStage host-oci-packaging; then
  if [[ -d "${HOST_HANDOFF}" && ! -L "${HOST_HANDOFF}" ]] && \
      (cd "${HOST_HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null); then
    echo "Adopting verified host/OCI package from the interrupted stage."
  else
    [[ ! -e "${HOST_HANDOFF}" && ! -L "${HOST_HANDOFF}" ]] || \
      Fail "unfinished host handoff package exists but does not validate"
    ./tools/build_host_handoff_container.sh --architecture arm64 \
      --release-id "${RELEASE_ID}" --package-only \
      --output-directory "build/host-handoff/${RELEASE_ID}"
  fi
  CompleteStage host-oci-packaging "$(Sha256 "${HOST_HANDOFF}/SHA256SUMS")"
fi
RequireLine "${HOST_HANDOFF}/HOST-HANDOFF.txt" 'architecture=arm64'
RequireLine "${HOST_HANDOFF}/host/HOST-BUILD-METADATA.txt" 'architecture=arm64'
RequireLine "${HOST_HANDOFF}/agent/AGENT-BUILD-METADATA.txt" 'architecture=arm64'
(
  cd "${HOST_HANDOFF}"
  sha256sum --check SHA256SUMS >/dev/null
)
VerifyStageChecksum host-oci-packaging "$(Sha256 "${HOST_HANDOFF}/SHA256SUMS")"

readonly RUNTIME_IMAGE_ID="$(sed -n 's/^runtime_image_id=//p' \
  "${HOST_HANDOFF}/HOST-HANDOFF.txt")"
readonly RUNTIME_IMAGE_SOURCE_ID="$(sed -n 's/^runtime_image_source_id=//p' \
  "${HOST_HANDOFF}/HOST-HANDOFF.txt")"
readonly SOURCE_SHA256="$(./tools/host_source_fingerprint.sh --package-content \
  "${WORK_ROOT}")"
[[ "${RUNTIME_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "host handoff runtime image ID is malformed"
[[ "${RUNTIME_IMAGE_SOURCE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "host handoff runtime source image ID is malformed"
[[ "$(docker image inspect "${RUNTIME_IMAGE_SOURCE_ID}" \
  --format '{{.Os}}/{{.Architecture}}' 2>/dev/null || true)" == linux/arm64 ]] || \
  Fail "host handoff source image is not available as linux/arm64"
python3 ./tools/verify_oci_image_archive.py \
  --archive "${HOST_HANDOFF}/runtime-image/mentor-pi-runtime.tar" \
  --image-id "${RUNTIME_IMAGE_ID}" --os linux --architecture arm64 >/dev/null || \
  Fail "host handoff OCI archive is not a valid linux/arm64 image"

if BeginStage runtime-smoke; then
  ./tools/test_host_runtime_image.sh --architecture arm64 \
    --image "${RUNTIME_IMAGE_SOURCE_ID}" \
    --host-prefix "${HOST_HANDOFF}/host" \
    --agent-prefix "${HOST_HANDOFF}/agent"
  runtime_output_sha="$(printf 'runtime-smoke:%s\n' "$(Sha256 "${HOST_HANDOFF}/SHA256SUMS")" | sha256sum | awk '{print $1}')"
  CompleteStage runtime-smoke "${runtime_output_sha}"
fi
runtime_output_sha="$(printf 'runtime-smoke:%s\n' "$(Sha256 "${HOST_HANDOFF}/SHA256SUMS")" | sha256sum | awk '{print $1}')"
VerifyStageChecksum runtime-smoke "${runtime_output_sha}"

current_stage=final-bundle
if StageDone final-bundle; then
  [[ -d "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
    Fail "completed final bundle is missing or symbolic"
  (cd "${OUTPUT_ROOT}" && sha256sum --check SHA256SUMS >/dev/null)
  VerifyStageChecksum final-bundle "$(Sha256 "${OUTPUT_ROOT}/SHA256SUMS")"
  completed_successfully=1
  rm -rf -- "${WORK_ROOT}"
  rm -f -- "${ACTIVE_FILE}"
  echo "Checksummed arm64 RDK handoff: ${OUTPUT_ROOT}"
  exit 0
fi
final_marker="$(StageMarker final-bundle)"
[[ ! -e "${final_marker}" && ! -L "${final_marker}" ]] || \
  Fail "final bundle checkpoint is malformed, symbolic, or tampered"
if [[ -d "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] && \
    (cd "${OUTPUT_ROOT}" && sha256sum --check SHA256SUMS >/dev/null); then
  echo "Adopting verified final bundle from the interrupted stage."
  CompleteStage final-bundle "$(Sha256 "${OUTPUT_ROOT}/SHA256SUMS")"
  completed_successfully=1
  rm -rf -- "${WORK_ROOT}"
  rm -f -- "${ACTIVE_FILE}"
  echo "Checksummed arm64 RDK handoff: ${OUTPUT_ROOT}"
  exit 0
fi
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "generated handoff output path already exists"
staging_root="$(mktemp -d "$(dirname "${OUTPUT_ROOT}")/.rrclite-rdk-handoff.XXXXXX")"
cp -a "${HOST_HANDOFF}" "${staging_root}/host-handoff"
cp -a "${BOARD_HANDOFF}" "${staging_root}/board-handoff"
cat >"${staging_root}/RDK-HANDOFF.txt" <<EOF
package_format=rrclite-rdk-handoff-v1
created_utc=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
release_id=${RELEASE_ID}
build_execution=qemu-emulated
build_host_architecture=amd64
target_architecture=arm64
native_target_validated=0
source_sha256=${SOURCE_SHA256}
image_stage_input_sha256=${IMAGE_STAGE_INPUT_SHA}
pid_stage_input_sha256=${PID_STAGE_INPUT_SHA}
agent_stage_input_sha256=${AGENT_STAGE_INPUT_SHA}
host_build_stage_input_sha256=${HOST_BUILD_STAGE_INPUT_SHA}
host_package_stage_input_sha256=${HOST_PACKAGE_STAGE_INPUT_SHA}
host_handoff_directory=host-handoff
board_handoff_directory=board-handoff
EOF
cat >"${staging_root}/INSTALL.txt" <<'EOF'
Follow docs/tutorials/01-prepare-ubuntu-development-host.md in host-handoff to
verify and transfer this complete bundle. On the native arm64 RDK X5, follow
Tutorial 02 to flash board-handoff/firmware-pid-release without rebuilding,
then Tutorial 03 to load the OCI image, install the Agent and host prefixes,
configure the CH9102F identity, verify systemd, and start production. Never
start the controller before the packaged PID flash succeeds with every
actuator disconnected.
EOF
(
  cd "${staging_root}"
  find host-handoff board-handoff RDK-HANDOFF.txt INSTALL.txt -type f -print | LC_ALL=C sort |
    while IFS= read -r file; do printf '%s  %s\n' "$(Sha256 "${file}")" "${file}"; done > SHA256SUMS
  sha256sum --check SHA256SUMS >/dev/null
)
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "handoff output appeared while staging"
mv "${staging_root}" "${OUTPUT_ROOT}"
staging_root=""
CompleteStage final-bundle "$(Sha256 "${OUTPUT_ROOT}/SHA256SUMS")"
completed_successfully=1
rm -rf -- "${WORK_ROOT}"
rm -f -- "${ACTIVE_FILE}"
echo "Checksummed arm64 RDK handoff: ${OUTPUT_ROOT}"
