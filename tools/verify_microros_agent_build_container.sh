#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"
readonly EVIDENCE_PARENT="${PROJECT_ROOT}/build/microros-agent-verification"

architecture=""
evidence_id="$(date -u '+%Y%m%dT%H%M%SZ')"
dry_run=0
staging_root=""
publish_lock=""

Fail() {
  echo "micro-ROS Agent build verification failed: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: verify_microros_agent_build_container.sh \
  --architecture amd64|arm64 [--evidence-id SAFE_ID] [--dry-run]
       verify_microros_agent_build_container.sh --print-default-image \
         --architecture amd64|arm64
EOF
  exit 2
}

Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    Fail "neither sha256sum nor shasum is installed"
  fi
}

ReadSingleValue() {
  local file="$1"
  local key="$2"
  local count
  local line
  count="$(grep -Ec "^${key}=" "${file}" || true)"
  [[ "${count}" == "1" ]] || \
    Fail "${file} must contain exactly one ${key}= entry"
  line="$(grep -E "^${key}=" "${file}")"
  printf '%s' "${line#*=}"
}

Cleanup() {
  if [[ -n "${staging_root}" ]]; then
    case "${staging_root}" in
      "${EVIDENCE_PARENT}"/.agent-build.*)
        chmod -R u+rwX "${staging_root}" 2>/dev/null || true
        rm -rf -- "${staging_root}"
        ;;
      *) echo "Refusing unsafe Agent-evidence cleanup: ${staging_root}" >&2 ;;
    esac
  fi
  if [[ -n "${publish_lock}" ]]; then
    case "${publish_lock}" in
      "${EVIDENCE_PARENT}"/*.lock) rmdir "${publish_lock}" 2>/dev/null || true ;;
      *) echo "Refusing unsafe Agent-publication-lock cleanup: ${publish_lock}" >&2 ;;
    esac
  fi
}
trap Cleanup EXIT

if [[ "$#" -eq 3 && "$1" == "--print-default-image" && \
  "$2" == "--architecture" ]]; then
  "${IMAGE_SELECTOR}" microros "$3"
  exit 0
fi
while (($# > 0)); do
  case "$1" in
    --architecture)
      (($# >= 2)) || Usage
      architecture="$2"
      shift 2
      ;;
    --evidence-id)
      (($# >= 2)) || Usage
      evidence_id="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *) Usage ;;
  esac
done
[[ "${architecture}" == "amd64" || "${architecture}" == "arm64" ]] || \
  Fail "architecture must be amd64 or arm64"
[[ "${evidence_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] || \
  Fail "evidence ID must be 1-64 safe characters"
[[ -x "${IMAGE_SELECTOR}" ]] || Fail "pinned image selector is unavailable"
readonly DEFAULT_IMAGE="$("${IMAGE_SELECTOR}" microros "${architecture}")"
[[ "${DEFAULT_IMAGE}" =~ ^[^[:space:]@]+@sha256:[0-9a-f]{64}$ ]] || \
  Fail "default Agent builder is not digest-pinned"

readonly OUTPUT_ROOT="${EVIDENCE_PARENT}/${evidence_id}-${architecture}"
if [[ "${dry_run}" == "1" ]]; then
  printf 'image=%s\nplatform=linux/%s\noutput=%s\nnetwork=source-fetch-only\n' \
    "${DEFAULT_IMAGE}" "${architecture}" "${OUTPUT_ROOT}"
  exit 0
fi

command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker is not running or accessible"
[[ -x "${SCRIPT_DIR}/verify_microros_agent_build_in_container.sh" ]] || \
  Fail "container entry point is missing or not executable"
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "evidence output already exists: ${OUTPUT_ROOT}"

readonly IMAGE_ARCH="$(docker image inspect "${DEFAULT_IMAGE}" \
  --format '{{.Architecture}}' 2>/dev/null || true)"
readonly IMAGE_ID="$(docker image inspect "${DEFAULT_IMAGE}" \
  --format '{{.Id}}' 2>/dev/null || true)"
[[ "${IMAGE_ARCH}" == "${architecture}" ]] || \
  Fail "pinned builder for ${architecture} is not local; run make setup"
[[ "${IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "could not resolve the pinned builder image ID"

mkdir -p "${EVIDENCE_PARENT}"
lock_candidate="${OUTPUT_ROOT}.lock"
if ! mkdir "${lock_candidate}"; then
  Fail "Agent evidence ID is already being produced: ${evidence_id}-${architecture}"
fi
publish_lock="${lock_candidate}"
staging_root="$(mktemp -d "${EVIDENCE_PARENT}/.agent-build.XXXXXX")"
readonly STAGING_ROOT="${staging_root}"
readonly SOURCE_FINGERPRINT_BEFORE="$(
  "${SCRIPT_DIR}/host_source_fingerprint.sh" "${PROJECT_ROOT}"
)"

RunContainerPhase() {
  local phase="$1"
  local network="$2"
  local log="$3"
  local docker_status
  local tee_status
  local -a pipeline_status
  set +e
  docker run --rm \
    --platform "linux/${architecture}" \
    --network "${network}" \
    --user "$(id -u):$(id -g)" \
    --cap-drop ALL \
    --security-opt no-new-privileges \
    --env HOME=/tmp/rrclite-agent-home \
    --env RRCLITE_AGENT_ARCH="${architecture}" \
    --env RRCLITE_AGENT_IMAGE_REF="${DEFAULT_IMAGE}" \
    --env RRCLITE_AGENT_IMAGE_ID="${IMAGE_ID}" \
    --env RRCLITE_AGENT_PHASE="${phase}" \
    --env RRCLITE_AGENT_HOST_SOURCE_SHA="${SOURCE_FINGERPRINT_BEFORE}" \
    --volume "${PROJECT_ROOT}:/project:ro" \
    --volume "${STAGING_ROOT}:/evidence" \
    --entrypoint /bin/bash \
    "${DEFAULT_IMAGE}" \
    /project/tools/verify_microros_agent_build_in_container.sh \
    2>&1 | tee "${STAGING_ROOT}/${log}"
  pipeline_status=("${PIPESTATUS[@]}")
  docker_status="${pipeline_status[0]}"
  tee_status="${pipeline_status[1]}"
  set -e
  [[ "${docker_status}" == "0" && "${tee_status}" == "0" ]] || \
    Fail "Agent ${phase} phase failed (docker=${docker_status}, tee=${tee_status})"
}

RunContainerPhase fetch bridge fetch.log
RunContainerPhase build none build.log
[[ "${STAGING_ROOT}/work" == \
   "${EVIDENCE_PARENT}/.agent-build."*/work ]] || \
  Fail "refusing unsafe Agent work-directory cleanup"
rm -rf -- "${STAGING_ROOT}/work"

readonly SOURCE_FINGERPRINT_AFTER="$(
  "${SCRIPT_DIR}/host_source_fingerprint.sh" "${PROJECT_ROOT}"
)"
[[ "${SOURCE_FINGERPRINT_AFTER}" == "${SOURCE_FINGERPRINT_BEFORE}" ]] || \
  Fail "host/Agent sources changed during verification"

readonly -a EVIDENCE_FILES=(
  AGENT-BUILD-EVIDENCE.txt
  agent-elf-header.txt
  agent-help.txt
  agent-ldd.txt
  build.log
  dpkg-packages.tsv
  fetch.log
  install-tree-files.sha256
  install-tree-symlinks.txt
  micro_xrce_agent_rrclite_modem_lines.patch
  microros_agent_source.lock
)
: >"${STAGING_ROOT}/SHA256SUMS"
for evidence_file in "${EVIDENCE_FILES[@]}"; do
  [[ -f "${STAGING_ROOT}/${evidence_file}" && \
     ! -L "${STAGING_ROOT}/${evidence_file}" ]] || \
    Fail "Agent evidence is missing or symbolic: ${evidence_file}"
  if [[ "${evidence_file}" != "install-tree-symlinks.txt" ]]; then
    [[ -s "${STAGING_ROOT}/${evidence_file}" ]] || \
      Fail "Agent evidence is unexpectedly empty: ${evidence_file}"
  fi
  printf '%s  %s\n' "$(Sha256 "${STAGING_ROOT}/${evidence_file}")" \
    "${evidence_file}" >>"${STAGING_ROOT}/SHA256SUMS"
done
readonly METADATA="${STAGING_ROOT}/AGENT-BUILD-EVIDENCE.txt"
readonly CAPTURED_LOCK="${STAGING_ROOT}/microros_agent_source.lock"
readonly PROJECT_LOCK="${PROJECT_ROOT}/tools/microros_agent_source.lock"
readonly CAPTURED_PATCH="${STAGING_ROOT}/micro_xrce_agent_rrclite_modem_lines.patch"
readonly PROJECT_PATCH="${PROJECT_ROOT}/tools/patches/micro_xrce_agent_rrclite_modem_lines.patch"
RequireMetadataEquals() {
  local key="$1"
  local expected="$2"
  [[ "$(ReadSingleValue "${METADATA}" "${key}")" == "${expected}" ]] || \
    Fail "Agent evidence metadata mismatch for ${key}"
}
RequireMetadataSha() {
  local key="$1"
  local expected="$2"
  local value
  value="$(ReadSingleValue "${METADATA}" "${key}")"
  [[ "${value}" =~ ^[0-9a-f]{64}$ && "${value}" == "${expected}" ]] || \
    Fail "Agent evidence SHA mismatch for ${key}"
}

RequireMetadataEquals format rrclite-agent-build-evidence-v1
RequireMetadataEquals evidence_class immutable-builder-compatibility
RequireMetadataEquals deployable_artifact 0
RequireMetadataEquals ubuntu 22.04
RequireMetadataEquals ros_distro humble
RequireMetadataEquals architecture "${architecture}"
RequireMetadataEquals builder_image "${DEFAULT_IMAGE}"
RequireMetadataEquals builder_image_id "${IMAGE_ID}"
RequireMetadataEquals result pass
readonly AGENT_REPOSITORY="$(ReadSingleValue "${PROJECT_LOCK}" agent_repository)"
readonly AGENT_COMMIT="$(ReadSingleValue "${PROJECT_LOCK}" agent_commit)"
readonly MSGS_REPOSITORY="$(ReadSingleValue "${PROJECT_LOCK}" messages_repository)"
readonly MSGS_COMMIT="$(ReadSingleValue "${PROJECT_LOCK}" messages_commit)"
readonly XRCE_AGENT_REPOSITORY="$(ReadSingleValue \
  "${PROJECT_LOCK}" xrce_agent_repository)"
readonly XRCE_AGENT_COMMIT="$(ReadSingleValue \
  "${PROJECT_LOCK}" xrce_agent_commit)"
RequireMetadataEquals agent_repository "${AGENT_REPOSITORY}"
RequireMetadataEquals agent_commit "${AGENT_COMMIT}"
RequireMetadataEquals messages_repository "${MSGS_REPOSITORY}"
RequireMetadataEquals messages_commit "${MSGS_COMMIT}"
RequireMetadataEquals xrce_agent_repository "${XRCE_AGENT_REPOSITORY}"
RequireMetadataEquals xrce_agent_commit "${XRCE_AGENT_COMMIT}"
cmp "${CAPTURED_LOCK}" "${PROJECT_LOCK}" >/dev/null || \
  Fail "captured Agent source lock differs from the project lock"
cmp "${CAPTURED_PATCH}" "${PROJECT_PATCH}" >/dev/null || \
  Fail "captured RRCLite Agent patch differs from the project patch"
RequireMetadataSha source_lock_sha256 "$(Sha256 "${PROJECT_LOCK}")"
RequireMetadataSha rrclite_patch_sha256 "$(Sha256 "${PROJECT_PATCH}")"
RequireMetadataSha host_source_sha256 "${SOURCE_FINGERPRINT_BEFORE}"
RequireMetadataSha install_tree_manifest_sha256 \
  "$(Sha256 "${STAGING_ROOT}/install-tree-files.sha256")"
RequireMetadataSha install_tree_symlink_manifest_sha256 \
  "$(Sha256 "${STAGING_ROOT}/install-tree-symlinks.txt")"
RequireMetadataSha dpkg_manifest_sha256 \
  "$(Sha256 "${STAGING_ROOT}/dpkg-packages.tsv")"
RequireMetadataSha verification_script_sha256 \
  "$(Sha256 "${SCRIPT_DIR}/verify_microros_agent_build_in_container.sh")"
RequireMetadataSha orchestrator_script_sha256 \
  "$(Sha256 "${SCRIPT_DIR}/verify_microros_agent_build_container.sh")"
RequireMetadataSha production_installer_sha256 \
  "$(Sha256 "${SCRIPT_DIR}/install_microros_agent.sh")"
RequireMetadataSha shared_build_helper_sha256 \
  "$(Sha256 "${SCRIPT_DIR}/build_microros_agent_from_lock.sh")"
RequireMetadataSha runtime_wrapper_sha256 \
  "$(Sha256 "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/scripts/run_micro_ros_agent")"
readonly EXECUTABLE_MANIFEST_ROW_COUNT="$(grep -Ec \
  '  [*]?.[/]lib/micro_ros_agent/micro_ros_agent$' \
  "${STAGING_ROOT}/install-tree-files.sha256" || true)"
[[ "${EXECUTABLE_MANIFEST_ROW_COUNT}" == "1" ]] || \
  Fail "install manifest does not identify exactly one Agent executable"
readonly MANIFEST_EXECUTABLE_SHA="$(grep -E \
  '  [*]?.[/]lib/micro_ros_agent/micro_ros_agent$' \
  "${STAGING_ROOT}/install-tree-files.sha256" | awk '{print $1}')"
RequireMetadataSha agent_executable_sha256 "${MANIFEST_EXECUTABLE_SHA}"
readonly LOADER_STATUS="$(ReadSingleValue "${METADATA}" loader_smoke_status)"
[[ "${LOADER_STATUS}" == "0" || "${LOADER_STATUS}" == "1" ]] || \
  Fail "Agent loader smoke status is invalid"
case "${architecture}" in
  amd64)
    grep -Fq 'Machine:                           Advanced Micro Devices X86-64' \
      "${STAGING_ROOT}/agent-elf-header.txt" || \
      Fail "Agent ELF is not x86-64"
    ;;
  arm64)
    grep -Fq 'Machine:                           AArch64' \
      "${STAGING_ROOT}/agent-elf-header.txt" || \
      Fail "Agent ELF is not AArch64"
    ;;
esac
if command -v sha256sum >/dev/null 2>&1; then
  (cd "${STAGING_ROOT}" && sha256sum --check SHA256SUMS)
else
  (cd "${STAGING_ROOT}" && shasum -a 256 --check SHA256SUMS)
fi
readonly SOURCE_FINGERPRINT_FINAL="$(
  "${SCRIPT_DIR}/host_source_fingerprint.sh" "${PROJECT_ROOT}"
)"
[[ "${SOURCE_FINGERPRINT_FINAL}" == "${SOURCE_FINGERPRINT_BEFORE}" ]] || \
  Fail "host/Agent sources changed while evidence was verified"
[[ "$(docker image inspect "${DEFAULT_IMAGE}" --format '{{.Id}}')" == \
   "${IMAGE_ID}" ]] || Fail "Agent builder image identity changed during verification"

chmod -R a-w "${STAGING_ROOT}"
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] || \
  Fail "Agent evidence output appeared before publication"
mv "${STAGING_ROOT}" "${OUTPUT_ROOT}"
staging_root=""
rmdir "${publish_lock}"
publish_lock=""
echo "Verified Agent build evidence: ${OUTPUT_ROOT}"
