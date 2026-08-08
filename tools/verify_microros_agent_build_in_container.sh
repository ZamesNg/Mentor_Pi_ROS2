#!/usr/bin/env bash

set -euo pipefail

readonly PROJECT_ROOT=/project
readonly EVIDENCE_ROOT=/evidence
readonly ROS_SETUP=/opt/ros/humble/setup.bash
readonly SOURCE_LOCK="${PROJECT_ROOT}/tools/microros_agent_source.lock"
readonly XRCE_AGENT_PATCH="${PROJECT_ROOT}/tools/patches/micro_xrce_agent_rrclite_modem_lines.patch"
readonly STATE_VALIDATOR="${PROJECT_ROOT}/tools/verify_microros_agent_install_state.sh"
readonly AGENT_BUILD_HELPER="${PROJECT_ROOT}/tools/build_microros_agent_from_lock.sh"

Fail() {
  echo "micro-ROS Agent container build failed: $*" >&2
  exit 1
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

Sha256() {
  sha256sum "$1" | awk '{print $1}'
}

[[ "$#" -eq 0 ]] || Fail "this container entry point accepts no arguments"
: "${RRCLITE_AGENT_ARCH:?RRCLITE_AGENT_ARCH is required}"
: "${RRCLITE_AGENT_IMAGE_REF:?RRCLITE_AGENT_IMAGE_REF is required}"
: "${RRCLITE_AGENT_IMAGE_ID:?RRCLITE_AGENT_IMAGE_ID is required}"
: "${RRCLITE_AGENT_PHASE:?RRCLITE_AGENT_PHASE is required}"
: "${RRCLITE_AGENT_HOST_SOURCE_SHA:?RRCLITE_AGENT_HOST_SOURCE_SHA is required}"
[[ "${RRCLITE_AGENT_ARCH}" == "amd64" || \
   "${RRCLITE_AGENT_ARCH}" == "arm64" ]] || \
  Fail "unsupported architecture ${RRCLITE_AGENT_ARCH}"
[[ -d "${PROJECT_ROOT}" && -d "${EVIDENCE_ROOT}" ]] || \
  Fail "required project/evidence mounts are missing"
[[ "${RRCLITE_AGENT_PHASE}" == "fetch" || \
   "${RRCLITE_AGENT_PHASE}" == "build" ]] || \
  Fail "Agent verification phase must be fetch or build"
[[ "${RRCLITE_AGENT_HOST_SOURCE_SHA}" =~ ^[0-9a-f]{64}$ ]] || \
  Fail "host source fingerprint is malformed"
[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] || \
  Fail "Agent source lock is missing or symbolic"
[[ -f "${XRCE_AGENT_PATCH}" && ! -L "${XRCE_AGENT_PATCH}" ]] || \
  Fail "RRCLite Agent patch is missing or symbolic"
[[ -x "${STATE_VALIDATOR}" ]] || Fail "Agent state validator is unavailable"
[[ -x "${AGENT_BUILD_HELPER}" ]] || Fail "Agent build helper is unavailable"
[[ -r "${ROS_SETUP}" ]] || Fail "ROS 2 Humble setup is unavailable"

readonly LOCK_FORMAT="$(ReadSingleValue "${SOURCE_LOCK}" format)"
readonly LOCK_ROS_DISTRO="$(ReadSingleValue "${SOURCE_LOCK}" ros_distro)"
readonly AGENT_REPOSITORY="$(ReadSingleValue \
  "${SOURCE_LOCK}" agent_repository)"
readonly AGENT_COMMIT="$(ReadSingleValue "${SOURCE_LOCK}" agent_commit)"
readonly MSGS_REPOSITORY="$(ReadSingleValue \
  "${SOURCE_LOCK}" messages_repository)"
readonly MSGS_COMMIT="$(ReadSingleValue "${SOURCE_LOCK}" messages_commit)"
readonly XRCE_AGENT_REPOSITORY="$(ReadSingleValue \
  "${SOURCE_LOCK}" xrce_agent_repository)"
readonly XRCE_AGENT_COMMIT="$(ReadSingleValue \
  "${SOURCE_LOCK}" xrce_agent_commit)"
[[ "${LOCK_FORMAT}" == "mentor-pi-micro-ros-agent-source-lock-v2" ]] || \
  Fail "unsupported Agent source-lock schema"
[[ "${LOCK_ROS_DISTRO}" == "humble" ]] || \
  Fail "Agent source lock is not for Humble"
[[ "${AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ && \
   "${MSGS_COMMIT}" =~ ^[0-9a-f]{40}$ && \
   "${XRCE_AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ ]] || \
  Fail "Agent source lock contains a malformed commit"

set +u
source "${ROS_SETUP}"
set -u
[[ "${ROS_DISTRO:-}" == "humble" ]] || \
  Fail "ROS setup did not identify ROS_DISTRO=humble"
readonly CONTAINER_ARCH="$(dpkg --print-architecture)"
[[ "${CONTAINER_ARCH}" == "${RRCLITE_AGENT_ARCH}" ]] || \
  Fail "container architecture ${CONTAINER_ARCH} does not match request"
mkdir -p "${HOME}"

readonly BUILD_ROOT="${EVIDENCE_ROOT}/work"
readonly SOURCE_ROOT="${BUILD_ROOT}/src"
readonly INSTALL_ROOT="${BUILD_ROOT}/install"

ValidateSources() {
  "${STATE_VALIDATOR}" \
    --os-release /etc/os-release \
    --architecture "${CONTAINER_ARCH}" \
    --repository "${SOURCE_ROOT}/micro-ROS-Agent" \
    --origin "${AGENT_REPOSITORY}" \
    --commit "${AGENT_COMMIT}" \
    --repository "${SOURCE_ROOT}/micro_ros_msgs" \
    --origin "${MSGS_REPOSITORY}" \
    --commit "${MSGS_COMMIT}" \
    --repository "${SOURCE_ROOT}/Micro-XRCE-DDS-Agent" \
    --origin "${XRCE_AGENT_REPOSITORY}" \
    --commit "${XRCE_AGENT_COMMIT}"
}

if [[ "${RRCLITE_AGENT_PHASE}" == "fetch" ]]; then
  [[ ! -e "${BUILD_ROOT}" && ! -L "${BUILD_ROOT}" ]] || \
    Fail "Agent verification work directory already exists"
  "${AGENT_BUILD_HELPER}" fetch --work-root "${BUILD_ROOT}"
  ValidateSources
  printf '%s\n' "$(Sha256 "${SOURCE_LOCK}")" \
    >"${BUILD_ROOT}/source-lock.sha256"
  echo "Fetched and verified pinned Humble Agent sources (${CONTAINER_ARCH})."
  exit 0
fi

[[ -f "${BUILD_ROOT}/source-lock.sha256" && \
   ! -L "${BUILD_ROOT}/source-lock.sha256" ]] || \
  Fail "verified Agent source-fetch marker is missing"
grep -Fqx "$(Sha256 "${SOURCE_LOCK}")" \
  "${BUILD_ROOT}/source-lock.sha256" || \
  Fail "Agent source lock changed between fetch and offline build"
ValidateSources

# This is intentionally a check, not an install. The digest-pinned builder
# image must already contain every dependency needed by the pinned sources;
# the subsequent network-disabled configure/link is the authoritative check.
"${AGENT_BUILD_HELPER}" build --work-root "${BUILD_ROOT}" \
  --dependency-mode preinstalled

readonly AGENT_EXECUTABLE="${INSTALL_ROOT}/lib/micro_ros_agent/micro_ros_agent"
[[ -x "${AGENT_EXECUTABLE}" && ! -L "${AGENT_EXECUTABLE}" ]] || \
  Fail "native Agent executable was not produced"
[[ -r "${INSTALL_ROOT}/local_setup.bash" ]] || \
  Fail "Agent install environment was not produced"
set +u
source "${INSTALL_ROOT}/local_setup.bash"
set -u

smoke_status=0
set +e
"${AGENT_EXECUTABLE}" --help \
  >"${EVIDENCE_ROOT}/agent-help.txt" 2>&1
smoke_status=$?
set -e
[[ "${smoke_status}" == "0" || "${smoke_status}" == "1" ]] || \
  Fail "Agent loader smoke returned ${smoke_status}"
grep -Fq 'Usage:' "${EVIDENCE_ROOT}/agent-help.txt" || \
  Fail "Agent loader smoke did not emit its usage banner"

(
  cd "${INSTALL_ROOT}"
  find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum
) >"${EVIDENCE_ROOT}/install-tree-files.sha256"
while IFS= read -r -d '' link; do
  resolved="$(readlink -f "${link}" || true)"
  [[ -n "${resolved}" && -e "${resolved}" && \
     "${resolved}" == "${INSTALL_ROOT}/"* ]] || \
    Fail "generated install tree contains an external or dangling link: ${link}"
done < <(find "${INSTALL_ROOT}" -type l -print0)
(
  cd "${INSTALL_ROOT}"
  find . -type l -printf '%p -> %l\n' | LC_ALL=C sort
) >"${EVIDENCE_ROOT}/install-tree-symlinks.txt"
dpkg-query -W -f='${binary:Package}\t${Version}\t${Architecture}\n' | \
  LC_ALL=C sort >"${EVIDENCE_ROOT}/dpkg-packages.tsv"
ldd "${AGENT_EXECUTABLE}" >"${EVIDENCE_ROOT}/agent-ldd.txt"
readelf -h "${AGENT_EXECUTABLE}" >"${EVIDENCE_ROOT}/agent-elf-header.txt"
cp "${SOURCE_LOCK}" "${EVIDENCE_ROOT}/microros_agent_source.lock"
cp "${XRCE_AGENT_PATCH}" \
  "${EVIDENCE_ROOT}/micro_xrce_agent_rrclite_modem_lines.patch"

readonly INSTALL_MANIFEST_SHA="$(Sha256 \
  "${EVIDENCE_ROOT}/install-tree-files.sha256")"
readonly PACKAGE_MANIFEST_SHA="$(Sha256 \
  "${EVIDENCE_ROOT}/dpkg-packages.tsv")"
readonly SYMLINK_MANIFEST_SHA="$(Sha256 \
  "${EVIDENCE_ROOT}/install-tree-symlinks.txt")"
readonly SOURCE_LOCK_SHA="$(Sha256 "${SOURCE_LOCK}")"
readonly XRCE_AGENT_PATCH_SHA="$(Sha256 "${XRCE_AGENT_PATCH}")"
readonly EXECUTABLE_SHA="$(Sha256 "${AGENT_EXECUTABLE}")"
readonly VERIFIER_SHA="$(Sha256 \
  "${PROJECT_ROOT}/tools/verify_microros_agent_build_in_container.sh")"
readonly INSTALLER_SHA="$(Sha256 \
  "${PROJECT_ROOT}/tools/install_microros_agent.sh")"
readonly BUILD_HELPER_SHA="$(Sha256 "${AGENT_BUILD_HELPER}")"
readonly ORCHESTRATOR_SHA="$(Sha256 \
  "${PROJECT_ROOT}/tools/verify_microros_agent_build_container.sh")"
readonly WRAPPER_SHA="$(Sha256 \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/scripts/run_micro_ros_agent")"

cat >"${EVIDENCE_ROOT}/AGENT-BUILD-EVIDENCE.txt" <<EOF
format=rrclite-agent-build-evidence-v1
evidence_class=immutable-builder-compatibility
deployable_artifact=0
created_utc=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
ubuntu=22.04
ros_distro=humble
architecture=${CONTAINER_ARCH}
builder_image=${RRCLITE_AGENT_IMAGE_REF}
builder_image_id=${RRCLITE_AGENT_IMAGE_ID}
agent_repository=${AGENT_REPOSITORY}
agent_commit=${AGENT_COMMIT}
messages_repository=${MSGS_REPOSITORY}
messages_commit=${MSGS_COMMIT}
xrce_agent_repository=${XRCE_AGENT_REPOSITORY}
xrce_agent_commit=${XRCE_AGENT_COMMIT}
source_lock_sha256=${SOURCE_LOCK_SHA}
rrclite_patch_sha256=${XRCE_AGENT_PATCH_SHA}
host_source_sha256=${RRCLITE_AGENT_HOST_SOURCE_SHA}
agent_executable_sha256=${EXECUTABLE_SHA}
install_tree_manifest_sha256=${INSTALL_MANIFEST_SHA}
install_tree_symlink_manifest_sha256=${SYMLINK_MANIFEST_SHA}
dpkg_manifest_sha256=${PACKAGE_MANIFEST_SHA}
verification_script_sha256=${VERIFIER_SHA}
orchestrator_script_sha256=${ORCHESTRATOR_SHA}
production_installer_sha256=${INSTALLER_SHA}
shared_build_helper_sha256=${BUILD_HELPER_SHA}
runtime_wrapper_sha256=${WRAPPER_SHA}
loader_smoke_status=${smoke_status}
result=pass
EOF

echo "Pinned Humble micro-ROS Agent source build passed (${CONTAINER_ARCH})."
echo "Agent executable SHA-256: ${EXECUTABLE_SHA}"
