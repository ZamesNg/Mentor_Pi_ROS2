#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly AGENT_SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly ALTO_SOURCE_LOCK="${SCRIPT_DIR}/altro_source.lock"
readonly ALTO_CHECKOUT="${PROJECT_ROOT}/mentor_pi_ros2/third_party/altro-cpp"
readonly ALTO_PATCH="${SCRIPT_DIR}/patches/altro-disable-docs.patch"
readonly HOST_DEPENDENCY_BOOTSTRAP="${SCRIPT_DIR}/bootstrap_host_dependencies.sh"

host_prefix=""
agent_prefix=""
runtime_image_archive=""
runtime_image_id=""
runtime_image_source_id=""
output_directory=""
release_id=""
staging_root=""
validation_root=""

Usage() {
  cat >&2 <<'EOF'
Usage: package_host_handoff.sh --host-prefix ABSOLUTE_PATH
  --agent-prefix ABSOLUTE_PATH --runtime-image-archive ABSOLUTE_PATH
  --runtime-image-id sha256:HEX --runtime-image-source-id sha256:HEX
  --output-directory PATH --release-id SAFE_ID
EOF
  exit 2
}

Fail() {
  echo "Host-handoff packaging error: $*" >&2
  exit 1
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

ReadMetadataValue() {
  local metadata="$1"
  local key="$2"
  local line=""
  line="$(grep -E "^${key}=" "${metadata}" || true)"
  [[ -n "${line}" && "${line}" != *$'\n'* ]] ||
    Fail "${metadata} must contain exactly one ${key}= entry"
  printf '%s' "${line#*=}"
}

ValidateRelativeSymlinks() {
  local root="$1"
  local link=""
  local target=""
  local resolved=""
  while IFS= read -r -d '' link; do
    target="$(readlink -- "${link}")"
    [[ -n "${target}" && "${target}" != /* && \
       "${target}" != *$'\n'* && "${target}" != *$'\t'* ]] || \
      Fail "handoff symlink has an unsafe target: ${link}"
    resolved="$(realpath -m "$(dirname "${link}")/${target}")"
    case "${resolved}" in
      "${root}"/*) ;;
      *) Fail "handoff symlink escapes its prefix: ${link}" ;;
    esac
    [[ -e "${resolved}" ]] || Fail "handoff symlink is dangling: ${link}"
  done < <(find "${root}" -type l -print0)
}

Cleanup() {
  if [[ -n "${validation_root}" && -d "${validation_root}" ]]; then
    rm -rf -- "${validation_root}"
  fi
  if [[ -n "${staging_root}" && -d "${staging_root}" ]]; then
    rm -rf -- "${staging_root}"
  fi
}
trap Cleanup EXIT

while (($# > 0)); do
  case "$1" in
    --host-prefix) host_prefix="${2:-}"; shift 2 ;;
    --agent-prefix) agent_prefix="${2:-}"; shift 2 ;;
    --runtime-image-archive) runtime_image_archive="${2:-}"; shift 2 ;;
    --runtime-image-id) runtime_image_id="${2:-}"; shift 2 ;;
    --runtime-image-source-id) runtime_image_source_id="${2:-}"; shift 2 ;;
    --output-directory) output_directory="${2:-}"; shift 2 ;;
    --release-id) release_id="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${host_prefix}" == /* && -d "${host_prefix}" ]] || Usage
[[ "${agent_prefix}" == /* && -d "${agent_prefix}" ]] || Usage
[[ "${runtime_image_archive}" == /* && -s "${runtime_image_archive}" ]] || Usage
[[ "${runtime_image_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || Usage
[[ "${runtime_image_source_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || Usage
[[ -n "${output_directory}" ]] || Usage
[[ "${release_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]] ||
  Fail "release ID must contain 1-64 safe filename characters"
[[ -x "${FINGERPRINT_TOOL}" ]] || Fail "host fingerprint tool is missing"
[[ -f "${AGENT_SOURCE_LOCK}" && ! -L "${AGENT_SOURCE_LOCK}" ]] ||
  Fail "Agent source lock is missing or symbolic"
[[ "$(ReadMetadataValue "${AGENT_SOURCE_LOCK}" format)" == \
    "mentor-pi-micro-ros-agent-source-lock-v2" ]] ||
  Fail "unsupported Agent source lock metadata"
[[ "$(ReadMetadataValue "${AGENT_SOURCE_LOCK}" ros_distro)" == "humble" ]] ||
  Fail "Agent source lock targets the wrong ROS distribution"
[[ -f "${ALTO_SOURCE_LOCK}" && ! -L "${ALTO_SOURCE_LOCK}" ]] || \
  Fail "ALTO source lock is missing or symbolic"
if grep -Ev '^[A-Z0-9_]+=[A-Za-z0-9./:+_-]+$|^$' "${ALTO_SOURCE_LOCK}" | grep -q .; then
  Fail "ALTO source lock contains unsupported syntax"
fi
# shellcheck disable=SC1090
source "${ALTO_SOURCE_LOCK}"
[[ "${ALTO_REPOSITORY}" == https://github.com/ZamesNg/altro-cpp && \
   "${ALTO_COMMIT}" =~ ^[0-9a-f]{40}$ && \
   "${ALTO_LICENSE}" == GPL-2.0-or-later ]] || \
  Fail "ALTO source lock is invalid"
"${HOST_DEPENDENCY_BOOTSTRAP}" --verify-existing >/dev/null
[[ -f "${ALTO_PATCH}" && ! -L "${ALTO_PATCH}" ]] || \
  Fail "ALTO compatibility patch is missing or symbolic"

readonly BUILD_METADATA="${host_prefix}/HOST-BUILD-METADATA.txt"
[[ -f "${BUILD_METADATA}" && ! -L "${BUILD_METADATA}" ]] ||
  Fail "host build metadata is missing"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" format)" == \
    "rrclite-host-build-v2" ]] || Fail "unsupported host build metadata"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" ubuntu)" == "22.04" ]] ||
  Fail "host release has the wrong Ubuntu identity"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" target_os)" == "ubuntu" ]] ||
  Fail "host release targets the wrong OS"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" target_version)" == "22.04" ]] ||
  Fail "host release targets the wrong Ubuntu version"
readonly ARCHITECTURE="$(ReadMetadataValue "${BUILD_METADATA}" architecture)"
[[ "${ARCHITECTURE}" == "amd64" || "${ARCHITECTURE}" == "arm64" ]] ||
  Fail "host release has unsupported architecture ${ARCHITECTURE}"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" ros_distro)" == "humble" ]] ||
  Fail "host release targets the wrong ROS distribution"
[[ "$(ReadMetadataValue "${BUILD_METADATA}" build_type)" == "Release" ]] ||
  Fail "host release is not a Release build"
readonly BUILDER_IMAGE="$(ReadMetadataValue "${BUILD_METADATA}" builder_image)"
if [[ ! "${BUILDER_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
  Fail "host build metadata has an unpinned builder identity"
fi
readonly BUILDER_IMAGE_ID="$(ReadMetadataValue \
  "${BUILD_METADATA}" builder_image_id)"
[[ "${BUILDER_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "host builder image ID is not content-addressed"
readonly RECORDED_SOURCE="$(ReadMetadataValue "${BUILD_METADATA}" source_sha256)"
readonly CURRENT_BUILD_SOURCE="$(${FINGERPRINT_TOOL} --compile "${PROJECT_ROOT}")"
readonly CURRENT_PACKAGE_SOURCE="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
[[ "${RECORDED_SOURCE}" =~ ^[0-9a-f]{64}$ ]] ||
  Fail "host build source fingerprint is malformed"
[[ "${RECORDED_SOURCE}" == "${CURRENT_BUILD_SOURCE}" ]] ||
  Fail "host release source ${RECORDED_SOURCE} does not match current ${CURRENT_BUILD_SOURCE}"

readonly REQUIRED_EXECUTABLES=(
  configuration_supervisor
  qualification_campaign
  qualification_monitor
  motor_commissioning
  capture_board_diagnostics
  install_production_assets
  promote_host_release
  require_controller_target_inactive
  run_production_container
)
for executable in "${REQUIRED_EXECUTABLES[@]}"; do
  [[ -x "${host_prefix}/lib/mentor_pi_bringup/${executable}" ]] ||
    Fail "host release is missing executable ${executable}"
done
for tracker in mecanum_mpc_tracker ackermann_mpc_tracker; do
  [[ -x "${host_prefix}/lib/mentor_pi_tracking/${tracker}" ]] || \
    Fail "host release is missing tracking executable ${tracker}"
done
[[ -f "${host_prefix}/share/mentor_pi_tracking/licenses/ALTO-GPL-2.0.txt" ]] || \
  Fail "host release is missing the ALTO license"
readonly AGENT_METADATA="${agent_prefix}/AGENT-BUILD-METADATA.txt"
readonly AGENT_EXECUTABLE="${agent_prefix}/lib/micro_ros_agent/micro_ros_agent"
[[ -f "${AGENT_METADATA}" && ! -L "${AGENT_METADATA}" && \
   -x "${AGENT_EXECUTABLE}" && ! -L "${AGENT_EXECUTABLE}" ]] || \
  Fail "verified Agent release is incomplete"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" architecture)" == \
  "${ARCHITECTURE}" ]] || Fail "Agent architecture differs from the host release"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" format)" == \
  "rrclite-adaptive-agent-v1" ]] || Fail "unsupported Agent build metadata"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" ros_distro)" == humble ]] || \
  Fail "Agent targets the wrong ROS distribution"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" source_lock_sha256)" == \
  "$(Sha256 "${AGENT_SOURCE_LOCK}")" ]] || Fail "Agent source lock is stale"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" builder_identity)" =~ \
  ^sha256:[0-9a-f]{64}$ ]] || Fail "Agent builder identity is not content-addressed"
[[ "$(ReadMetadataValue "${AGENT_METADATA}" executable_sha256)" == \
  "$(Sha256 "${AGENT_EXECUTABLE}")" ]] || Fail "Agent executable hash is stale"
ValidateRelativeSymlinks "${agent_prefix}"
first_link="$(find "${host_prefix}" -type l -print -quit)"
[[ -z "${first_link}" ]] || Fail "host release contains symlink ${first_link}"

if [[ "${output_directory}" == /* ]]; then
  output_candidate="${output_directory}"
else
  output_candidate="${PROJECT_ROOT}/${output_directory}"
fi
mkdir -p "$(dirname "${output_candidate}")"
output_parent="$(cd "$(dirname "${output_candidate}")" && pwd -P)"
output_name="$(basename "${output_candidate}")"
[[ "${output_name}" != "." && "${output_name}" != ".." ]] ||
  Fail "output directory must name a new child"
readonly OUTPUT_ROOT="${output_parent}/${output_name}"
[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] ||
  Fail "refusing to replace existing output ${OUTPUT_ROOT}"

validation_root="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-host-layout.XXXXXX")"
mkdir -p "${validation_root}/environment"
cat >"${validation_root}/environment/os-release" <<'EOF'
ID=ubuntu
VERSION_ID="22.04"
EOF
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${validation_root}" \
MENTOR_PI_DEPLOYMENT_TEST_OS_RELEASE="${validation_root}/environment/os-release" \
MENTOR_PI_DEPLOYMENT_TEST_ARCHITECTURE="${ARCHITECTURE}" \
  "${host_prefix}/lib/mentor_pi_bringup/promote_host_release" \
    --staged-prefix "${host_prefix}" --release-id package-layout-check \
    >/dev/null
rm -rf -- "${validation_root}"
validation_root=""

staging_root="$(mktemp -d "${output_parent}/.mentor-pi-host-handoff.XXXXXX")"
mkdir -p "${staging_root}/host" \
  "${staging_root}/agent" \
  "${staging_root}/runtime-image" \
  "${staging_root}/docs/tutorials" \
  "${staging_root}/corresponding-source/mentor_pi/mentor_pi_ros2/src" \
  "${staging_root}/corresponding-source/mentor_pi/mentor_pi_ros2/third_party/altro-cpp" \
  "${staging_root}/corresponding-source/mentor_pi/tools/patches"
cp -a "${host_prefix}/." "${staging_root}/host/"
cp -a "${agent_prefix}/." "${staging_root}/agent/"
install -m 0644 "${runtime_image_archive}" \
  "${staging_root}/runtime-image/mentor-pi-runtime.tar"
cp -a "${PROJECT_ROOT}/docs/tutorials/." \
  "${staging_root}/docs/tutorials/"
git -C "${ALTO_CHECKOUT}" archive "${ALTO_COMMIT}" | \
  tar -xf - -C \
    "${staging_root}/corresponding-source/mentor_pi/mentor_pi_ros2/third_party/altro-cpp"
cp -a "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces" \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_tracking" \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_tracking_interfaces" \
  "${staging_root}/corresponding-source/mentor_pi/mentor_pi_ros2/src/"
install -m 0644 "${ALTO_PATCH}" \
  "${staging_root}/corresponding-source/mentor_pi/tools/patches/altro-disable-docs.patch"
install -m 0644 "${ALTO_SOURCE_LOCK}" \
  "${staging_root}/corresponding-source/mentor_pi/tools/altro_source.lock"
cat >"${staging_root}/corresponding-source/mentor_pi/BUILD.txt" <<'EOF'
This directory is the build-shaped corresponding source for the GPL-linked
Mentor Pi tracker. In an Ubuntu 22.04/ROS 2 Humble environment with the package
dependencies installed, build it without network access using:

  source /opt/ros/humble/setup.bash
  colcon build --base-paths mentor_pi_ros2/src \
    --packages-up-to mentor_pi_tracking \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

The pinned ALTO source, integration patch, source lock, project interfaces, and
all CMake/package controls required by that command are included here.
EOF
cat >"${staging_root}/THIRD-PARTY-NOTICES.txt" <<EOF
ALTO C++ trajectory optimization library
Source: ${ALTO_REPOSITORY}
Commit: ${ALTO_COMMIT}
License: GNU GPL version 2 or later
Corresponding and project integration source: corresponding-source/mentor_pi
Compatibility patch: corresponding-source/mentor_pi/tools/patches/altro-disable-docs.patch
Compatibility patch SHA-256: $(Sha256 "${ALTO_PATCH}")
Source lock: corresponding-source/mentor_pi/tools/altro_source.lock
EOF

symlink_manifest="${staging_root}/SYMLINKS.txt"
: >"${symlink_manifest}"
while IFS= read -r -d '' link; do
  relative="${link#"${staging_root}/"}"
  target="$(readlink -- "${link}")"
  printf '%s\t%s\n' "${relative}" "${target}" >>"${symlink_manifest}"
done < <(find "${staging_root}/host" "${staging_root}/agent" \
  -type l -print0 | LC_ALL=C sort -z)

readonly CREATED_UTC="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
readonly RUNTIME_ARCHIVE_SHA="$(Sha256 \
  "${staging_root}/runtime-image/mentor-pi-runtime.tar")"
cat >"${staging_root}/HOST-HANDOFF.txt" <<EOF
package_format=rrclite-host-handoff-v2
release_id=${release_id}
created_utc=${CREATED_UTC}
ubuntu=22.04
target_os=ubuntu
target_version=22.04
architecture=${ARCHITECTURE}
ros_distro=humble
build_type=Release
source_sha256=${CURRENT_PACKAGE_SOURCE}
builder_image=${BUILDER_IMAGE}
builder_image_id=${BUILDER_IMAGE_ID}
host_prefix_directory=host
agent_prefix_directory=agent
runtime_image_archive=runtime-image/mentor-pi-runtime.tar
runtime_image_archive_format=oci-v1
runtime_image_archive_sha256=${RUNTIME_ARCHIVE_SHA}
runtime_image_id=${runtime_image_id}
runtime_image_source_id=${runtime_image_source_id}
altro_repository=${ALTO_REPOSITORY}
altro_commit=${ALTO_COMMIT}
altro_source_lock_sha256=$(Sha256 "${ALTO_SOURCE_LOCK}")
altro_patch_sha256=$(Sha256 "${ALTO_PATCH}")
corresponding_source_directory=corresponding-source
symlink_manifest=SYMLINKS.txt
EOF
cat >"${staging_root}/INSTALL.txt" <<EOF
Verify from this directory:
  sha256sum --check SHA256SUMS
  while IFS=$'\t' read -r path target; do
    test -L "\${path}" && test "\$(readlink -- "\${path}")" = "\${target}"
  done < SYMLINKS.txt

On Ubuntu 22.04 ${ARCHITECTURE}, with mentor-pi-controller.target inactive:
  docker load --input runtime-image/mentor-pi-runtime.tar
  sudo install -d /opt/mentor_pi/releases/agent/${release_id}
  sudo cp -a agent/. /opt/mentor_pi/releases/agent/${release_id}/
  agent_sha=\$(sed -n 's/^executable_sha256=//p' agent/AGENT-BUILD-METADATA.txt)
  echo "\${agent_sha}  /opt/mentor_pi/releases/agent/${release_id}/lib/micro_ros_agent/micro_ros_agent" | sudo sha256sum --check --strict -
  sudo ln -sfn /opt/mentor_pi/releases/agent/${release_id} /opt/mentor_pi/micro_ros_agent
  sudo ./host/lib/mentor_pi_bringup/promote_host_release \\
    --staged-prefix "\${PWD}/host" --release-id ${release_id}

Confirm the loaded image platform:
  test "\$(docker image inspect ${runtime_image_id} --format '{{.Id}}')" = ${runtime_image_id}
  test "\$(docker image inspect ${runtime_image_id} --format '{{.Os}}/{{.Architecture}}')" = linux/${ARCHITECTURE}

Connect exactly one 1a86:55d4 CH9102F, identify its unique serial or ID_PATH
(not a changing tty number), then follow docs/tutorials/03-build-and-run-humble-host.md
to run install_production_assets with --runtime-image ${runtime_image_id}.
Run systemd-analyze verify for both units. Do not enable or start the controller
until the exact packaged PID firmware is flashed; afterward use
systemctl enable --now mentor-pi-controller.target and inspect the target,
runtime service, and journal.
EOF

readonly POST_STAGING_SOURCE="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
[[ "${POST_STAGING_SOURCE}" == "${CURRENT_PACKAGE_SOURCE}" ]] ||
  Fail "project-owned host source changed while staging the handoff"

manifest="${staging_root}/SHA256SUMS"
: >"${manifest}"
while IFS= read -r file; do
  relative="${file#"${staging_root}/"}"
  printf '%s  %s\n' "$(Sha256 "${file}")" "${relative}" >>"${manifest}"
done < <(find "${staging_root}" -type f ! -name SHA256SUMS -print | LC_ALL=C sort)

if command -v sha256sum >/dev/null 2>&1; then
  (cd "${staging_root}" && sha256sum --check SHA256SUMS >/dev/null)
else
  (cd "${staging_root}" && shasum -a 256 --check SHA256SUMS >/dev/null)
fi

[[ ! -e "${OUTPUT_ROOT}" && ! -L "${OUTPUT_ROOT}" ]] ||
  Fail "output appeared while packaging: ${OUTPUT_ROOT}"
mv "${staging_root}" "${OUTPUT_ROOT}"
staging_root=""
trap - EXIT

echo "Checksummed Mentor Pi host handoff: ${OUTPUT_ROOT}"
