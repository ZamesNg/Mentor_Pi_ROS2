#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly AGENT_SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"

host_prefix=""
output_directory=""
release_id=""
staging_root=""
validation_root=""

Usage() {
  cat >&2 <<'EOF'
Usage: package_host_handoff.sh --host-prefix ABSOLUTE_PATH
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
    --output-directory) output_directory="${2:-}"; shift 2 ;;
    --release-id) release_id="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${host_prefix}" == /* && -d "${host_prefix}" ]] || Usage
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
if [[ "${BUILDER_IMAGE}" != "native-ubuntu-22.04" &&
      ! "${BUILDER_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
  Fail "host build metadata has an unpinned builder identity"
fi
readonly RECORDED_SOURCE="$(ReadMetadataValue "${BUILD_METADATA}" source_sha256)"
readonly CURRENT_SOURCE="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
[[ "${RECORDED_SOURCE}" =~ ^[0-9a-f]{64}$ ]] ||
  Fail "host build source fingerprint is malformed"
[[ "${RECORDED_SOURCE}" == "${CURRENT_SOURCE}" ]] ||
  Fail "host release source ${RECORDED_SOURCE} does not match current ${CURRENT_SOURCE}"

readonly REQUIRED_EXECUTABLES=(
  configuration_supervisor
  qualification_campaign
  qualification_monitor
  motor_commissioning
  capture_board_diagnostics
  install_production_assets
  promote_host_release
  require_controller_target_inactive
  run_configuration_supervisor
)
for executable in "${REQUIRED_EXECUTABLES[@]}"; do
  [[ -x "${host_prefix}/lib/mentor_pi_bringup/${executable}" ]] ||
    Fail "host release is missing executable ${executable}"
done
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
cat >"${validation_root}/environment/humble-setup.bash" <<'EOF'
: "${AMENT_TRACE_SETUP_FILES:=}"
export ROS_DISTRO=humble
EOF
MENTOR_PI_DEPLOYMENT_TEST_ROOT="${validation_root}" \
MENTOR_PI_DEPLOYMENT_TEST_OS_RELEASE="${validation_root}/environment/os-release" \
MENTOR_PI_DEPLOYMENT_TEST_ARCHITECTURE="${ARCHITECTURE}" \
MENTOR_PI_DEPLOYMENT_TEST_ROS_SETUP="${validation_root}/environment/humble-setup.bash" \
  "${host_prefix}/lib/mentor_pi_bringup/promote_host_release" \
    --staged-prefix "${host_prefix}" --release-id package-layout-check \
    >/dev/null
rm -rf -- "${validation_root}"
validation_root=""

staging_root="$(mktemp -d "${output_parent}/.mentor-pi-host-handoff.XXXXXX")"
mkdir -p "${staging_root}/host" \
  "${staging_root}/agent-installer/tools" \
  "${staging_root}/agent-installer/tools/patches" \
  "${staging_root}/agent-installer/mentor_pi_ros2/src/mentor_pi_bringup/scripts" \
  "${staging_root}/docs/tutorials"
cp -a "${host_prefix}/." "${staging_root}/host/"
install -m 0755 \
  "${SCRIPT_DIR}/install_microros_agent.sh" \
  "${SCRIPT_DIR}/build_microros_agent_from_lock.sh" \
  "${SCRIPT_DIR}/require_microros_agent_install_idle.sh" \
  "${SCRIPT_DIR}/verify_microros_agent_install_state.sh" \
  "${staging_root}/agent-installer/tools/"
install -m 0644 "${AGENT_SOURCE_LOCK}" \
  "${staging_root}/agent-installer/tools/microros_agent_source.lock"
install -m 0644 \
  "${SCRIPT_DIR}/patches/micro_xrce_agent_rrclite_modem_lines.patch" \
  "${staging_root}/agent-installer/tools/patches/"
install -m 0755 \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/scripts/run_micro_ros_agent" \
  "${staging_root}/agent-installer/mentor_pi_ros2/src/mentor_pi_bringup/scripts/"
cp -a "${PROJECT_ROOT}/docs/tutorials/." \
  "${staging_root}/docs/tutorials/"

readonly CREATED_UTC="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
readonly PACKAGED_AGENT_SOURCE_LOCK="${staging_root}/agent-installer/tools/microros_agent_source.lock"
readonly AGENT_LOCK_SHA="$(Sha256 "${PACKAGED_AGENT_SOURCE_LOCK}")"
readonly PACKAGED_AGENT_PATCH="${staging_root}/agent-installer/tools/patches/micro_xrce_agent_rrclite_modem_lines.patch"
readonly AGENT_PATCH_SHA="$(Sha256 "${PACKAGED_AGENT_PATCH}")"
cat >"${staging_root}/AGENT-METADATA.txt" <<EOF
format=rrclite-agent-handoff-v2
ros_distro=humble
installation=pinned-source-build
source_lock=agent-installer/tools/microros_agent_source.lock
source_lock_sha256=${AGENT_LOCK_SHA}
rrclite_patch=agent-installer/tools/patches/micro_xrce_agent_rrclite_modem_lines.patch
rrclite_patch_sha256=${AGENT_PATCH_SHA}
installer=agent-installer/tools/install_microros_agent.sh
runtime_executable=/opt/mentor_pi/bin/mentor_pi_micro_ros_agent
EOF
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
source_sha256=${CURRENT_SOURCE}
builder_image=${BUILDER_IMAGE}
host_prefix_directory=host
agent_installer_directory=agent-installer
agent_metadata=AGENT-METADATA.txt
EOF
cat >"${staging_root}/INSTALL.txt" <<EOF
Verify from this directory:
  sha256sum --check SHA256SUMS

On Ubuntu 22.04 ${ARCHITECTURE}, with mentor-pi-controller.target inactive:
  sudo ./agent-installer/tools/install_microros_agent.sh
  sudo ./host/lib/mentor_pi_bringup/promote_host_release \\
    --staged-prefix "\${PWD}/host" --release-id ${release_id}

Connect exactly one CH9102F, identify its tty/serial or ID_PATH, then follow
docs/tutorials/onboard-computer/03-build-and-run-humble-host.md before installing
udev/systemd site assets. Do not enable the target before review.
EOF

readonly POST_STAGING_SOURCE="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
[[ "${POST_STAGING_SOURCE}" == "${CURRENT_SOURCE}" ]] ||
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
