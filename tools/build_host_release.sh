#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ENVIRONMENT_CHECK="${SCRIPT_DIR}/verify_host_build_environment.sh"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly RELOCATION_CHECK="${SCRIPT_DIR}/verify_host_release_relocation.sh"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"

project_root="${DEFAULT_PROJECT_ROOT}"
output_prefix=""
work_directory=""

Usage() {
  cat >&2 <<'EOF'
Usage: build_host_release.sh --output-prefix ABSOLUTE_PATH
  --work-directory ABSOLUTE_PATH [--project-root ABSOLUTE_PATH]
EOF
  exit 2
}

Fail() {
  echo "Host release build error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --output-prefix) output_prefix="${2:-}"; shift 2 ;;
    --work-directory) work_directory="${2:-}"; shift 2 ;;
    --project-root) project_root="${2:-}"; shift 2 ;;
    *) Usage ;;
  esac
done

[[ "${output_prefix}" == /* && "${output_prefix}" != "/" ]] || Usage
[[ "${work_directory}" == /* && "${work_directory}" != "/" ]] || Usage
[[ "${project_root}" == /* && -d "${project_root}" ]] || Usage
[[ "$(id -u)" != 0 ]] || Fail "build the host release as an unprivileged user"
for path in "${output_prefix}" "${work_directory}"; do
  [[ ! -e "${path}" && ! -L "${path}" ]] ||
    Fail "refusing to replace existing path: ${path}"
done
[[ -x "${ENVIRONMENT_CHECK}" && -x "${FINGERPRINT_TOOL}" &&
      -x "${RELOCATION_CHECK}" ]] ||
  Fail "host build verification tools are missing"
for package in mentor_pi_interfaces mentor_pi_bringup; do
  [[ -f "${project_root}/src/${package}/package.xml" ]] ||
    Fail "missing source package ${package}"
done
readonly INITIAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"

"${ENVIRONMENT_CHECK}" --check-tools yes

set +u
source "${ROS_SETUP}"
set -u
rosdep check --from-paths \
  "${project_root}/src/mentor_pi_interfaces" \
  "${project_root}/src/mentor_pi_bringup" \
  --ignore-src --rosdistro humble

mkdir -p "$(dirname "${output_prefix}")" "$(dirname "${work_directory}")"
mkdir "${work_directory}"
readonly LOG_ROOT="${work_directory}/log"
readonly BUILD_ROOT="${work_directory}/build"

colcon --log-base "${LOG_ROOT}" build \
  --merge-install \
  --base-paths \
    "${project_root}/src/mentor_pi_interfaces" \
    "${project_root}/src/mentor_pi_bringup" \
  --build-base "${BUILD_ROOT}" \
  --install-base "${output_prefix}" \
  --packages-up-to mentor_pi_bringup \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

set +u
source "${output_prefix}/setup.bash"
set -u
colcon --log-base "${LOG_ROOT}" test \
  --merge-install \
  --build-base "${BUILD_ROOT}" \
  --install-base "${output_prefix}" \
  --packages-select mentor_pi_interfaces mentor_pi_bringup \
  --event-handlers console_direct+
colcon test-result --test-result-base "${BUILD_ROOT}" --verbose

readonly POST_TEST_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"
[[ "${POST_TEST_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}" ]] ||
  Fail "project-owned host source changed during build or tests"
readonly ARCHITECTURE="$(dpkg --print-architecture)"
readonly CREATED_UTC="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
readonly COMPILER_VERSION="$(c++ --version | head -n 1 | tr ' ' '_')"
readonly BUILDER_IMAGE="${MENTOR_PI_HOST_BUILDER_IMAGE:-native-ubuntu-22.04}"
if [[ "${BUILDER_IMAGE}" != "native-ubuntu-22.04" &&
      ! "${BUILDER_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]]; then
  Fail "builder image must be a pinned digest or native-ubuntu-22.04"
fi
cat >"${output_prefix}/HOST-BUILD-METADATA.txt" <<EOF
format=rrclite-host-build-v2
ubuntu=22.04
target_os=ubuntu
target_version=22.04
architecture=${ARCHITECTURE}
ros_distro=humble
build_type=Release
source_sha256=${INITIAL_SOURCE_FINGERPRINT}
created_utc=${CREATED_UTC}
compiler=${COMPILER_VERSION}
builder_image=${BUILDER_IMAGE}
EOF

required_paths=(
  setup.bash
  lib/mentor_pi_bringup/configuration_supervisor
  lib/mentor_pi_bringup/qualification_campaign
  lib/mentor_pi_bringup/qualification_monitor
  lib/mentor_pi_bringup/motor_commissioning
  lib/mentor_pi_bringup/capture_board_diagnostics
  lib/mentor_pi_bringup/install_production_assets
  lib/mentor_pi_bringup/promote_host_release
  lib/mentor_pi_bringup/require_controller_target_inactive
  lib/mentor_pi_bringup/run_configuration_supervisor
  share/mentor_pi_bringup/config/controller.yaml
  share/mentor_pi_bringup/launch/controller.launch.xml
  share/mentor_pi_bringup/systemd/mentor-pi-agent.service
  share/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in
  share/ros_package_schema/package_common.xsd
  share/ros_package_schema/package_format3.xsd
)
for relative in "${required_paths[@]}"; do
  [[ -f "${output_prefix}/${relative}" ]] ||
    Fail "merged release is missing ${relative}"
done
first_link="$(find "${output_prefix}" -type l -print -quit)"
[[ -z "${first_link}" ]] || Fail "merged release contains symlink ${first_link}"

"${RELOCATION_CHECK}" --prefix "${output_prefix}" \
  --work-directory "${work_directory}"
readonly FINAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"
[[ "${FINAL_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}" ]] ||
  Fail "project-owned host source changed during relocation verification"

echo "Built and relocation-verified merged host release: ${output_prefix}"
