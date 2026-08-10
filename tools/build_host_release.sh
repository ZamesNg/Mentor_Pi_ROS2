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
skip_tests=0
resume=0

Usage() {
  cat >&2 <<'EOF'
Usage: build_host_release.sh --output-prefix ABSOLUTE_PATH
  --work-directory ABSOLUTE_PATH [--project-root ABSOLUTE_PATH] [--skip-tests]
  [--resume]
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
    --skip-tests) skip_tests=1; shift ;;
    --resume) resume=1; shift ;;
    *) Usage ;;
  esac
done

[[ "${output_prefix}" == /* && "${output_prefix}" != "/" ]] || Usage
[[ "${work_directory}" == /* && "${work_directory}" != "/" ]] || Usage
[[ "${project_root}" == /* && -d "${project_root}" ]] || Usage
[[ "$(id -u)" != 0 ]] || Fail "build the host release as an unprivileged user"
[[ "${output_prefix}" != *$'\n'* && "${work_directory}" != *$'\n'* ]] || \
  Fail "build paths must not contain newlines"
[[ -x "${ENVIRONMENT_CHECK}" && -x "${FINGERPRINT_TOOL}" &&
      -x "${RELOCATION_CHECK}" ]] ||
  Fail "host build verification tools are missing"
for package in mentor_pi_interfaces mentor_pi_bringup mentor_pi_hardwares \
    mentor_pi_tracking_interfaces mentor_pi_tracking; do
  [[ -f "${project_root}/mentor_pi_ros2/src/${package}/package.xml" ]] ||
    Fail "missing source package ${package}"
done
readonly INITIAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} --compile "${project_root}")"
readonly ARCHITECTURE="$(dpkg --print-architecture)"
readonly BUILDER_IMAGE="${MENTOR_PI_HOST_BUILDER_IMAGE:-}"
readonly BUILDER_IMAGE_ID="${MENTOR_PI_HOST_BUILDER_IMAGE_ID:-}"
readonly RESUME_METADATA="${work_directory}/.rrclite-host-resume"
readonly QEMU_DEFERRED_TEST="configuration_supervisor_launch_test"
[[ "${BUILDER_IMAGE}" =~ @sha256:[0-9a-f]{64}$ ]] || \
  Fail "builder image must be pinned by a sha256 manifest digest"
[[ "${BUILDER_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] || \
  Fail "builder image ID must be content-addressed"

VerifyResumeMetadata() {
  [[ -f "${RESUME_METADATA}" && ! -L "${RESUME_METADATA}" ]] || \
    Fail "host resume metadata is missing or symbolic"
  [[ "$(awk 'END {print NR}' "${RESUME_METADATA}")" == 7 ]] || \
    Fail "host resume metadata has an unexpected field count"
  for expected in \
      'format=rrclite-host-resume-v1' \
      "architecture=${ARCHITECTURE}" \
      "builder_image=${BUILDER_IMAGE}" \
      "builder_image_id=${BUILDER_IMAGE_ID}" \
      "output_prefix=${output_prefix}"; do
    key="${expected%%=*}"
    [[ "$(awk -F= -v key="${key}" '$1 == key {count++} END {print count + 0}' \
      "${RESUME_METADATA}")" == 1 ]] || \
      Fail "host resume metadata field ${key} is missing or duplicated"
    grep -Fqx "${expected}" "${RESUME_METADATA}" || \
      Fail "host resume metadata differs: ${expected}"
  done
  recorded_source="$(sed -n 's/^source_sha256=//p' "${RESUME_METADATA}")"
  [[ "$(awk -F= '$1 == "source_sha256" {count++} END {print count + 0}' \
      "${RESUME_METADATA}")" == 1 && \
     "${recorded_source}" =~ ^[0-9a-f]{64}$ ]] || \
    Fail "host resume source fingerprint is malformed or duplicated"
  if [[ "${recorded_source}" != "${INITIAL_SOURCE_FINGERPRINT}" ]]; then
    sed "s/^source_sha256=.*/source_sha256=${INITIAL_SOURCE_FINGERPRINT}/" \
      "${RESUME_METADATA}" >"${RESUME_METADATA}.tmp"
    mv "${RESUME_METADATA}.tmp" "${RESUME_METADATA}"
    echo "Refreshing incremental host build for changed host sources."
  fi
  recorded_jobs="$(sed -n 's/^build_jobs=//p' "${RESUME_METADATA}")"
  [[ "$(awk -F= '$1 == "build_jobs" {count++} END {print count + 0}' \
      "${RESUME_METADATA}")" == 1 && "${recorded_jobs}" =~ ^[1-9][0-9]*$ ]] || \
    Fail "host resume build-job count is malformed or duplicated"
  if [[ "${recorded_jobs}" != "${RRCLITE_BUILD_JOBS:-1}" ]]; then
    sed "s/^build_jobs=.*/build_jobs=${RRCLITE_BUILD_JOBS:-1}/" \
      "${RESUME_METADATA}" >"${RESUME_METADATA}.tmp"
    mv "${RESUME_METADATA}.tmp" "${RESUME_METADATA}"
    echo "Updating incremental host package workers to ${RRCLITE_BUILD_JOBS:-1}."
  fi
}

mkdir -p "$(dirname "${output_prefix}")" "$(dirname "${work_directory}")"
if ((resume == 1)); then
  [[ -d "${work_directory}" && ! -L "${work_directory}" ]] || \
    Fail "resumed host build requires an existing regular work directory"
  [[ (! -e "${output_prefix}" && ! -L "${output_prefix}") || \
     (-d "${output_prefix}" && ! -L "${output_prefix}") ]] || \
    Fail "resumed host prefix is not absent or a regular directory"
  VerifyResumeMetadata
  echo "Resuming matching incremental host build: ${work_directory}"
else
  for path in "${output_prefix}" "${work_directory}"; do
    [[ ! -e "${path}" && ! -L "${path}" ]] || \
      Fail "refusing to replace existing path: ${path}"
  done
  mkdir "${work_directory}"
  cat >"${RESUME_METADATA}.tmp" <<EOF
format=rrclite-host-resume-v1
source_sha256=${INITIAL_SOURCE_FINGERPRINT}
architecture=${ARCHITECTURE}
builder_image=${BUILDER_IMAGE}
builder_image_id=${BUILDER_IMAGE_ID}
build_jobs=${RRCLITE_BUILD_JOBS:-1}
output_prefix=${output_prefix}
EOF
  mv "${RESUME_METADATA}.tmp" "${RESUME_METADATA}"
fi
completion_metadata="${output_prefix}/HOST-BUILD-COMPLETE.txt"
if [[ -e "${completion_metadata}" || -L "${completion_metadata}" ]]; then
  [[ -f "${completion_metadata}" && ! -L "${completion_metadata}" ]] || \
    Fail "host completion marker is not a regular file"
  rm -f -- "${completion_metadata}"
fi

"${ENVIRONMENT_CHECK}" --check-tools yes

set +u
source "${ROS_SETUP}"
set -u
rosdep check --from-paths \
  "${project_root}/mentor_pi_ros2/src" \
  --ignore-src --rosdistro humble

readonly LOG_ROOT="${work_directory}/log"
readonly BUILD_ROOT="${work_directory}/build"

if ((skip_tests == 1)); then
  readonly BUILD_TESTING=OFF
else
  readonly BUILD_TESTING=ON
fi

colcon --log-base "${LOG_ROOT}" build \
  --executor parallel \
  --parallel-workers "${RRCLITE_BUILD_JOBS:-1}" \
  --merge-install \
  --base-paths \
    "${project_root}/mentor_pi_ros2/src" \
  --build-base "${BUILD_ROOT}" \
  --install-base "${output_prefix}" \
  --packages-up-to mentor_pi_bringup mentor_pi_hardwares mentor_pi_tracking \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${BUILD_TESTING}"

if ((skip_tests == 0)); then
  set +u
  source "${output_prefix}/setup.bash"
  set -u
  test_arguments=()
  if [[ "${RRCLITE_QEMU_EMULATED_TESTS:-0}" == 1 ]]; then
    test_arguments+=(--ctest-args -E "^${QEMU_DEFERRED_TEST}$")
    echo "QEMU handoff excludes native-only test: ${QEMU_DEFERRED_TEST}"
  fi
  colcon --log-base "${LOG_ROOT}" test \
    --executor parallel \
    --parallel-workers "${RRCLITE_BUILD_JOBS:-1}" \
    --merge-install \
    --build-base "${BUILD_ROOT}" \
    --install-base "${output_prefix}" \
    --packages-select mentor_pi_interfaces mentor_pi_bringup \
      mentor_pi_hardwares mentor_pi_tracking_interfaces mentor_pi_tracking \
    --event-handlers console_direct+ \
    "${test_arguments[@]}"
  colcon test-result --test-result-base "${BUILD_ROOT}" --verbose
fi

readonly POST_TEST_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} --compile "${project_root}")"
[[ "${POST_TEST_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}" ]] ||
  Fail "project-owned host source changed during build or tests"
readonly CREATED_UTC="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
readonly COMPILER_VERSION="$(c++ --version | head -n 1 | tr ' ' '_')"
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
builder_image_id=${BUILDER_IMAGE_ID}
tests=$([[ "${skip_tests}" == 1 ]] && echo skipped || \
  { [[ "${RRCLITE_QEMU_EMULATED_TESTS:-0}" == 1 ]] && \
      echo qemu-passed-native-launch-deferred || echo passed; })
native_configuration_supervisor_launch_test=$(
  [[ "${skip_tests}" == 1 ]] && echo not-run || \
  { [[ "${RRCLITE_QEMU_EMULATED_TESTS:-0}" == 1 ]] && echo deferred || echo passed; })
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
  lib/mentor_pi_bringup/run_production_container
  share/mentor_pi_bringup/config/controller.yaml
  share/mentor_pi_bringup/launch/controller.launch.py
  share/mentor_pi_bringup/systemd/mentor-pi-runtime.service
  share/mentor_pi_bringup/systemd/mentor-pi-controller.target
  share/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in
  lib/libmentor_pi_hardwares.so
  share/mentor_pi_hardwares/ros2_control_plugins.xml
  share/mentor_pi_hardwares/launch/mecanum.launch.py
  share/mentor_pi_hardwares/launch/ackermann.launch.py
  share/mentor_pi_hardwares/launch/vehicle.launch.py
  share/mentor_pi_hardwares/config/mecanum/hardware.yaml
  share/mentor_pi_hardwares/config/ackermann/hardware.yaml
  share/mentor_pi_hardwares/config/mecanum/mentor_pi.urdf.xacro
  share/mentor_pi_hardwares/config/ackermann/mentor_pi.urdf.xacro
  lib/mentor_pi_tracking/mecanum_mpc_tracker
  lib/mentor_pi_tracking/ackermann_mpc_tracker
  lib/mentor_pi_tracking/polynomial_trajectory_publisher
  share/mentor_pi_tracking/licenses/ALTO-GPL-2.0.txt
)
for relative in "${required_paths[@]}"; do
  [[ -f "${output_prefix}/${relative}" ]] ||
    Fail "merged release is missing ${relative}"
done
first_link="$(find "${output_prefix}" -type l -print -quit)"
[[ -z "${first_link}" ]] || Fail "merged release contains symlink ${first_link}"

"${RELOCATION_CHECK}" --prefix "${output_prefix}" \
  --work-directory "${work_directory}"
readonly FINAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} --compile "${project_root}")"
[[ "${FINAL_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}" ]] ||
  Fail "project-owned host source changed during relocation verification"
cat >"${completion_metadata}.tmp" <<EOF
format=rrclite-host-build-complete-v1
source_sha256=${INITIAL_SOURCE_FINGERPRINT}
metadata_sha256=$(sha256sum "${output_prefix}/HOST-BUILD-METADATA.txt" | awk '{print $1}')
tests=$([[ "${skip_tests}" == 1 ]] && echo skipped || \
  { [[ "${RRCLITE_QEMU_EMULATED_TESTS:-0}" == 1 ]] && \
      echo qemu-passed-native-launch-deferred || echo passed; })
native_configuration_supervisor_launch_test=$(
  [[ "${skip_tests}" == 1 ]] && echo not-run || \
  { [[ "${RRCLITE_QEMU_EMULATED_TESTS:-0}" == 1 ]] && echo deferred || echo passed; })
EOF
mv "${completion_metadata}.tmp" "${completion_metadata}"

echo "Built and relocation-verified merged host release: ${output_prefix}"
