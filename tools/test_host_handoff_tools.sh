#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ENVIRONMENT_CHECK="${SCRIPT_DIR}/verify_host_build_environment.sh"
readonly RELOCATION_CHECK="${SCRIPT_DIR}/verify_host_release_relocation.sh"
readonly PACKAGE_TOOL="${SCRIPT_DIR}/package_host_handoff.sh"
readonly FINGERPRINT_TOOL="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-host-tools.XXXXXX")"

Cleanup() {
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Host handoff tool test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

VerifyManifest() {
  local directory="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum --check SHA256SUMS >/dev/null)
  else
    (cd "${directory}" && shasum -a 256 --check SHA256SUMS >/dev/null)
  fi
}

readonly PLATFORM_ROOT="${TEST_ROOT}/platform"
mkdir -p "${PLATFORM_ROOT}/etc" "${PLATFORM_ROOT}/usr/lib" \
  "${PLATFORM_ROOT}/fake-bin"
cat >"${PLATFORM_ROOT}/usr/lib/os-release" <<'EOF'
ID=ubuntu
VERSION_ID="24.04"
EOF
ln -s ../usr/lib/os-release "${PLATFORM_ROOT}/etc/os-release"
for tool in colcon rosdep cmake c++ python3 sha256sum tar gzip; do
  cat >"${PLATFORM_ROOT}/fake-bin/${tool}" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
  chmod +x "${PLATFORM_ROOT}/fake-bin/${tool}"
done
cat >"${PLATFORM_ROOT}/jazzy-setup.bash" <<EOF
export ROS_DISTRO=jazzy
export PATH="${PLATFORM_ROOT}/fake-bin:/usr/bin:/bin"
EOF

"${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/jazzy-setup.bash" \
  --check-tools yes >/dev/null
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib/missing-release "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/jazzy-setup.bash" \
  --check-tools no
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/jazzy-setup.bash" \
  --check-tools no
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib/os-release "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture riscv64 \
  --ros-setup "${PLATFORM_ROOT}/jazzy-setup.bash" \
  --check-tools no

MakeFakePrefix() {
  local prefix="$1"
  local hardcoded_setup="${2:-no}"
  mkdir -p \
    "${prefix}/lib/mentor_pi_bringup" \
    "${prefix}/lib" \
    "${prefix}/share/ament_index/resource_index/packages" \
    "${prefix}/share/mentor_pi_interfaces" \
    "${prefix}/share/ros_package_schema" \
    "${prefix}/share/mentor_pi_bringup/config" \
    "${prefix}/share/mentor_pi_bringup/launch" \
    "${prefix}/share/mentor_pi_bringup/systemd" \
    "${prefix}/share/mentor_pi_bringup/udev" \
    "${prefix}/test-bin"
  : >"${prefix}/share/ament_index/resource_index/packages/mentor_pi_bringup"
  : >"${prefix}/share/ament_index/resource_index/packages/mentor_pi_interfaces"
  cp "${PROJECT_ROOT}/src/mentor_pi_interfaces/package.xml" \
    "${prefix}/share/mentor_pi_interfaces/package.xml"
  cp "${PROJECT_ROOT}/src/ros_package_schema/package_common.xsd" \
    "${PROJECT_ROOT}/src/ros_package_schema/package_format3.xsd" \
    "${prefix}/share/ros_package_schema/"
  cp "${PROJECT_ROOT}/src/mentor_pi_bringup/config/controller.yaml" \
    "${prefix}/share/mentor_pi_bringup/config/controller.yaml"
  cp "${PROJECT_ROOT}/src/mentor_pi_bringup/launch/controller.launch.xml" \
    "${prefix}/share/mentor_pi_bringup/launch/controller.launch.xml"
  for asset in mentor-pi-configuration-supervisor.default \
      mentor-pi-runtime.service mentor-pi-agent.service \
      mentor-pi-configuration-supervisor.service mentor-pi-controller.target; do
    cp "${PROJECT_ROOT}/src/mentor_pi_bringup/systemd/${asset}" \
      "${prefix}/share/mentor_pi_bringup/systemd/${asset}"
  done
  cp "${PROJECT_ROOT}/src/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in" \
    "${prefix}/share/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in"
  cp "${PROJECT_ROOT}/src/mentor_pi_bringup/systemd/promote_host_release" \
    "${PROJECT_ROOT}/src/mentor_pi_bringup/systemd/require_controller_target_inactive" \
    "${prefix}/lib/mentor_pi_bringup/"
  for executable in configuration_supervisor qualification_campaign \
      qualification_monitor motor_commissioning capture_board_diagnostics \
      install_production_assets run_configuration_supervisor; do
    cat >"${prefix}/lib/mentor_pi_bringup/${executable}" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
  done
  chmod +x "${prefix}/lib/mentor_pi_bringup/"*
  for library in libmentor_pi_interfaces__rosidl_generator_c.so \
      libmentor_pi_interfaces__rosidl_typesupport_cpp.so \
      libmentor_pi_interfaces__rosidl_typesupport_fastrtps_cpp.so; do
    printf 'fixture\n' >"${prefix}/lib/${library}"
  done
  cat >"${prefix}/test-bin/ros2" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" == pkg && "${2:-}" == prefix ]]; then
  printf '%s\n' "${prefix}"
  exit 0
fi
if [[ "${1:-}" == interface && "${2:-}" == show ]]; then
  exit 0
fi
exit 2
EOF
  cat >"${prefix}/test-bin/ldd" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ -f "${1:-}" ]]
echo 'libfixture.so => /usr/lib/libfixture.so'
EOF
  chmod +x "${prefix}/test-bin/ros2" "${prefix}/test-bin/ldd"
  if [[ "${hardcoded_setup}" == yes ]]; then
    cat >"${prefix}/setup.bash" <<EOF
export AMENT_PREFIX_PATH="${prefix}"
export PATH="${prefix}/test-bin:/usr/bin:/bin"
EOF
  else
    cat >"${prefix}/setup.bash" <<'EOF'
_mentor_pi_fixture_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export AMENT_PREFIX_PATH="${_mentor_pi_fixture_prefix}"
export PATH="${_mentor_pi_fixture_prefix}/test-bin:/usr/bin:/bin"
unset _mentor_pi_fixture_prefix
EOF
  fi
  local source_fingerprint
  source_fingerprint="$(${FINGERPRINT_TOOL} "${PROJECT_ROOT}")"
  cat >"${prefix}/HOST-BUILD-METADATA.txt" <<EOF
format=rrclite-host-build-v1
target_os=ubuntu
target_version=24.04
architecture=arm64
ros_distro=jazzy
build_type=Release
source_sha256=${source_fingerprint}
created_utc=1970-01-01T00:00:00Z
compiler=fixture
builder_image=native-ubuntu-24.04
EOF
}

readonly GOOD_PREFIX="${TEST_ROOT}/good-prefix"
MakeFakePrefix "${GOOD_PREFIX}"
mkdir "${TEST_ROOT}/relocation-work"
"${RELOCATION_CHECK}" --prefix "${GOOD_PREFIX}" \
  --work-directory "${TEST_ROOT}/relocation-work"
[[ -d "${GOOD_PREFIX}" ]] || Fail "relocation check did not restore staging prefix"
[[ -z "$(find "${TEST_ROOT}" -maxdepth 1 -name '*.relocation-hidden.*' -print)" ]] ||
  Fail "relocation check left a hidden prefix"

readonly BROKEN_PREFIX="${TEST_ROOT}/broken-prefix"
MakeFakePrefix "${BROKEN_PREFIX}" yes
mkdir "${TEST_ROOT}/broken-relocation-work"
ExpectFailure "${RELOCATION_CHECK}" --prefix "${BROKEN_PREFIX}" \
  --work-directory "${TEST_ROOT}/broken-relocation-work"
[[ -d "${BROKEN_PREFIX}" ]] ||
  Fail "failed relocation check did not restore staging prefix"

readonly HANDOFF="${TEST_ROOT}/host-handoff"
"${PACKAGE_TOOL}" --host-prefix "${GOOD_PREFIX}" \
  --output-directory "${HANDOFF}" --release-id test-arm64 >/dev/null
[[ -f "${HANDOFF}/HOST-HANDOFF.txt" ]] || Fail "handoff metadata is missing"
[[ -f "${HANDOFF}/AGENT-METADATA.txt" ]] || Fail "Agent metadata is missing"
[[ -x "${HANDOFF}/host/lib/mentor_pi_bringup/capture_board_diagnostics" ]] ||
  Fail "current diagnostic executable is missing from handoff"
[[ -x "${HANDOFF}/host/lib/mentor_pi_bringup/qualification_campaign" ]] ||
  Fail "qualification campaign executable is missing from handoff"
[[ -x "${HANDOFF}/host/lib/mentor_pi_bringup/require_controller_target_inactive" ]] ||
  Fail "current idle guard is missing from handoff"
[[ -f "${HANDOFF}/agent-installer/tools/microros_agent_source.lock" ]] ||
  Fail "Agent source lock is missing from handoff"
[[ -f "${HANDOFF}/docs/host-preparation-and-handoff.md" ]] ||
  Fail "host preparation guide is missing from handoff"
[[ -f "${HANDOFF}/docs/board-arrival-bringup-checklist.md" ]] ||
  Fail "board-arrival checklist is missing from handoff"
VerifyManifest "${HANDOFF}"
grep -Fqx 'builder_image=native-ubuntu-24.04' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "builder provenance was not propagated"

initial_fingerprint_line="$(grep -n -F \
  'INITIAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
build_line="$(grep -n -F 'colcon --log-base "${LOG_ROOT}" build' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
post_test_line="$(grep -n -F \
  'POST_TEST_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
relocation_line="$(grep -n -F \
  '"${RELOCATION_CHECK}" --prefix "${output_prefix}"' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
final_fingerprint_line="$(grep -n -F \
  'FINAL_SOURCE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${project_root}")"' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
[[ -n "${initial_fingerprint_line}" && -n "${build_line}" &&
      -n "${post_test_line}" && -n "${relocation_line}" &&
      -n "${final_fingerprint_line}" ]] ||
  Fail "host build source-stability gates are missing"
((initial_fingerprint_line < build_line &&
  build_line < post_test_line &&
  post_test_line < relocation_line &&
  relocation_line < final_fingerprint_line)) ||
  Fail "host build source-stability gates are in the wrong order"
grep -Fq 'POST_TEST_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}' \
  "${SCRIPT_DIR}/build_host_release.sh" ||
  Fail "post-test source equality is not enforced"
test_command_line="$(grep -n -F 'colcon --log-base "${LOG_ROOT}" test' \
  "${SCRIPT_DIR}/build_host_release.sh" | cut -d: -f1)"
merge_test_line="$(awk -v start="${test_command_line}" \
  'NR > start && NR <= start + 4 && /--merge-install/ {print NR; exit}' \
  "${SCRIPT_DIR}/build_host_release.sh")"
[[ -n "${test_command_line}" && -n "${merge_test_line}" ]] ||
  Fail "merged host release test command omits --merge-install"
grep -Fq 'FINAL_SOURCE_FINGERPRINT}" == "${INITIAL_SOURCE_FINGERPRINT}' \
  "${SCRIPT_DIR}/build_host_release.sh" ||
  Fail "post-relocation source equality is not enforced"

readonly FINGERPRINT_PROJECT="${TEST_ROOT}/fingerprint-project"
mkdir -p "${FINGERPRINT_PROJECT}/src" "${FINGERPRINT_PROJECT}/tools" \
  "${FINGERPRINT_PROJECT}/docs"
cp -R "${PROJECT_ROOT}/src/mentor_pi_interfaces" \
  "${PROJECT_ROOT}/src/mentor_pi_bringup" \
  "${FINGERPRINT_PROJECT}/src/"
cp -R "${PROJECT_ROOT}/src/ros_package_schema" \
  "${FINGERPRINT_PROJECT}/src/"
readonly FINGERPRINT_STANDALONE_INPUTS=(
  docs/board-arrival-bringup-checklist.md
  docs/host-preparation-and-handoff.md
  tools/build_host_handoff_container.sh
  tools/build_host_release.sh
  tools/host_handoff_container_entrypoint.sh
  tools/host_source_fingerprint.sh
  tools/install_microros_agent.sh
  tools/microros_agent_source.lock
  tools/package_host_handoff.sh
  tools/prepare_host_build_dependencies.sh
  tools/require_microros_agent_install_idle.sh
  tools/verify_host_build_environment.sh
  tools/verify_host_release_relocation.sh
  tools/verify_microros_agent_install_state.sh
)
for relative in "${FINGERPRINT_STANDALONE_INPUTS[@]}"; do
  cp "${PROJECT_ROOT}/${relative}" "${FINGERPRINT_PROJECT}/${relative}"
done

readonly BASELINE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${FINGERPRINT_PROJECT}")"
readonly FINGERPRINT_SHARED_INPUTS=(
  src/ros_package_schema/README.md
  src/ros_package_schema/package_common.xsd
  src/ros_package_schema/package_format3.xsd
)
for relative in "${FINGERPRINT_SHARED_INPUTS[@]}"; do
  printf '\n<!-- host-fingerprint-mutation-fixture -->\n' \
    >>"${FINGERPRINT_PROJECT}/${relative}"
  mutated_fingerprint="$(${FINGERPRINT_TOOL} "${FINGERPRINT_PROJECT}")"
  [[ "${mutated_fingerprint}" != "${BASELINE_FINGERPRINT}" ]] ||
    Fail "host fingerprint omitted shared input ${relative}"
  cp "${PROJECT_ROOT}/${relative}" "${FINGERPRINT_PROJECT}/${relative}"
done
for relative in "${FINGERPRINT_STANDALONE_INPUTS[@]}"; do
  printf '\n# host-fingerprint-mutation-fixture\n' \
    >>"${FINGERPRINT_PROJECT}/${relative}"
  mutated_fingerprint="$(${FINGERPRINT_TOOL} "${FINGERPRINT_PROJECT}")"
  [[ "${mutated_fingerprint}" != "${BASELINE_FINGERPRINT}" ]] ||
    Fail "host fingerprint omitted standalone input ${relative}"
  cp "${PROJECT_ROOT}/${relative}" "${FINGERPRINT_PROJECT}/${relative}"
done

grep -Fq 'POST_STAGING_SOURCE}" == "${CURRENT_SOURCE}' \
  "${PACKAGE_TOOL}" ||
  Fail "handoff packaging does not enforce post-staging source stability"

cp "${GOOD_PREFIX}/HOST-BUILD-METADATA.txt" \
  "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid"
sed 's/^builder_image=.*/builder_image=ros:jazzy-ros-base/' \
  "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid" \
  >"${GOOD_PREFIX}/HOST-BUILD-METADATA.txt"
ExpectFailure "${PACKAGE_TOOL}" --host-prefix "${GOOD_PREFIX}" \
  --output-directory "${TEST_ROOT}/bad-builder" --release-id bad-builder
mv "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid" \
  "${GOOD_PREFIX}/HOST-BUILD-METADATA.txt"

bash -n "${ENVIRONMENT_CHECK}" "${RELOCATION_CHECK}" "${PACKAGE_TOOL}"
echo "Host build, relocation, and handoff tool tests passed."
