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
VERSION_ID="22.04"
EOF
ln -s ../usr/lib/os-release "${PLATFORM_ROOT}/etc/os-release"
for tool in colcon rosdep cmake c++ python3 sha256sum tar gzip; do
  cat >"${PLATFORM_ROOT}/fake-bin/${tool}" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
  chmod +x "${PLATFORM_ROOT}/fake-bin/${tool}"
done
cat >"${PLATFORM_ROOT}/humble-setup.bash" <<EOF
export ROS_DISTRO=humble
export PATH="${PLATFORM_ROOT}/fake-bin:/usr/bin:/bin"
EOF

"${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/humble-setup.bash" \
  --check-tools yes >/dev/null
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib/missing-release "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/humble-setup.bash" \
  --check-tools no
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture arm64 \
  --ros-setup "${PLATFORM_ROOT}/humble-setup.bash" \
  --check-tools no
rm "${PLATFORM_ROOT}/etc/os-release"
ln -s ../usr/lib/os-release "${PLATFORM_ROOT}/etc/os-release"
ExpectFailure "${ENVIRONMENT_CHECK}" \
  --os-release "${PLATFORM_ROOT}/etc/os-release" \
  --architecture riscv64 \
  --ros-setup "${PLATFORM_ROOT}/humble-setup.bash" \
  --check-tools no

MakeFakePrefix() {
  local prefix="$1"
  local hardcoded_setup="${2:-no}"
  mkdir -p \
    "${prefix}/lib/mentor_pi_bringup" \
    "${prefix}/lib" \
    "${prefix}/share/ament_index/resource_index/packages" \
    "${prefix}/share/mentor_pi_interfaces" \
    "${prefix}/share/mentor_pi_bringup/config" \
    "${prefix}/share/mentor_pi_bringup/launch" \
    "${prefix}/share/mentor_pi_bringup/systemd" \
    "${prefix}/share/mentor_pi_bringup/udev" \
    "${prefix}/test-bin"
  : >"${prefix}/share/ament_index/resource_index/packages/mentor_pi_bringup"
  : >"${prefix}/share/ament_index/resource_index/packages/mentor_pi_interfaces"
  cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces/package.xml" \
    "${prefix}/share/mentor_pi_interfaces/package.xml"
  cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/config/controller.yaml" \
    "${prefix}/share/mentor_pi_bringup/config/controller.yaml"
  cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/launch/controller.launch.py" \
    "${prefix}/share/mentor_pi_bringup/launch/controller.launch.py"
  for asset in mentor-pi-configuration-supervisor.default \
      mentor-pi-runtime.service mentor-pi-agent.service \
      mentor-pi-configuration-supervisor.service mentor-pi-controller.target; do
    cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/systemd/${asset}" \
      "${prefix}/share/mentor_pi_bringup/systemd/${asset}"
  done
  cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in" \
    "${prefix}/share/mentor_pi_bringup/udev/99-mentor-pi-mcu.rules.in"
  cp "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/systemd/promote_host_release" \
    "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup/systemd/require_controller_target_inactive" \
    "${prefix}/lib/mentor_pi_bringup/"
  for executable in configuration_supervisor qualification_campaign \
      qualification_monitor motor_commissioning \
      capture_board_diagnostics \
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
  printf 'fixture\n' >"${prefix}/lib/libmentor_pi_hardwares.so"
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
format=rrclite-host-build-v2
ubuntu=22.04
target_os=ubuntu
target_version=22.04
architecture=arm64
ros_distro=humble
build_type=Release
source_sha256=${source_fingerprint}
created_utc=1970-01-01T00:00:00Z
compiler=fixture
builder_image=native-ubuntu-22.04
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
[[ -f "${HANDOFF}/agent-installer/tools/patches/micro_xrce_agent_rrclite_modem_lines.patch" ]] ||
  Fail "RRCLite Agent modem-line patch is missing from handoff"
[[ -x "${HANDOFF}/agent-installer/tools/build_microros_agent_from_lock.sh" ]] ||
  Fail "shared Agent source-build helper is missing from handoff"
readonly PACKAGED_TUTORIALS=(
  01-prepare-ubuntu-development-host.md
  02-build-and-flash-default-pid-firmware.md
  03-build-and-run-humble-host.md
  04-run-passive-board-bringup.md
  05-characterize-board-hardware.md
  06-ros2-cli-hardware-checkout.md
  07-run-stress-soak-and-release-gates.md
  08-run-mentor-pi-hardwares.md
)
for tutorial in "${PACKAGED_TUTORIALS[@]}"; do
  [[ -f "${HANDOFF}/docs/tutorials/${tutorial}" ]] ||
    Fail "numbered tutorial is missing from handoff: ${tutorial}"
done
grep -Fq 'docs/tutorials/03-build-and-run-humble-host.md' \
  "${HANDOFF}/INSTALL.txt" ||
  Fail "handoff installation text does not point to Tutorial 03"
VerifyManifest "${HANDOFF}"
grep -Fqx 'package_format=rrclite-host-handoff-v2' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "handoff schema was not upgraded"
grep -Fqx 'ubuntu=22.04' "${HANDOFF}/host/HOST-BUILD-METADATA.txt" ||
  Fail "host build metadata does not record the exact Ubuntu identity"
grep -Fqx 'ubuntu=22.04' "${HANDOFF}/HOST-HANDOFF.txt" ||
  Fail "handoff does not record the exact Ubuntu identity"
grep -Fqx 'target_os=ubuntu' "${HANDOFF}/HOST-HANDOFF.txt" ||
  Fail "handoff targets the wrong OS"
grep -Fqx 'target_version=22.04' "${HANDOFF}/HOST-HANDOFF.txt" ||
  Fail "handoff targets the wrong Ubuntu version"
grep -Fqx 'architecture=arm64' "${HANDOFF}/HOST-HANDOFF.txt" ||
  Fail "handoff did not propagate the release architecture"
grep -Fqx 'ros_distro=humble' "${HANDOFF}/HOST-HANDOFF.txt" ||
  Fail "handoff does not bind ROS 2 Humble"
grep -Fqx 'format=rrclite-agent-handoff-v2' \
  "${HANDOFF}/AGENT-METADATA.txt" || Fail "Agent schema was not upgraded"
grep -Fqx 'ros_distro=humble' "${HANDOFF}/AGENT-METADATA.txt" ||
  Fail "Agent metadata does not bind ROS 2 Humble"
grep -Eq '^rrclite_patch_sha256=[0-9a-f]{64}$' \
  "${HANDOFF}/AGENT-METADATA.txt" ||
  Fail "Agent metadata does not bind the RRCLite modem-line patch"
grep -Fqx 'builder_image=native-ubuntu-22.04' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "builder provenance was not propagated"
grep -Fqx 'ubuntu=22.04' "${SCRIPT_DIR}/build_host_release.sh" ||
  Fail "host build metadata producer omits the exact Ubuntu identity"

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
mkdir -p "${FINGERPRINT_PROJECT}/mentor_pi_ros2/src" \
  "${FINGERPRINT_PROJECT}/tools/patches" \
  "${FINGERPRINT_PROJECT}/tools/docker" \
  "${FINGERPRINT_PROJECT}/docs/tutorials"
cp -R "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_interfaces" \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup" \
  "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_hardwares" \
  "${FINGERPRINT_PROJECT}/mentor_pi_ros2/src/"
cp -R "${PROJECT_ROOT}/docs/tutorials/." \
  "${FINGERPRINT_PROJECT}/docs/tutorials/"
readonly FINGERPRINT_STANDALONE_INPUTS=(
  Makefile
  tools/build_agent.sh
  tools/build_host_handoff_container.sh
  tools/build_microros_agent_from_lock.sh
  tools/build_host.sh
  tools/build_host_runtime_image.sh
  tools/build_host_release.sh
  tools/host_build_container_entrypoint.sh
  tools/host_handoff_container_entrypoint.sh
  tools/host_source_fingerprint.sh
  tools/docker/host-runtime.Dockerfile
  tools/install_microros_agent.sh
  tools/microros_agent_source.lock
  tools/open_runtime_shell.sh
  tools/package_host_handoff.sh
  tools/patches/micro_xrce_agent_rrclite_modem_lines.patch
  tools/prepare_host_build_dependencies.sh
  tools/require_microros_agent_install_idle.sh
  tools/run_runtime.sh
  tools/select_pinned_build_image.sh
  tools/verify_host_build_environment.sh
  tools/verify_host_release_relocation.sh
  tools/verify_microros_agent_build_container.sh
  tools/verify_microros_agent_build_in_container.sh
  tools/test_active_build_policy.sh
  tools/test_ros_workspace_layout.sh
  tools/verify_microros_agent_install_state.sh
)
for relative in "${FINGERPRINT_STANDALONE_INPUTS[@]}"; do
  cp "${PROJECT_ROOT}/${relative}" "${FINGERPRINT_PROJECT}/${relative}"
done

readonly BASELINE_FINGERPRINT="$(${FINGERPRINT_TOOL} "${FINGERPRINT_PROJECT}")"
readonly FINGERPRINT_SHARED_INPUTS=(
  mentor_pi_ros2/src/mentor_pi_interfaces/package.xml
  mentor_pi_ros2/src/mentor_pi_bringup/package.xml
  mentor_pi_ros2/src/mentor_pi_hardwares/package.xml
  docs/tutorials/01-prepare-ubuntu-development-host.md
  docs/tutorials/02-build-and-flash-default-pid-firmware.md
  docs/tutorials/03-build-and-run-humble-host.md
  docs/tutorials/04-run-passive-board-bringup.md
  docs/tutorials/05-characterize-board-hardware.md
  docs/tutorials/06-ros2-cli-hardware-checkout.md
  docs/tutorials/07-run-stress-soak-and-release-gates.md
  docs/tutorials/08-run-mentor-pi-hardwares.md
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

ExpectMetadataMutationFailure() {
  local mutation="$1"
  local output_name="$2"
  cp "${GOOD_PREFIX}/HOST-BUILD-METADATA.txt" \
    "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid"
  sed "${mutation}" "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid" \
    >"${GOOD_PREFIX}/HOST-BUILD-METADATA.txt"
  ExpectFailure "${PACKAGE_TOOL}" --host-prefix "${GOOD_PREFIX}" \
    --output-directory "${TEST_ROOT}/${output_name}" \
    --release-id "${output_name}"
  mv "${GOOD_PREFIX}/HOST-BUILD-METADATA.valid" \
    "${GOOD_PREFIX}/HOST-BUILD-METADATA.txt"
}

ExpectMetadataMutationFailure \
  's/^builder_image=.*/builder_image=ros:humble-ros-base/' bad-builder
ExpectMetadataMutationFailure \
  's/^format=.*/format=rrclite-host-build-v1/' legacy-schema
ExpectMetadataMutationFailure '/^ubuntu=/d' missing-ubuntu
ExpectMetadataMutationFailure 's/^ubuntu=.*/ubuntu=24.04/' wrong-ubuntu
ExpectMetadataMutationFailure 's/^target_os=.*/target_os=debian/' wrong-os
ExpectMetadataMutationFailure \
  's/^target_version=.*/target_version=24.04/' wrong-version
ExpectMetadataMutationFailure \
  's/^architecture=.*/architecture=riscv64/' wrong-architecture
ExpectMetadataMutationFailure \
  's/^ros_distro=.*/ros_distro=rolling/' wrong-distro

for architecture in amd64 arm64; do
  default_host_image="$(
    "${SCRIPT_DIR}/build_host_handoff_container.sh" --print-default-image \
      --architecture "${architecture}"
  )"
  [[ "${default_host_image}" =~ ^mentor-pi/rrclite-host-runtime:humble-${architecture}-[0-9a-f]{16}$ ]] ||
    Fail "default ${architecture} host runtime image is not content-addressed"
done
grep -Fq 'if [[ -z "${image}" ]]' \
  "${SCRIPT_DIR}/build_host_handoff_container.sh" ||
  Fail "the host wrapper no longer preserves an explicit --image override"
grep -Fq '"${IMAGE_ARCH}" == "${architecture}"' \
  "${SCRIPT_DIR}/build_host_handoff_container.sh" ||
  Fail "the host wrapper does not verify the local image architecture"
grep -Fq 'org.mentor-pi.host-runtime.base' \
  "${SCRIPT_DIR}/build_host_handoff_container.sh" ||
  Fail "the host wrapper does not verify the runtime image base"

bash -n "${ENVIRONMENT_CHECK}" "${RELOCATION_CHECK}" "${PACKAGE_TOOL}" \
  "${SCRIPT_DIR}/build_host_handoff_container.sh"
echo "Host build, relocation, and handoff tool tests passed."
