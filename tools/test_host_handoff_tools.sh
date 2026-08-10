#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly PACKAGER="${SCRIPT_DIR}/package_host_handoff.sh"
readonly FINGERPRINT="${SCRIPT_DIR}/host_source_fingerprint.sh"
readonly OCI_EXPORTER="${SCRIPT_DIR}/export_oci_image_archive.sh"
readonly TEST_ROOT="$(mktemp -d)"
readonly IMAGE_ID="sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
readonly IMAGE_SOURCE_ID="sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"

Cleanup() {
  [[ ! -d "${TEST_ROOT}" ]] || rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Host handoff contract test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output=""
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "expected '${expected}' in failure: ${output}"
}

readonly HOST_PREFIX="${TEST_ROOT}/host"
readonly AGENT_PREFIX="${TEST_ROOT}/agent"
readonly IMAGE_ARCHIVE="${TEST_ROOT}/runtime.tar"
readonly EXPORT_FIXTURE_ARCHIVE="${TEST_ROOT}/docker-save.tar"
readonly EXPORT_FIXTURE_OUTPUT="${TEST_ROOT}/exported-runtime.tar"
readonly FAKE_DOCKER_DIRECTORY="${TEST_ROOT}/fake-bin"
mkdir -p "${FAKE_DOCKER_DIRECTORY}"
readonly EXPORT_FIXTURE_IMAGE_ID="$(python3 - "${EXPORT_FIXTURE_ARCHIVE}" <<'PY'
import hashlib
import io
import json
import sys
import tarfile

archive_path = sys.argv[1]
config = json.dumps({"architecture": "amd64", "os": "linux"}).encode()
digest = hashlib.sha256(config).hexdigest()
manifest = json.dumps([{
    "Config": f"{digest}.json",
    "Layers": ["fixture/layer.tar"],
    "RepoTags": ["fixture:latest"],
}]).encode()
with tarfile.open(archive_path, "w") as archive:
    for name, data in (("manifest.json", manifest),
                       (f"{digest}.json", config),
                       ("fixture/layer.tar", b"layer")):
        info = tarfile.TarInfo(name)
        info.size = len(data)
        archive.addfile(info, io.BytesIO(data))
print(f"sha256:{digest}")
PY
)"
cat >"${FAKE_DOCKER_DIRECTORY}/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
if [[ "$1" == image && "$2" == inspect ]]; then
  case "$5" in
    '{{.Id}}') printf '%s\n' "${EXPORT_FIXTURE_IMAGE_ID}" ;;
    '{{.Os}}') printf '%s\n' linux ;;
    '{{.Architecture}}') printf '%s\n' amd64 ;;
    '{{index .RepoTags 0}}') printf '%s\n' fixture:latest ;;
    *) exit 2 ;;
  esac
elif [[ "$1" == save && "$2" == --output ]]; then
  cp "${EXPORT_FIXTURE_ARCHIVE}" "$3"
else
  exit 2
fi
EOF
chmod +x "${FAKE_DOCKER_DIRECTORY}/docker"
normal_export_output="$(PATH="${FAKE_DOCKER_DIRECTORY}:${PATH}" env \
  EXPORT_FIXTURE_ARCHIVE="${EXPORT_FIXTURE_ARCHIVE}" \
  EXPORT_FIXTURE_IMAGE_ID="${EXPORT_FIXTURE_IMAGE_ID}" \
  "${OCI_EXPORTER}" fixture:latest "${EXPORT_FIXTURE_OUTPUT}")"
[[ "${normal_export_output}" == \
  "Exported OCI runtime image ${EXPORT_FIXTURE_IMAGE_ID} (source ${EXPORT_FIXTURE_IMAGE_ID}): ${EXPORT_FIXTURE_OUTPUT}" ]] || \
  Fail "OCI image exporter normal output does not remain informational"
rm -f -- "${EXPORT_FIXTURE_OUTPUT}"
machine_export_output="$(PATH="${FAKE_DOCKER_DIRECTORY}:${PATH}" env \
  EXPORT_FIXTURE_ARCHIVE="${EXPORT_FIXTURE_ARCHIVE}" \
  EXPORT_FIXTURE_IMAGE_ID="${EXPORT_FIXTURE_IMAGE_ID}" \
  "${OCI_EXPORTER}" --print-runtime-id fixture:latest "${EXPORT_FIXTURE_OUTPUT}")"
[[ "${machine_export_output}" == "${EXPORT_FIXTURE_IMAGE_ID}" ]] || \
  Fail "OCI image exporter machine output is not exactly one runtime ID"
tar -tf "${EXPORT_FIXTURE_OUTPUT}" oci-layout index.json >/dev/null || \
  Fail "OCI image exporter does not retain direct archive member checks"
mkdir -p "${HOST_PREFIX}/lib/mentor_pi_bringup" \
  "${HOST_PREFIX}/lib/mentor_pi_tracking" \
  "${HOST_PREFIX}/share/mentor_pi_tracking/licenses" \
  "${AGENT_PREFIX}/lib/micro_ros_agent"

for executable in configuration_supervisor qualification_campaign \
    qualification_monitor motor_commissioning capture_board_diagnostics \
    install_production_assets promote_host_release \
    require_controller_target_inactive run_production_container; do
  printf '%s\n' '#!/usr/bin/env bash' 'exit 0' \
    >"${HOST_PREFIX}/lib/mentor_pi_bringup/${executable}"
  chmod +x "${HOST_PREFIX}/lib/mentor_pi_bringup/${executable}"
done
for tracker in mecanum_mpc_tracker ackermann_mpc_tracker; do
  printf '%s\n' '#!/usr/bin/env bash' 'exit 0' \
    >"${HOST_PREFIX}/lib/mentor_pi_tracking/${tracker}"
  chmod +x "${HOST_PREFIX}/lib/mentor_pi_tracking/${tracker}"
done
printf 'GNU GENERAL PUBLIC LICENSE\n' \
  >"${HOST_PREFIX}/share/mentor_pi_tracking/licenses/ALTO-GPL-2.0.txt"
printf '%s\n' '#!/usr/bin/env bash' 'exit 0' \
  >"${AGENT_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
chmod +x "${AGENT_PREFIX}/lib/micro_ros_agent/micro_ros_agent"
printf 'shared library\n' >"${AGENT_PREFIX}/lib/libfixture.so.1"
ln -s libfixture.so.1 "${AGENT_PREFIX}/lib/libfixture.so"
printf '%s\n' 'oci-layout fixture' >"${IMAGE_ARCHIVE}"

readonly SOURCE_SHA="$("${FINGERPRINT}" "${PROJECT_ROOT}")"
cat >"${HOST_PREFIX}/HOST-BUILD-METADATA.txt" <<EOF
format=rrclite-host-build-v2
ubuntu=22.04
target_os=ubuntu
target_version=22.04
architecture=amd64
ros_distro=humble
build_type=Release
builder_image=ros:humble-ros-base@sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
builder_image_id=sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
source_sha256=${SOURCE_SHA}
EOF
readonly AGENT_LOCK_SHA="$(sha256sum "${SCRIPT_DIR}/microros_agent_source.lock" | awk '{print $1}')"
readonly AGENT_SHA="$(sha256sum \
  "${AGENT_PREFIX}/lib/micro_ros_agent/micro_ros_agent" | awk '{print $1}')"
cat >"${AGENT_PREFIX}/AGENT-BUILD-METADATA.txt" <<EOF
format=rrclite-adaptive-agent-v1
ubuntu_target=22.04
ros_distro=humble
architecture=amd64
source_lock_sha256=${AGENT_LOCK_SHA}
rrclite_patch_sha256=cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
builder_identity=sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
executable_sha256=${AGENT_SHA}
EOF

readonly HANDOFF="${TEST_ROOT}/handoff"
"${PACKAGER}" \
  --host-prefix "${HOST_PREFIX}" \
  --agent-prefix "${AGENT_PREFIX}" \
  --runtime-image-archive "${IMAGE_ARCHIVE}" \
  --runtime-image-id "${IMAGE_ID}" \
  --runtime-image-source-id "${IMAGE_SOURCE_ID}" \
  --output-directory "${HANDOFF}" \
  --release-id fixture-r1 >/dev/null

(cd "${HANDOFF}" && sha256sum --check SHA256SUMS >/dev/null) || \
  Fail "handoff checksum manifest does not verify"
grep -Fqx 'package_format=rrclite-host-handoff-v2' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "handoff format is missing"
grep -Fqx "runtime_image_id=${IMAGE_ID}" "${HANDOFF}/HOST-HANDOFF.txt" || \
  Fail "runtime image identity is missing"
grep -Fqx "runtime_image_source_id=${IMAGE_SOURCE_ID}" \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "runtime image source identity is missing"
grep -Eq '^runtime_image_archive_sha256=[0-9a-f]{64}$' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "runtime archive hash is missing"
grep -Fqx 'runtime_image_archive_format=oci-v1' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "OCI archive format is missing"
grep -Fqx $'agent/lib/libfixture.so\tlibfixture.so.1' \
  "${HANDOFF}/SYMLINKS.txt" || Fail "Agent symlink manifest is missing"
[[ "$(readlink "${HANDOFF}/agent/lib/libfixture.so")" == libfixture.so.1 ]] || \
  Fail "Agent shared-library symlink was not preserved"
[[ -x "${HANDOFF}/agent/lib/micro_ros_agent/micro_ros_agent" ]] || \
  Fail "Agent artifact is missing"
[[ -s "${HANDOFF}/runtime-image/mentor-pi-runtime.tar" ]] || \
  Fail "runtime image archive is missing"
[[ -f "${HANDOFF}/THIRD-PARTY-NOTICES.txt" && \
   -f "${HANDOFF}/corresponding-source/mentor_pi/mentor_pi_ros2/third_party/altro-cpp/LICENSE" && \
   -f "${HANDOFF}/corresponding-source/mentor_pi/tools/altro_source.lock" && \
   -f "${HANDOFF}/corresponding-source/mentor_pi/mentor_pi_ros2/src/mentor_pi_tracking/package.xml" && \
   -f "${HANDOFF}/corresponding-source/mentor_pi/BUILD.txt" ]] || \
  Fail "handoff omits ALTO or tracking corresponding source"
readonly CORRESPONDING_ROOT="${HANDOFF}/corresponding-source/mentor_pi"
readonly TRACKING_CMAKE="${CORRESPONDING_ROOT}/mentor_pi_ros2/src/mentor_pi_tracking/CMakeLists.txt"
readonly EXPECTED_PATCH="${CORRESPONDING_ROOT}/tools/patches/altro-disable-docs.patch"
readonly EXPECTED_ALTO="${CORRESPONDING_ROOT}/mentor_pi_ros2/third_party/altro-cpp"
[[ -f "${TRACKING_CMAKE}" && -f "${EXPECTED_PATCH}" && \
   -f "${EXPECTED_ALTO}/CMakeLists.txt" ]] || \
  Fail "corresponding source does not preserve the build-expected layout"
(cd "${EXPECTED_ALTO}" && patch --dry-run --silent -p1 -i "${EXPECTED_PATCH}") || \
  Fail "packaged ALTO patch does not apply to packaged corresponding source"
grep -Eq '^altro_patch_sha256=[0-9a-f]{64}$' \
  "${HANDOFF}/HOST-HANDOFF.txt" || Fail "handoff omits the ALTO patch checksum"
[[ "$(find "${HANDOFF}/docs/tutorials" -maxdepth 1 -name '*.md' | wc -l)" == 8 ]] || \
  Fail "handoff does not contain exactly one 01--08 tutorial sequence"
[[ -z "$(find "${HANDOFF}/docs/tutorials" -mindepth 1 -type d -print -quit)" ]] || \
  Fail "handoff contains obsolete host-track tutorial directories"

cp -a "${HOST_PREFIX}" "${TEST_ROOT}/bad-host"
sed -i 's#builder_image=.*#builder_image=native-ubuntu-22.04#' \
  "${TEST_ROOT}/bad-host/HOST-BUILD-METADATA.txt"
ExpectFailure 'unpinned builder identity' "${PACKAGER}" \
  --host-prefix "${TEST_ROOT}/bad-host" \
  --agent-prefix "${AGENT_PREFIX}" \
  --runtime-image-archive "${IMAGE_ARCHIVE}" \
  --runtime-image-id "${IMAGE_ID}" \
  --runtime-image-source-id "${IMAGE_SOURCE_ID}" \
  --output-directory "${TEST_ROOT}/bad-output" \
  --release-id bad

cp -a "${HOST_PREFIX}" "${TEST_ROOT}/bad-host-image-id"
sed -i 's#builder_image_id=.*#builder_image_id=mutable-tag#' \
  "${TEST_ROOT}/bad-host-image-id/HOST-BUILD-METADATA.txt"
ExpectFailure 'builder image ID is not content-addressed' "${PACKAGER}" \
  --host-prefix "${TEST_ROOT}/bad-host-image-id" \
  --agent-prefix "${AGENT_PREFIX}" \
  --runtime-image-archive "${IMAGE_ARCHIVE}" \
  --runtime-image-id "${IMAGE_ID}" \
  --runtime-image-source-id "${IMAGE_SOURCE_ID}" \
  --output-directory "${TEST_ROOT}/bad-image-id-output" \
  --release-id bad-image-id

for tracked in tools/select_build_jobs.sh tools/detect_host_profile.sh \
    tools/validate_docker_host.sh \
    tools/prepare_build_images.sh tools/docker/host-runtime.zshrc \
    tools/build_host_handoff_container.sh tools/export_oci_image_archive.py \
    tools/export_oci_image_archive.sh tools/run_with_build_lock.sh; do
  grep -Fq "${tracked}" "${FINGERPRINT}" || \
    Fail "host fingerprint omits ${tracked}"
done
grep -Fq 'AppendDirectory "${PROJECT_ROOT}/docs/tutorials"' "${FINGERPRINT}" || \
  Fail "host fingerprint omits the tutorial sequence"
grep -Fq 'AppendDirectory "${PROJECT_ROOT}/mentor_pi_ros2/src/mentor_pi_bringup"' \
  "${FINGERPRINT}" || Fail "host fingerprint omits production runtime assets"
for obsolete in setup_onboard_ros_environment install_onboard_microros_setup \
    bootstrap_native_arm_toolchain; do
  ! grep -Fq "${obsolete}" "${FINGERPRINT}" || \
    Fail "host fingerprint retains obsolete ${obsolete} input"
done

grep -Fq '"${OCI_EXPORTER}" --print-runtime-id' \
  "${SCRIPT_DIR}/build_host_handoff_container.sh" || \
  Fail "container handoff does not capture the exported runtime image ID"
[[ -x "${OCI_EXPORTER}" ]] || Fail "OCI image exporter is not executable"
grep -Fq 'oci-layout' "${OCI_EXPORTER}" || \
  Fail "OCI image exporter does not validate the OCI layout marker"
grep -Fq "printf '%s\\n' \"\${runtime_image_id}\"" "${OCI_EXPORTER}" || \
  Fail "OCI image exporter has no machine-readable runtime ID output"
grep -Fq 'MENTOR_PI_RUNTIME_IMAGE_SOURCE_ID' \
  "${SCRIPT_DIR}/host_handoff_container_entrypoint.sh" || \
  Fail "container handoff does not propagate runtime image source provenance"
grep -Fq 'RRCLITE_BUILD_JOBS' "${SCRIPT_DIR}/host_handoff_container_entrypoint.sh" || \
  Fail "container handoff does not propagate the shared build budget"

echo "Docker host handoff contract tests passed."
