#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly INSTALLER="${SCRIPT_DIR}/install_rdk_production.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "RDK production installer test failure: $*" >&2
  exit 1
}

readonly RELEASE_ID="rdk-arm64-20260811T010203Z"
readonly IMAGE_ID="sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
readonly BUNDLE="${TEST_ROOT}/${RELEASE_ID}"
readonly HOST_HANDOFF="${BUNDLE}/host-handoff"
readonly AGENT_EXECUTABLE="${HOST_HANDOFF}/agent/lib/micro_ros_agent/micro_ros_agent"
mkdir -p "${HOST_HANDOFF}/agent/lib/micro_ros_agent" \
  "${HOST_HANDOFF}/host/lib/mentor_pi_bringup" \
  "${HOST_HANDOFF}/runtime-image" "${TEST_ROOT}/fake-bin"
printf '%s\n' fixture-agent >"${AGENT_EXECUTABLE}"
chmod +x "${AGENT_EXECUTABLE}"
readonly AGENT_SHA="$(sha256sum "${AGENT_EXECUTABLE}" | awk '{print $1}')"
cat >"${HOST_HANDOFF}/agent/AGENT-BUILD-METADATA.txt" <<EOF
executable_sha256=${AGENT_SHA}
EOF
printf '%s\n' fixture-image > \
  "${HOST_HANDOFF}/runtime-image/mentor-pi-runtime.tar"
cat >"${HOST_HANDOFF}/HOST-HANDOFF.txt" <<EOF
release_id=${RELEASE_ID}
runtime_image_id=${IMAGE_ID}
EOF
printf '%s\n' fixture-library >"${HOST_HANDOFF}/agent/lib/libfixture.so.1"
ln -s libfixture.so.1 "${HOST_HANDOFF}/agent/lib/libfixture.so"
printf 'agent/lib/libfixture.so\tlibfixture.so.1\n' > \
  "${HOST_HANDOFF}/SYMLINKS.txt"

cat >"${HOST_HANDOFF}/host/lib/mentor_pi_bringup/promote_host_release" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >"${FAKE_PROMOTER_LOG:?}"
mkdir -p "${MENTOR_PI_PRODUCTION_TEST_ROOT}/opt/mentor_pi/host/lib/mentor_pi_bringup" \
  "${MENTOR_PI_PRODUCTION_TEST_ROOT}/opt/mentor_pi/releases/host/rdk-arm64-20260811T010203Z"
cp "${FAKE_INSTALLER_SOURCE:?}" \
  "${MENTOR_PI_PRODUCTION_TEST_ROOT}/opt/mentor_pi/host/lib/mentor_pi_bringup/install_production_assets"
chmod +x \
  "${MENTOR_PI_PRODUCTION_TEST_ROOT}/opt/mentor_pi/host/lib/mentor_pi_bringup/install_production_assets"
EOF
chmod +x "${HOST_HANDOFF}/host/lib/mentor_pi_bringup/promote_host_release"

cat >"${TEST_ROOT}/fake-installer" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >"${FAKE_INSTALLER_LOG:?}"
EOF
chmod +x "${TEST_ROOT}/fake-installer"

(cd "${HOST_HANDOFF}" && \
  find . -type f ! -name SHA256SUMS -printf '%P\n' | LC_ALL=C sort | \
    xargs sha256sum >SHA256SUMS)

cat >"${TEST_ROOT}/fake-receiver" <<EOF
#!/usr/bin/env bash
printf '%s\n' '${BUNDLE}'
EOF
chmod +x "${TEST_ROOT}/fake-receiver"

cat >"${TEST_ROOT}/fake-bin/docker" <<EOF
#!/usr/bin/env bash
set -euo pipefail
case "\${1:-} \${2:-}" in
  'load --input') exit 0 ;;
  'image inspect')
    if [[ "\${*: -1}" == *Architecture* ]]; then
      printf '%s\n' linux/arm64
    else
      printf '%s\n' '${IMAGE_ID}'
    fi
    ;;
  *) exit 2 ;;
esac
EOF
cat >"${TEST_ROOT}/fake-bin/systemctl" <<'EOF'
#!/usr/bin/env bash
[[ "${1:-}" == is-active ]] && exit 3
exit 0
EOF
cat >"${TEST_ROOT}/fake-bin/systemd-analyze" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >"${FAKE_SYSTEMD_ANALYZE_LOG:?}"
EOF
cat >"${TEST_ROOT}/fake-bin/sudo" <<'EOF'
#!/usr/bin/env bash
exec "$@"
EOF
chmod +x "${TEST_ROOT}/fake-bin/"*

FAKE_PROMOTER_LOG="${TEST_ROOT}/promoter.log" \
FAKE_INSTALLER_LOG="${TEST_ROOT}/installer.log" \
FAKE_INSTALLER_SOURCE="${TEST_ROOT}/fake-installer" \
FAKE_SYSTEMD_ANALYZE_LOG="${TEST_ROOT}/systemd-analyze.log" \
MENTOR_PI_PRODUCTION_TEST_ROOT="${TEST_ROOT}/installed" \
MENTOR_PI_RDK_RECEIVER="${TEST_ROOT}/fake-receiver" \
PATH="${TEST_ROOT}/fake-bin:${PATH}" \
  "${INSTALLER}" --mode first-install --device /dev/ttyUSB0 \
    --ros-domain-id 37 --identity-kind serial \
    --identity-value RRCLITE_A1B2C3 >/dev/null

grep -Fqx -- "--staged-prefix ${HOST_HANDOFF}/host --release-id ${RELEASE_ID}" \
  "${TEST_ROOT}/promoter.log" || Fail "host promotion arguments are wrong"
grep -Fqx -- \
  "--mode first-install --ros-domain-id 37 --identity-kind serial --identity-value RRCLITE_A1B2C3 --device /dev/ttyUSB0 --runtime-image ${IMAGE_ID}" \
  "${TEST_ROOT}/installer.log" || Fail "production installer arguments are wrong"
grep -Fq 'mentor-pi-runtime.service' "${TEST_ROOT}/systemd-analyze.log" || \
  Fail "systemd units were not verified"
[[ -L "${TEST_ROOT}/installed/opt/mentor_pi/micro_ros_agent" ]] || \
  Fail "active Agent link was not installed"

FAKE_PROMOTER_LOG="${TEST_ROOT}/promoter.log" \
FAKE_INSTALLER_LOG="${TEST_ROOT}/installer.log" \
FAKE_INSTALLER_SOURCE="${TEST_ROOT}/fake-installer" \
FAKE_SYSTEMD_ANALYZE_LOG="${TEST_ROOT}/systemd-analyze.log" \
MENTOR_PI_PRODUCTION_TEST_ROOT="${TEST_ROOT}/installed" \
MENTOR_PI_RDK_RECEIVER="${TEST_ROOT}/fake-receiver" \
PATH="${TEST_ROOT}/fake-bin:${PATH}" \
  "${INSTALLER}" --mode upgrade --device /dev/ttyUSB0 \
    --ros-domain-id 37 --identity-kind serial \
    --identity-value RRCLITE_A1B2C3 >/dev/null
grep -Fqx -- "--activate-release ${RELEASE_ID}" \
  "${TEST_ROOT}/promoter.log" || \
  Fail "installed release was not reactivated for rollback or retry"
grep -Fq -- '--mode upgrade' "${TEST_ROOT}/installer.log" || \
  Fail "upgrade mode was not forwarded"

echo "RDK production installer tests passed."
