#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/mentor-pi-host-policy.XXXXXX")"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "Docker host-policy test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

cat >"${TEST_ROOT}/meminfo" <<'EOF'
MemAvailable:    7340032 kB
EOF
[[ "$(RRCLITE_MEMINFO_PATH="${TEST_ROOT}/meminfo" RRCLITE_CPU_COUNT=8 \
  "${SCRIPT_DIR}/select_build_jobs.sh")" == 3 ]] || \
  Fail "7 GiB/8 CPU policy should select three jobs"
[[ "$(RRCLITE_MEMINFO_PATH="${TEST_ROOT}/meminfo" RRCLITE_CPU_COUNT=2 \
  "${SCRIPT_DIR}/select_build_jobs.sh")" == 2 ]] || \
  Fail "CPU count should cap the job budget"
[[ "$(RRCLITE_MEMINFO_PATH="${TEST_ROOT}/meminfo" RRCLITE_CPU_COUNT=8 \
  RRCLITE_BUILD_JOBS=6 "${SCRIPT_DIR}/select_build_jobs.sh")" == 6 ]] || \
  Fail "valid explicit job override was not honored"
if RRCLITE_MEMINFO_PATH="${TEST_ROOT}/meminfo" RRCLITE_CPU_COUNT=4 \
    RRCLITE_BUILD_JOBS=5 "${SCRIPT_DIR}/select_build_jobs.sh" \
    >/dev/null 2>&1; then
  Fail "job override above the CPU count was accepted"
fi

cat >"${TEST_ROOT}/model" <<'EOF'
D-Robotics RDK X5
EOF
mkdir -p "${TEST_ROOT}/arm64-bin"
cat >"${TEST_ROOT}/arm64-bin/uname" <<'EOF'
#!/usr/bin/env bash
[[ "${1:-}" == -m ]] || exit 2
printf '%s\n' aarch64
EOF
chmod +x "${TEST_ROOT}/arm64-bin/uname"
profile="$(PATH="${TEST_ROOT}/arm64-bin:${PATH}" \
  RRCLITE_DEVICE_TREE_MODEL="${TEST_ROOT}/model" \
  "${SCRIPT_DIR}/detect_host_profile.sh")"
grep -Fqx 'profile=rdk-x5' <<<"${profile}" || \
  Fail "RDK X5 arm64 model fixture was not detected"

cat >"${TEST_ROOT}/ubuntu-22.04" <<'EOF'
ID=ubuntu
VERSION_ID="22.04"
EOF
cat >"${TEST_ROOT}/ubuntu-24.04" <<'EOF'
ID=ubuntu
VERSION_ID="24.04"
EOF
for fixture in ubuntu-22.04 ubuntu-24.04; do
  architecture=amd64
  [[ "${fixture}" != ubuntu-22.04 ]] || architecture=arm64
  policy="$("${SCRIPT_DIR}/validate_docker_host.sh" \
    --os-release "${TEST_ROOT}/${fixture}" \
    --architecture "${architecture}")"
  [[ "${policy}" == *'runtime=docker-humble'* && \
     "${policy}" == *'host_ros=unused'* ]] || \
    Fail "${fixture} did not select Docker-only Humble"
done
if grep -Eq 'source .*(/opt/ros|setup[.](bash|zsh))' \
    "${SCRIPT_DIR}/validate_docker_host.sh" "${SCRIPT_DIR}/doctor.sh"; then
  Fail "Docker host routing sources host ROS"
fi

case "$(uname -m)" in
  x86_64 | amd64) opposite_architecture=arm64 ;;
  aarch64 | arm64) opposite_architecture=amd64 ;;
  *) Fail "test host architecture is unsupported" ;;
esac
"${SCRIPT_DIR}/prepare_build_images.sh" \
  --architecture "${opposite_architecture}" --print project >/dev/null
ExpectFailure "${SCRIPT_DIR}/prepare_build_images.sh" \
  --architecture "${opposite_architecture}"

echo "Docker host-profile and build-job policy tests passed."
