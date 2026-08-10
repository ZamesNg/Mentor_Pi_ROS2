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
[[ "$(RRCLITE_CPU_COUNT=24 "${SCRIPT_DIR}/select_rdk_handoff_jobs.sh")" == 8 ]] || \
  Fail "RDK handoff should default to eight workers on a large host"
[[ "$(RRCLITE_CPU_COUNT=3 "${SCRIPT_DIR}/select_rdk_handoff_jobs.sh")" == 3 ]] || \
  Fail "RDK handoff should scale down to the available CPU count"
[[ "$(RRCLITE_CPU_COUNT=16 RRCLITE_BUILD_JOBS=6 \
  "${SCRIPT_DIR}/select_rdk_handoff_jobs.sh")" == 6 ]] || \
  Fail "RDK handoff did not honor its explicit worker override"
ExpectFailure env RRCLITE_CPU_COUNT=4 RRCLITE_BUILD_JOBS=5 \
  "${SCRIPT_DIR}/select_rdk_handoff_jobs.sh"

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

grep -Fqx 'rdk-handoff:' "${SCRIPT_DIR}/../Makefile" || \
  Fail "Makefile does not expose the dedicated RDK handoff target"
grep -Fq './tools/rdk_handoff.sh' "${SCRIPT_DIR}/../Makefile" || \
  Fail "RDK handoff target does not use its dedicated implementation"
if grep -Eq '^[[:space:]]*ash([[:space:]]|$)' \
    "${SCRIPT_DIR}/rdk_handoff.sh"; then
  Fail "RDK handoff contains an invalid ash command"
fi
grep -Fq 'linux/arm64 QEMU/binfmt preflight failed' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not fail closed when binfmt is unavailable"
grep -Fq 'build_execution=qemu-emulated' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff omits emulation provenance"
grep -Fq 'target_architecture=arm64' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff is not fixed to arm64"
grep -Fq 'select_rdk_handoff_jobs.sh' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not use its bounded eight-worker policy"
grep -Fq 'RDK handoff QEMU package workers:' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not print the selected worker count"
grep -Fq 'rsync -a --exclude' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not isolate architecture-unsuffixed outputs"
grep -Fq 'rdk-handoff-cache/arm64/microros' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not preserve its verified arm64 micro-ROS cache"
grep -Fq 'Restored persistent arm64 micro-ROS cache for validation.' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "RDK handoff does not revalidate the persistent micro-ROS cache"
for contract in RDK_HANDOFF_FRESH CHECKPOINT.txt 'Retry with: make rdk-handoff' \
    --verified-build --resume 'test_host_runtime_image.sh --architecture arm64' \
    'MAKEFLAGS="-j1 -l1"'; do
  grep -Fq -- "${contract}" "${SCRIPT_DIR}/rdk_handoff.sh" \
      "${SCRIPT_DIR}/host_handoff_container_entrypoint.sh" || \
    Fail "resumable handoff contract is missing ${contract}"
done
if grep -Eq 'ALLOW_EMULATION|TARGET_ARCH' "${SCRIPT_DIR}/rdk_handoff.sh"; then
  Fail "RDK handoff requires an obsolete generic emulation switch"
fi

mkdir -p "${TEST_ROOT}/qemu-missing-bin"
cat >"${TEST_ROOT}/qemu-missing-bin/uname" <<'EOF'
#!/usr/bin/env bash
[[ "${1:-}" == -m ]] || exit 2
printf '%s\n' x86_64
EOF
cat >"${TEST_ROOT}/qemu-missing-bin/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == info ]] && exit 0
[[ "${1:-}" == run ]] && exit 42
exit 2
EOF
chmod +x "${TEST_ROOT}/qemu-missing-bin/uname" \
  "${TEST_ROOT}/qemu-missing-bin/docker"
set +e
preflight_output="$(PATH="${TEST_ROOT}/qemu-missing-bin:${PATH}" \
  RRCLITE_BUILD_LOCK_HELD=1 "${SCRIPT_DIR}/rdk_handoff.sh" 2>&1)"
preflight_status=$?
set -e
((preflight_status != 0)) || Fail "RDK handoff accepted missing QEMU/binfmt"
[[ "${preflight_output}" == *'linux/arm64 QEMU/binfmt preflight failed'* ]] || \
  Fail "RDK handoff did not report the failed QEMU/binfmt preflight"

echo "Docker host-profile and build-job policy tests passed."
