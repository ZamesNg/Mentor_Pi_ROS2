#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
readonly PACKAGE_SCRIPT="${SCRIPT_DIR}/package_board_handoff.sh"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/rrclite-handoff-test.XXXXXX")"
readonly TEST_ROOT

Cleanup() {
  [[ -d "${TEST_ROOT}" ]] || return
  chmod -R u+rwX "${TEST_ROOT}" 2>/dev/null || true
  rm -rf -- "${TEST_ROOT}"
}
trap Cleanup EXIT

Fail() {
  echo "Board-handoff test failure: $*" >&2
  exit 1
}

AssertFile() {
  [[ -f "$1" ]] || Fail "missing file $1"
}

AssertNoPath() {
  [[ ! -e "$1" && ! -L "$1" ]] || Fail "unexpected path $1"
}

VerifyManifest() {
  local directory="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum --check SHA256SUMS >/dev/null)
  else
    (cd "${directory}" && shasum -a 256 --check SHA256SUMS >/dev/null)
  fi
}

SetUpFakeProject() {
  local project="$1"
  mkdir -p -- "${project}/tools"
  cp -- "${PACKAGE_SCRIPT}" "${SCRIPT_DIR}/run_with_build_lock.sh" \
    "${project}/tools/"
  chmod +x "${project}/tools/package_board_handoff.sh" \
    "${project}/tools/run_with_build_lock.sh"

  cat >"${project}/tools/build_firmware.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
build_root="${project_root}/firmware/mentor_pi_mcu/build/stm32"
count_file="${project_root}/build-count.txt"
order_file="${project_root}/build-order.txt"

count=0
if [[ -f "${count_file}" ]]; then
  read -r count <"${count_file}"
fi
count=$((count + 1))
printf '%s\n' "${count}" >"${count_file}"

mode="PID"

printf '%s\n' "${mode}" >>"${order_file}"

if [[ -n "${FAKE_FAIL_PID_FROM_COUNT:-}" && \
      "${count}" -ge "${FAKE_FAIL_PID_FROM_COUNT}" ]]; then
  exit 10
fi

mkdir -p -- "${build_root}"
printf '%s\n' 'CMAKE_BUILD_TYPE:STRING=MinSizeRel' \
  >"${build_root}/CMakeCache.txt"
for extension in elf hex bin map; do
  printf '%s-build-%s-%s\n' "${mode}" "${count}" "${extension}" \
    >"${build_root}/mentor_pi_mcu.${extension}"
done
Sha256() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}
printf '%s\n' \
  'schema=rrclite-firmware-build-v2' \
  'target=STM32F407VET6' \
  'ros_distro=humble' \
  "builder_mode=${FAKE_BUILDER_MODE:-docker-pinned}" \
  "motor_mode=${mode}" \
  'control_mode=CLOSED_LOOP' \
  'artifact_mode=NORMAL' \
  'release_qualified=0' \
  'source_sha256=0000000000000000000000000000000000000000000000000000000000000000' \
  'interfaces_sha256=1111111111111111111111111111111111111111111111111111111111111111' \
  'microros_archive_sha256=2222222222222222222222222222222222222222222222222222222222222222' \
  'microros_tree_sha256=3333333333333333333333333333333333333333333333333333333333333333' \
  >"${build_root}/rrclite-build-metadata.txt"
for extension in elf hex bin map; do
  artifact_hash="$(Sha256 "${build_root}/mentor_pi_mcu.${extension}")"
  if [[ "${FAKE_BAD_METADATA_EXTENSION:-}" == "${extension}" ]]; then
    artifact_hash="ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
  fi
  printf '%s_sha256=%s\n' "${extension}" "${artifact_hash}" \
    >>"${build_root}/rrclite-build-metadata.txt"
done
if [[ -n "${FAKE_MISSING_METADATA_KEY:-}" ]]; then
  grep -v "^${FAKE_MISSING_METADATA_KEY}=" \
    "${build_root}/rrclite-build-metadata.txt" \
    >"${build_root}/rrclite-build-metadata.tmp"
  mv "${build_root}/rrclite-build-metadata.tmp" \
    "${build_root}/rrclite-build-metadata.txt"
fi
EOF
  chmod +x "${project}/tools/build_firmware.sh"
}

AssertPidBuild() {
  local project="$1"
  local cache="${project}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
  AssertFile "${cache}"
  grep -Fqx 'CMAKE_BUILD_TYPE:STRING=MinSizeRel' "${cache}" || \
    Fail "authoritative PID build did not use MinSizeRel"
}

base_project="${TEST_ROOT}/base-default"
SetUpFakeProject "${base_project}"
"${base_project}/tools/package_board_handoff.sh" handoff >/dev/null
AssertFile "${base_project}/handoff/firmware-pid-release/mentor_pi_mcu-firmware-pid-release.elf"
AssertFile "${base_project}/handoff/firmware-pid-release/SHA256SUMS"
AssertFile "${base_project}/handoff/firmware-pid-release/BUILD-METADATA.txt"
AssertFile "${base_project}/handoff/firmware-pid-release/BUILD-MODE.txt"
grep -Fqx 'classification=NORMAL_CLOSED_LOOP_DEFAULT' \
  "${base_project}/handoff/firmware-pid-release/BUILD-MODE.txt" || \
  Fail "firmware-pid-release BUILD-MODE does not mark the normal closed-loop default"
grep -Fqx 'control_mode=CLOSED_LOOP' \
  "${base_project}/handoff/firmware-pid-release/BUILD-MODE.txt" || \
  Fail "firmware-pid-release BUILD-MODE does not report closed-loop PID"
VerifyManifest "${base_project}/handoff/firmware-pid-release"
VerifyManifest "${base_project}/handoff"
AssertPidBuild "${base_project}"
[[ "$(tr -d '\n' <"${base_project}/build-order.txt")" == "PID" ]] || \
  Fail "base build order was not PID only"
"${base_project}/tools/package_board_handoff.sh" --verified-build \
  verified-handoff >/dev/null
AssertFile "${base_project}/verified-handoff/firmware-pid-release/mentor_pi_mcu-firmware-pid-release.elf"
[[ "$(tr -d '\n' <"${base_project}/build-order.txt")" == "PID" ]] || \
  Fail "verified-build packaging rebuilt PID firmware"

failure_project="${TEST_ROOT}/metadata-mismatch"
SetUpFakeProject "${failure_project}"
if FAKE_BAD_METADATA_EXTENSION=hex \
    "${failure_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "metadata/artifact mismatch unexpectedly packaged"
fi
AssertNoPath "${failure_project}/handoff"
[[ "$(tr -d '\n' <"${failure_project}/build-order.txt")" == "PID" ]] || \
  Fail "failure build order was not PID only"

restore_failure_project="${TEST_ROOT}/pid-build-failure"
SetUpFakeProject "${restore_failure_project}"
if FAKE_FAIL_PID_FROM_COUNT=1 \
    "${restore_failure_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "simulated PID build failure unexpectedly succeeded"
fi
AssertNoPath "${restore_failure_project}/handoff"
AssertNoPath \
  "${restore_failure_project}/firmware/mentor_pi_mcu/build/stm32"
[[ "$(tr -d '\n' <"${restore_failure_project}/build-order.txt")" == "PID" ]] || \
  Fail "restore-failure build order was not PID only"

missing_metadata_project="${TEST_ROOT}/missing-metadata"
SetUpFakeProject "${missing_metadata_project}"
if FAKE_MISSING_METADATA_KEY=microros_tree_sha256 \
    "${missing_metadata_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "missing metadata key unexpectedly packaged"
fi
AssertNoPath "${missing_metadata_project}/handoff"

unsupported_builder_project="${TEST_ROOT}/unsupported-builder"
SetUpFakeProject "${unsupported_builder_project}"
if FAKE_BUILDER_MODE=native-explicit \
    "${unsupported_builder_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "unsupported native-explicit build unexpectedly packaged"
fi
AssertNoPath "${unsupported_builder_project}/handoff"

echo "Board-handoff packaging tests passed."
