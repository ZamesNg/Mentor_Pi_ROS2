#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
readonly PACKAGE_SCRIPT="${SCRIPT_DIR}/package_board_handoff.sh"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/rrclite-handoff-test.XXXXXX")"
readonly TEST_ROOT

Cleanup() {
  cmake -E remove_directory "${TEST_ROOT}"
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
  cmake -E make_directory "${project}/tools"
  cmake -E copy "${PACKAGE_SCRIPT}" \
    "${project}/tools/package_board_handoff.sh"
  chmod +x "${project}/tools/package_board_handoff.sh"

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

case "${RRCLITE_MOTOR_COMMISSIONING:-0}" in
  0)
    mode="LOCKED"
    option="OFF"
    ack=""
    ;;
  1)
    [[ "${RRCLITE_MOTOR_COMMISSIONING_ACK:-}" == "MOTORS_RAISED" ]]
    mode="COMMISSIONING"
    option="ON"
    ack="MOTORS_RAISED"
    ;;
  *)
    exit 2
    ;;
esac
printf '%s\n' "${mode}" >>"${order_file}"

if [[ "${mode}" == "COMMISSIONING" && \
      "${FAKE_FAIL_COMMISSIONING:-0}" == "1" ]]; then
  exit 9
fi
if [[ "${mode}" == "LOCKED" && \
      -n "${FAKE_FAIL_LOCKED_FROM_COUNT:-}" && \
      "${count}" -ge "${FAKE_FAIL_LOCKED_FROM_COUNT}" ]]; then
  exit 10
fi

cmake -E make_directory "${build_root}"
printf 'RRCLITE_MOTOR_COMMISSIONING:BOOL=%s\n' "${option}" \
  >"${build_root}/CMakeCache.txt"
printf 'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=%s\n' "${ack}" \
  >>"${build_root}/CMakeCache.txt"
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
  "motor_mode=${mode}" \
  'artifact_mode=NORMAL' \
  "commissioning_ack=${ack}" \
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

AssertLockedAuthoritativeBuild() {
  local project="$1"
  local cache="${project}/firmware/mentor_pi_mcu/build/stm32/CMakeCache.txt"
  AssertFile "${cache}"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING:BOOL=OFF' "${cache}" || \
    Fail "authoritative build is not locked"
  grep -Fqx 'RRCLITE_MOTOR_COMMISSIONING_ACK:STRING=' "${cache}" || \
    Fail "authoritative build retained a commissioning acknowledgement"
}

locked_project="${TEST_ROOT}/locked-only"
SetUpFakeProject "${locked_project}"
"${locked_project}/tools/package_board_handoff.sh" handoff >/dev/null
AssertFile "${locked_project}/handoff/locked/mentor_pi_mcu-locked.elf"
AssertFile "${locked_project}/handoff/locked/SHA256SUMS"
AssertFile "${locked_project}/handoff/locked/BUILD-METADATA.txt"
AssertNoPath "${locked_project}/handoff/commissioning-nonrelease"
VerifyManifest "${locked_project}/handoff/locked"
VerifyManifest "${locked_project}/handoff"
AssertLockedAuthoritativeBuild "${locked_project}"
[[ "$(tr -d '\n' <"${locked_project}/build-order.txt")" == "LOCKED" ]] || \
  Fail "locked-only build order was not LOCKED"

gate_project="${TEST_ROOT}/missing-gate"
SetUpFakeProject "${gate_project}"
if RRCLITE_MOTOR_COMMISSIONING=1 \
    "${gate_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "commissioning succeeded without MOTORS_RAISED acknowledgement"
fi
AssertNoPath "${gate_project}/handoff"
AssertNoPath "${gate_project}/build-order.txt"

dual_project="${TEST_ROOT}/dual-package"
SetUpFakeProject "${dual_project}"
RRCLITE_MOTOR_COMMISSIONING=1 \
RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
  "${dual_project}/tools/package_board_handoff.sh" handoff >/dev/null
AssertFile "${dual_project}/handoff/locked/mentor_pi_mcu-locked.elf"
AssertFile "${dual_project}/handoff/commissioning-nonrelease/mentor_pi_mcu-commissioning-nonrelease.elf"
AssertFile "${dual_project}/handoff/commissioning-nonrelease/SHA256SUMS"
AssertFile "${dual_project}/handoff/commissioning-nonrelease/BUILD-METADATA.txt"
VerifyManifest "${dual_project}/handoff/locked"
VerifyManifest "${dual_project}/handoff/commissioning-nonrelease"
VerifyManifest "${dual_project}/handoff"
AssertLockedAuthoritativeBuild "${dual_project}"
for extension in elf hex bin map; do
  cmp \
    "${dual_project}/firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.${extension}" \
    "${dual_project}/handoff/locked/mentor_pi_mcu-locked.${extension}"
done
expected_order=$'LOCKED\nCOMMISSIONING\nLOCKED'
[[ "$(sed -n '1,3p' "${dual_project}/build-order.txt")" == \
   "${expected_order}" ]] || Fail "dual-package build order was unsafe"

failure_project="${TEST_ROOT}/commissioning-failure"
SetUpFakeProject "${failure_project}"
if RRCLITE_MOTOR_COMMISSIONING=1 \
    RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
    FAKE_FAIL_COMMISSIONING=1 \
    "${failure_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "simulated commissioning failure unexpectedly succeeded"
fi
AssertNoPath "${failure_project}/handoff"
AssertLockedAuthoritativeBuild "${failure_project}"
expected_failure_order=$'LOCKED\nCOMMISSIONING\nLOCKED'
[[ "$(sed -n '1,3p' "${failure_project}/build-order.txt")" == \
   "${expected_failure_order}" ]] || \
  Fail "commissioning failure did not restore a locked build"

restore_failure_project="${TEST_ROOT}/locked-restore-failure"
SetUpFakeProject "${restore_failure_project}"
if RRCLITE_MOTOR_COMMISSIONING=1 \
    RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
    FAKE_FAIL_LOCKED_FROM_COUNT=3 \
    "${restore_failure_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "simulated locked-restoration failure unexpectedly succeeded"
fi
AssertNoPath "${restore_failure_project}/handoff"
AssertNoPath \
  "${restore_failure_project}/firmware/mentor_pi_mcu/build/stm32"

metadata_mismatch_project="${TEST_ROOT}/metadata-mismatch"
SetUpFakeProject "${metadata_mismatch_project}"
if FAKE_BAD_METADATA_EXTENSION=hex \
    "${metadata_mismatch_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "metadata/artifact mismatch unexpectedly packaged"
fi
AssertNoPath "${metadata_mismatch_project}/handoff"

missing_metadata_project="${TEST_ROOT}/missing-metadata"
SetUpFakeProject "${missing_metadata_project}"
if FAKE_MISSING_METADATA_KEY=microros_tree_sha256 \
    "${missing_metadata_project}/tools/package_board_handoff.sh" handoff \
    >/dev/null 2>&1; then
  Fail "missing metadata key unexpectedly packaged"
fi
AssertNoPath "${missing_metadata_project}/handoff"

echo "Board-handoff packaging tests passed."
