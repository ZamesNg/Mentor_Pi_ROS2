#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() { echo "RDK resume fixture failure: $*" >&2; exit 1; }
RequireLine() {
  [[ -f "$1" && ! -L "$1" ]] || Fail "required metadata is missing: $1"
  grep -Fqx "$2" "$1" || Fail "required metadata '$2' is missing from $1"
}

CURRENT_IDENTITY=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
CHECKPOINT_ROOT="${TEST_ROOT}/checkpoints"
CHECKPOINT_MANIFEST="${TEST_ROOT}/CHECKPOINT.txt"
current_stage=fixture
mkdir "${CHECKPOINT_ROOT}"
printf '%s\n' 'format=rrclite-rdk-checkpoint-v1' 'completed_stage=none' \
  >"${CHECKPOINT_MANIFEST}"

# Exercise the production checkpoint functions without starting Docker/QEMU.
checkpoint_functions="$(awk '
  /^StageMarker\(\)/ {copy=1}
  /^if \(\(resume == 0\)\); then/ {copy=0}
  copy {print}
' "${SCRIPT_DIR}/rdk_handoff.sh")"
[[ -n "${checkpoint_functions}" ]] || Fail "checkpoint functions were not found"
eval "${checkpoint_functions}"

grep -Fq 'RefreshChangedInputs' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff does not refresh changed source inputs"
grep -Fq 'Preserving unaffected completed RDK stage:' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff does not preserve unaffected completed stages"
grep -Fq -- '--compile "${WORK_ROOT}"' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff does not distinguish host build and packaging inputs"
grep -Fq -- '--package-payload' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff cannot isolate tutorial-only package refreshes"
! grep -Fq '"${HOST_FINGERPRINT}" "${WORK_ROOT}"' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "checkpoint validation still uses the legacy broad host fingerprint"
grep -Fq 'Refreshing only packaged tutorials; preserving the OCI archive' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "tutorial-only changes would recreate the OCI package"
grep -Fq 'verify_oci_image_archive.py' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff does not validate the packaged OCI descriptor"
grep -Fq 'Reusing compatible completed RDK handoff:' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "an unchanged completed handoff would start a new release"
grep -Fq 'Removed redundant incomplete RDK handoff work:' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "completed-handoff reuse retains redundant incomplete work"
grep -Fq 'RDK_HANDOFF_FRESH:-0' "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "the completed-handoff reuse path cannot be explicitly bypassed"
grep -Fq 'docker image inspect "${RUNTIME_IMAGE_SOURCE_ID}"' \
  "${SCRIPT_DIR}/rdk_handoff.sh" || \
  Fail "handoff inspects the packaged manifest ID instead of the source image"
for field in image_stage_input_sha256 microros_stage_input_sha256 \
    pid_stage_input_sha256 agent_stage_input_sha256 \
    host_build_stage_input_sha256 host_package_stage_input_sha256; do
  grep -Fq "${field}=" "${SCRIPT_DIR}/rdk_handoff.sh" || \
    Fail "handoff checkpoint omits per-stage identity ${field}"
done

stages=(image-preparation microros-library pid-firmware agent-build \
  host-build-tests host-oci-packaging runtime-smoke final-bundle)
index=0
for stage in "${stages[@]}"; do
  ((index += 1))
  checksum="$(printf '%064x' "${index}")"
  CompleteStage "${stage}" "${checksum}" >/dev/null
  StageDone "${stage}" || Fail "completed stage was not reusable: ${stage}"
  VerifyStageChecksum "${stage}" "${checksum}"
  if BeginStage "${stage}" >/dev/null; then
    Fail "retry did not skip validated stage: ${stage}"
  fi
done

grep -Fqx 'completed_stage=final-bundle' "${CHECKPOINT_MANIFEST}" || \
  Fail "manifest does not retain the latest completed stage"
[[ -z "$(find "${TEST_ROOT}" -name '*.tmp.*' -print -quit)" ]] || \
  Fail "atomic checkpoint temporary files remain"

printf 'tampered=yes\n' >>"$(StageMarker runtime-smoke)"
if (BeginStage runtime-smoke >/dev/null 2>&1); then
  Fail "tampered stage marker was accepted"
fi

rm -f -- "$(StageMarker final-bundle)"
ln -s /dev/null "$(StageMarker final-bundle)"
if (BeginStage final-bundle >/dev/null 2>&1); then
  Fail "symbolic stage marker was accepted"
fi

rm -f -- "$(StageMarker final-bundle)"
BeginStage final-bundle >/dev/null || Fail "missing unfinished marker was not resumable"

echo "RDK handoff eight-stage checkpoint/resume fixtures passed."
