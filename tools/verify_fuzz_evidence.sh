#!/usr/bin/env bash

set -euo pipefail

readonly REPORT_NAME="rrclite-fuzz-smoke-report.txt"
readonly CHECKSUM_NAME="SHA256SUMS"
readonly REQUIRED_PAYLOAD_FILES=(
  "${REPORT_NAME}"
  "campaign.log"
  "production-source-sha256.txt"
  "test-input-sha256.txt"
  "source-corpus-sha256.txt"
  "final-corpus-sha256.txt"
  "toolchain.txt"
  "evidence-binding.txt"
)

EXPECTED_RUN_ID=""

Usage() {
  cat <<'EOF'
Usage: ./tools/verify_fuzz_evidence.sh [--expected-run-id RUN_ID] EVIDENCE_DIR

Verify the closed RRCLite fuzz-evidence package, its SHA-256 manifest, and the
digest bindings recorded in its report. Without --expected-run-id, RUN_ID must
match the evidence directory basename.
EOF
}

Fail() {
  echo "Fuzz evidence verification error: $*" >&2
  exit 1
}

IsValidRunId() {
  [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$ ]]
}

Sha256File() {
  local file="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${file}" | awk '{print $1}'
    return
  fi
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "${file}" | awk '{print $1}'
    return
  fi
  Fail "neither sha256sum nor shasum is available"
}

ModeWithoutWriteBits() {
  local path="$1"
  local mode
  if mode="$(stat -c '%a' "${path}" 2>/dev/null)"; then
    :
  else
    mode="$(stat -f '%Lp' "${path}")"
  fi
  [[ "${mode}" =~ ^[0-7]{3,4}$ ]] || return 1
  (( (8#${mode} & 0222) == 0 ))
}

CheckSha256Manifest() {
  local directory="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum --check --strict "${CHECKSUM_NAME}") \
      >/dev/null
    return
  fi
  if command -v shasum >/dev/null 2>&1; then
    (cd "${directory}" && shasum -a 256 --check "${CHECKSUM_NAME}") \
      >/dev/null
    return
  fi
  Fail "neither sha256sum nor shasum is available"
}

ReportValue() {
  local report="$1"
  local key="$2"
  local count
  count="$(awk -F= -v key="${key}" '$1 == key { count += 1 } END { print count + 0 }' \
    "${report}")"
  [[ "${count}" == "1" ]] || \
    Fail "report key ${key} occurs ${count} times"
  awk -F= -v key="${key}" '$1 == key { sub(/^[^=]*=/, ""); print }' \
    "${report}"
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --expected-run-id)
      [[ "$#" -ge 2 ]] || Fail "--expected-run-id requires a value"
      EXPECTED_RUN_ID="$2"
      shift 2
      ;;
    -h | --help)
      Usage
      exit 0
      ;;
    --*)
      Usage >&2
      Fail "unknown argument: $1"
      ;;
    *)
      break
      ;;
  esac
done

[[ "$#" -eq 1 ]] || {
  Usage >&2
  Fail "exactly one evidence directory is required"
}

readonly EVIDENCE_DIRECTORY="$1"
[[ -d "${EVIDENCE_DIRECTORY}" ]] || \
  Fail "evidence directory does not exist: ${EVIDENCE_DIRECTORY}"
[[ ! -L "${EVIDENCE_DIRECTORY}" ]] || \
  Fail "evidence directory must not be a symbolic link"
ModeWithoutWriteBits "${EVIDENCE_DIRECTORY}" || \
  Fail "evidence directory has a writable mode bit"

for file in "${REQUIRED_PAYLOAD_FILES[@]}" "${CHECKSUM_NAME}"; do
  [[ -f "${EVIDENCE_DIRECTORY}/${file}" ]] || \
    Fail "missing required evidence file: ${file}"
  [[ ! -L "${EVIDENCE_DIRECTORY}/${file}" ]] || \
    Fail "evidence files must not be symbolic links: ${file}"
  ModeWithoutWriteBits "${EVIDENCE_DIRECTORY}/${file}" || \
    Fail "evidence file has a writable mode bit: ${file}"
done

readonly FILE_COUNT="$(find "${EVIDENCE_DIRECTORY}" -mindepth 1 -maxdepth 1 \
  -type f | wc -l | tr -d '[:space:]')"
[[ "${FILE_COUNT}" == "9" ]] || \
  Fail "evidence directory must contain exactly nine regular files"
readonly NON_FILE_COUNT="$(find "${EVIDENCE_DIRECTORY}" -mindepth 1 \
  -maxdepth 1 ! -type f | wc -l | tr -d '[:space:]')"
[[ "${NON_FILE_COUNT}" == "0" ]] || \
  Fail "evidence directory contains a non-regular entry"

readonly CHECKSUM_LINE_COUNT="$(wc -l <"${EVIDENCE_DIRECTORY}/${CHECKSUM_NAME}" \
  | tr -d '[:space:]')"
[[ "${CHECKSUM_LINE_COUNT}" == "8" ]] || \
  Fail "SHA256SUMS must contain exactly eight payload entries"
for file in "${REQUIRED_PAYLOAD_FILES[@]}"; do
  ENTRY_COUNT="$(awk -v file="${file}" '$2 == file { count += 1 } END { print count + 0 }' \
    "${EVIDENCE_DIRECTORY}/${CHECKSUM_NAME}")"
  [[ "${ENTRY_COUNT}" == "1" ]] || \
    Fail "checksum entry for ${file} occurs ${ENTRY_COUNT} times"
done
CheckSha256Manifest "${EVIDENCE_DIRECTORY}" || \
  Fail "SHA-256 payload verification failed"

readonly REPORT="${EVIDENCE_DIRECTORY}/${REPORT_NAME}"
readonly RUN_ID="$(ReportValue "${REPORT}" run_id)"
IsValidRunId "${RUN_ID}" || Fail "report has an invalid run_id"
if [[ -n "${EXPECTED_RUN_ID}" ]]; then
  IsValidRunId "${EXPECTED_RUN_ID}" || Fail "expected run ID is invalid"
  [[ "${RUN_ID}" == "${EXPECTED_RUN_ID}" ]] || \
    Fail "report run_id does not match --expected-run-id"
else
  [[ "${RUN_ID}" == "$(basename "${EVIDENCE_DIRECTORY}")" ]] || \
    Fail "report run_id does not match the evidence directory basename"
fi

[[ "$(ReportValue "${REPORT}" schema)" == "rrclite-fuzz-campaign-v2" ]] || \
  Fail "unsupported report schema"
[[ "$(ReportValue "${REPORT}" result)" == "PASS" ]] || \
  Fail "report does not record PASS"
[[ "$(ReportValue "${REPORT}" harness_count)" == "7" ]] || \
  Fail "report has the wrong harness count"
[[ "$(ReportValue "${REPORT}" sanitizers)" == \
  "fuzzer,address,undefined" ]] || Fail "report has the wrong sanitizers"
[[ "$(ReportValue "${REPORT}" semantic_preflight)" == "PASS" ]] || \
  Fail "semantic preflight did not pass"

for key in production_source_immutable test_input_immutable \
  source_corpus_immutable final_corpus_immutable toolchain_immutable; do
  [[ "$(ReportValue "${REPORT}" "${key}")" == "1" ]] || \
    Fail "${key} is not asserted"
done

readonly RUNS_PER_HARNESS="$(ReportValue "${REPORT}" runs_per_harness)"
readonly TOTAL_EXECUTIONS="$(ReportValue "${REPORT}" total_executions)"
[[ "${RUNS_PER_HARNESS}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "runs_per_harness is not a positive decimal integer"
[[ "${TOTAL_EXECUTIONS}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "total_executions is not a positive decimal integer"
((TOTAL_EXECUTIONS == RUNS_PER_HARNESS * 7)) || \
  Fail "total_executions is inconsistent with the run count"

readonly PRODUCTION_SOURCE_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/production-source-sha256.txt"
)"
readonly TEST_INPUT_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/test-input-sha256.txt"
)"
readonly SOURCE_CORPUS_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/source-corpus-sha256.txt"
)"
readonly FINAL_CORPUS_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/final-corpus-sha256.txt"
)"
readonly TOOLCHAIN_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/toolchain.txt"
)"
readonly EVIDENCE_BINDING_SHA256="$(
  Sha256File "${EVIDENCE_DIRECTORY}/evidence-binding.txt"
)"

[[ "$(ReportValue "${REPORT}" production_source_sha256)" == \
  "${PRODUCTION_SOURCE_SHA256}" ]] || Fail "production source digest mismatch"
[[ "$(ReportValue "${REPORT}" firmware_source_fingerprint_sha256)" == \
  "${PRODUCTION_SOURCE_SHA256}" ]] || \
  Fail "canonical firmware source fingerprint mismatch"
[[ "$(ReportValue "${REPORT}" test_input_sha256)" == \
  "${TEST_INPUT_SHA256}" ]] || Fail "test-input digest mismatch"
[[ "$(ReportValue "${REPORT}" source_corpus_sha256)" == \
  "${SOURCE_CORPUS_SHA256}" ]] || Fail "source-corpus digest mismatch"
[[ "$(ReportValue "${REPORT}" final_corpus_sha256)" == \
  "${FINAL_CORPUS_SHA256}" ]] || Fail "final-corpus digest mismatch"
[[ "$(ReportValue "${REPORT}" toolchain_sha256)" == \
  "${TOOLCHAIN_SHA256}" ]] || Fail "toolchain digest mismatch"
[[ "$(ReportValue "${REPORT}" evidence_binding_sha256)" == \
  "${EVIDENCE_BINDING_SHA256}" ]] || Fail "evidence-binding digest mismatch"

readonly EXPECTED_BINDING="$(printf '%s\n' \
  'schema=rrclite-fuzz-binding-v1' \
  "production_source_sha256=${PRODUCTION_SOURCE_SHA256}" \
  "test_input_sha256=${TEST_INPUT_SHA256}" \
  "source_corpus_sha256=${SOURCE_CORPUS_SHA256}" \
  "toolchain_sha256=${TOOLCHAIN_SHA256}")"
[[ "$(cat "${EVIDENCE_DIRECTORY}/evidence-binding.txt")" == \
  "${EXPECTED_BINDING}" ]] || Fail "evidence-binding contents are inconsistent"

echo "Fuzz evidence verification passed: ${EVIDENCE_DIRECTORY}"
