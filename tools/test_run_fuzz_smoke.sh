#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNNER="${SCRIPT_DIR}/run_fuzz_smoke.sh"
readonly PUBLISHER="${SCRIPT_DIR}/publish_fuzz_evidence.sh"
readonly VERIFIER="${SCRIPT_DIR}/verify_fuzz_evidence.sh"
readonly PAYLOAD_FILES=(
  "rrclite-fuzz-smoke-report.txt"
  "campaign.log"
  "production-source-sha256.txt"
  "test-input-sha256.txt"
  "source-corpus-sha256.txt"
  "final-corpus-sha256.txt"
  "toolchain.txt"
  "evidence-binding.txt"
)

Fail() {
  echo "Fuzz runner test failure: $*" >&2
  exit 1
}

Sha256File() {
  local file="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${file}" | awk '{print $1}'
    return
  fi
  shasum -a 256 "${file}" | awk '{print $1}'
}

WriteSha256Manifest() {
  local directory="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum "${PAYLOAD_FILES[@]}" >SHA256SUMS)
    return
  fi
  (cd "${directory}" && shasum -a 256 "${PAYLOAD_FILES[@]}" >SHA256SUMS)
}

MakeEvidenceSource() {
  local directory="$1"
  local run_id="$2"
  mkdir -p "${directory}"
  printf '%s\n' 'production source manifest fixture' \
    >"${directory}/production-source-sha256.txt"
  printf '%s\n' 'test input manifest fixture' \
    >"${directory}/test-input-sha256.txt"
  printf '%s\n' 'source corpus manifest fixture' \
    >"${directory}/source-corpus-sha256.txt"
  printf '%s\n' 'final corpus manifest fixture' \
    >"${directory}/final-corpus-sha256.txt"
  printf '%s\n' 'toolchain fixture' >"${directory}/toolchain.txt"
  printf '%s\n' 'campaign log fixture' >"${directory}/campaign.log"

  local production_sha256
  local test_sha256
  local source_corpus_sha256
  local final_corpus_sha256
  local toolchain_sha256
  local binding_sha256
  production_sha256="$(
    Sha256File "${directory}/production-source-sha256.txt"
  )"
  test_sha256="$(Sha256File "${directory}/test-input-sha256.txt")"
  source_corpus_sha256="$(
    Sha256File "${directory}/source-corpus-sha256.txt"
  )"
  final_corpus_sha256="$(
    Sha256File "${directory}/final-corpus-sha256.txt"
  )"
  toolchain_sha256="$(Sha256File "${directory}/toolchain.txt")"
  printf '%s\n' \
    'schema=rrclite-fuzz-binding-v1' \
    "production_source_sha256=${production_sha256}" \
    "test_input_sha256=${test_sha256}" \
    "source_corpus_sha256=${source_corpus_sha256}" \
    "toolchain_sha256=${toolchain_sha256}" \
    >"${directory}/evidence-binding.txt"
  binding_sha256="$(Sha256File "${directory}/evidence-binding.txt")"

  printf '%s\n' \
    'schema=rrclite-fuzz-campaign-v2' \
    'result=PASS' \
    "run_id=${run_id}" \
    'runs_per_harness=10000' \
    'harness_count=7' \
    'total_executions=70000' \
    'sanitizers=fuzzer,address,undefined' \
    'semantic_preflight=PASS' \
    "production_source_sha256=${production_sha256}" \
    "firmware_source_fingerprint_sha256=${production_sha256}" \
    'production_source_immutable=1' \
    "test_input_sha256=${test_sha256}" \
    'test_input_immutable=1' \
    "source_corpus_sha256=${source_corpus_sha256}" \
    'source_corpus_immutable=1' \
    "final_corpus_sha256=${final_corpus_sha256}" \
    'final_corpus_immutable=1' \
    "toolchain_sha256=${toolchain_sha256}" \
    'toolchain_immutable=1' \
    "evidence_binding_sha256=${binding_sha256}" \
    >"${directory}/rrclite-fuzz-smoke-report.txt"
}

[[ -x "${RUNNER}" ]] || Fail "runner is missing or not executable"
[[ -x "${PUBLISHER}" ]] || Fail "publisher is missing or not executable"
[[ -x "${VERIFIER}" ]] || Fail "verifier is missing or not executable"

readonly PLAN="$(${RUNNER} --runs 10000 --run-id plan-test --print-plan)"
grep -Fqx 'environment=linux-container' <<<"${PLAN}" || \
  Fail "plan does not select the Linux container"
grep -Fqx 'compiler=clang++-18' <<<"${PLAN}" || \
  Fail "plan does not select Clang 18"
grep -Fqx 'sanitizers=fuzzer,address,undefined' <<<"${PLAN}" || \
  Fail "plan does not enable the required sanitizers"
grep -Fqx 'runs_per_harness=10000' <<<"${PLAN}" || \
  Fail "plan has the wrong per-harness count"
grep -Fqx 'harness_count=7' <<<"${PLAN}" || \
  Fail "plan has the wrong harness count"
grep -Fqx 'total_executions=70000' <<<"${PLAN}" || \
  Fail "plan has the wrong total execution count"
grep -Fqx 'run_id=plan-test' <<<"${PLAN}" || \
  Fail "plan has the wrong run ID"
grep -Fqx 'evidence_directory=build/fuzz-evidence/plan-test' <<<"${PLAN}" || \
  Fail "plan has the wrong evidence directory"

for invalid_runs in 0 10000001 abc 1.5 -1; do
  if "${RUNNER}" --runs "${invalid_runs}" --print-plan >/dev/null 2>&1; then
    Fail "invalid run count was accepted: ${invalid_runs}"
  fi
done

if "${RUNNER}" --runs --print-plan >/dev/null 2>&1; then
  Fail "missing --runs value was accepted"
fi
if "${RUNNER}" --unknown >/dev/null 2>&1; then
  Fail "unknown option was accepted"
fi
for invalid_run_id in '../escape' '/absolute' 'bad/id' '..'; do
  if "${RUNNER}" --run-id "${invalid_run_id}" --print-plan \
    >/dev/null 2>&1; then
    Fail "invalid run ID was accepted: ${invalid_run_id}"
  fi
done

readonly TEST_DIRECTORY="$(
  mktemp -d "${TMPDIR:-/tmp}/rrclite-fuzz-evidence-test.XXXXXX"
)"
Cleanup() {
  chmod -R u+w "${TEST_DIRECTORY}" 2>/dev/null || true
  rm -rf -- "${TEST_DIRECTORY}"
}
trap Cleanup EXIT

readonly SOURCE_DIRECTORY="${TEST_DIRECTORY}/source"
readonly EVIDENCE_ROOT="${TEST_DIRECTORY}/evidence"
readonly TEST_RUN_ID="regression-run"
MakeEvidenceSource "${SOURCE_DIRECTORY}" "${TEST_RUN_ID}"
readonly PUBLISHED_DIRECTORY="$(
  "${PUBLISHER}" "${SOURCE_DIRECTORY}" "${EVIDENCE_ROOT}" "${TEST_RUN_ID}"
)"
[[ "${PUBLISHED_DIRECTORY}" == "${EVIDENCE_ROOT}/${TEST_RUN_ID}" ]] || \
  Fail "publisher returned the wrong directory"
"${VERIFIER}" "${PUBLISHED_DIRECTORY}" >/dev/null || \
  Fail "published evidence did not verify"

readonly REPORT_HASH_BEFORE="$(
  Sha256File "${PUBLISHED_DIRECTORY}/rrclite-fuzz-smoke-report.txt"
)"
if "${PUBLISHER}" "${SOURCE_DIRECTORY}" "${EVIDENCE_ROOT}" "${TEST_RUN_ID}" \
  >/dev/null 2>&1; then
  Fail "publisher overwrote an existing run"
fi
readonly REPORT_HASH_AFTER="$(
  Sha256File "${PUBLISHED_DIRECTORY}/rrclite-fuzz-smoke-report.txt"
)"
[[ "${REPORT_HASH_BEFORE}" == "${REPORT_HASH_AFTER}" ]] || \
  Fail "existing evidence changed after a rejected overwrite"

readonly TAMPER_SOURCE="${TEST_DIRECTORY}/tamper-source"
readonly TAMPER_ROOT="${TEST_DIRECTORY}/tamper-evidence"
readonly TAMPER_RUN_ID="tamper-run"
MakeEvidenceSource "${TAMPER_SOURCE}" "${TAMPER_RUN_ID}"
readonly TAMPER_DIRECTORY="$(
  "${PUBLISHER}" "${TAMPER_SOURCE}" "${TAMPER_ROOT}" "${TAMPER_RUN_ID}"
)"
chmod u+w "${TAMPER_DIRECTORY}" "${TAMPER_DIRECTORY}/campaign.log" \
  "${TAMPER_DIRECTORY}/production-source-sha256.txt" \
  "${TAMPER_DIRECTORY}/SHA256SUMS"
printf '%s\n' 'tampered' >>"${TAMPER_DIRECTORY}/campaign.log"
if "${VERIFIER}" "${TAMPER_DIRECTORY}" >/dev/null 2>&1; then
  Fail "verifier accepted a payload with a stale SHA-256 manifest"
fi
WriteSha256Manifest "${TAMPER_DIRECTORY}"
printf '%s\n' 'changed production manifest' \
  >"${TAMPER_DIRECTORY}/production-source-sha256.txt"
WriteSha256Manifest "${TAMPER_DIRECTORY}"
chmod a-w "${TAMPER_DIRECTORY}"/* "${TAMPER_DIRECTORY}"
if "${VERIFIER}" "${TAMPER_DIRECTORY}" >/dev/null 2>&1; then
  Fail "verifier accepted a report with a mismatched production digest"
fi

readonly CLEANUP_SOURCE="${TEST_DIRECTORY}/cleanup-source"
readonly CLEANUP_ROOT="${TEST_DIRECTORY}/cleanup-evidence"
MakeEvidenceSource "${CLEANUP_SOURCE}" cleanup-run
printf '%s\n' 'invalid binding' \
  >"${CLEANUP_SOURCE}/evidence-binding.txt"
if "${PUBLISHER}" "${CLEANUP_SOURCE}" "${CLEANUP_ROOT}" cleanup-run \
  >/dev/null 2>&1; then
  Fail "publisher accepted internally inconsistent evidence"
fi
[[ -z "$(find "${CLEANUP_ROOT}" -mindepth 1 -maxdepth 1 -print)" ]] || \
  Fail "failed publication left a staging or lock directory"

readonly REAL_ROOT="${TEST_DIRECTORY}/real-root"
readonly LINK_ROOT="${TEST_DIRECTORY}/link-root"
mkdir "${REAL_ROOT}"
ln -s "${REAL_ROOT}" "${LINK_ROOT}"
if "${PUBLISHER}" "${SOURCE_DIRECTORY}" "${LINK_ROOT}" symlink-root \
  >/dev/null 2>&1; then
  Fail "publisher accepted a symbolic-link evidence root"
fi

readonly SOURCE_LINK="${TEST_DIRECTORY}/source-link"
ln -s "${SOURCE_DIRECTORY}" "${SOURCE_LINK}"
if "${PUBLISHER}" "${SOURCE_LINK}" "${REAL_ROOT}" symlink-source \
  >/dev/null 2>&1; then
  Fail "publisher accepted a symbolic-link source directory"
fi

readonly BROKEN_ROOT="${TEST_DIRECTORY}/broken-root"
mkdir "${BROKEN_ROOT}"
ln -s missing-target "${BROKEN_ROOT}/broken-final"
if "${PUBLISHER}" "${SOURCE_DIRECTORY}" "${BROKEN_ROOT}" broken-final \
  >/dev/null 2>&1; then
  Fail "publisher accepted a pre-existing broken final symlink"
fi

readonly EVIDENCE_LINK="${TEST_DIRECTORY}/evidence-link"
ln -s "${PUBLISHED_DIRECTORY}" "${EVIDENCE_LINK}"
if "${VERIFIER}" --expected-run-id "${TEST_RUN_ID}" "${EVIDENCE_LINK}" \
  >/dev/null 2>&1; then
  Fail "verifier accepted a symbolic-link evidence directory"
fi

echo "Fuzz runner and immutable-evidence regression tests passed"
