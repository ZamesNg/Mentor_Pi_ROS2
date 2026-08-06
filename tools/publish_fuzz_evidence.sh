#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly VERIFIER="${SCRIPT_DIR}/verify_fuzz_evidence.sh"
readonly REQUIRED_PAYLOAD_FILES=(
  "rrclite-fuzz-smoke-report.txt"
  "campaign.log"
  "production-source-sha256.txt"
  "test-input-sha256.txt"
  "source-corpus-sha256.txt"
  "final-corpus-sha256.txt"
  "toolchain.txt"
  "evidence-binding.txt"
)

Usage() {
  cat <<'EOF'
Usage: ./tools/publish_fuzz_evidence.sh SOURCE_DIR EVIDENCE_ROOT RUN_ID

Validate and atomically publish one successful RRCLite fuzz run. RUN_ID may
contain letters, digits, dots, underscores, and hyphens. An existing run is
never replaced.
EOF
}

Fail() {
  echo "Fuzz evidence publication error: $*" >&2
  exit 1
}

WriteSha256Manifest() {
  local directory="$1"
  shift
  if command -v sha256sum >/dev/null 2>&1; then
    (cd "${directory}" && sha256sum "$@" >SHA256SUMS)
    return
  fi
  if command -v shasum >/dev/null 2>&1; then
    (cd "${directory}" && shasum -a 256 "$@" >SHA256SUMS)
    return
  fi
  Fail "neither sha256sum nor shasum is available"
}

[[ "$#" -eq 3 ]] || {
  Usage >&2
  Fail "SOURCE_DIR, EVIDENCE_ROOT, and RUN_ID are required"
}

readonly SOURCE_DIRECTORY="$1"
readonly EVIDENCE_ROOT="$2"
readonly RUN_ID="$3"
[[ "${RUN_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$ ]] || \
  Fail "invalid RUN_ID"
[[ -d "${SOURCE_DIRECTORY}" ]] || \
  Fail "source directory does not exist: ${SOURCE_DIRECTORY}"
[[ ! -L "${SOURCE_DIRECTORY}" ]] || \
  Fail "source directory must not be a symbolic link"
[[ -x "${VERIFIER}" ]] || Fail "evidence verifier is not executable"

for file in "${REQUIRED_PAYLOAD_FILES[@]}"; do
  [[ -f "${SOURCE_DIRECTORY}/${file}" ]] || \
    Fail "missing source evidence file: ${file}"
  [[ ! -L "${SOURCE_DIRECTORY}/${file}" ]] || \
    Fail "source evidence files must not be symbolic links: ${file}"
done

[[ ! -L "${EVIDENCE_ROOT}" ]] || \
  Fail "evidence root must not be a symbolic link"
mkdir -p "${EVIDENCE_ROOT}"
[[ -d "${EVIDENCE_ROOT}" && ! -L "${EVIDENCE_ROOT}" ]] || \
  Fail "evidence root is not a real directory"
readonly FINAL_DIRECTORY="${EVIDENCE_ROOT}/${RUN_ID}"
readonly LOCK_DIRECTORY="${EVIDENCE_ROOT}/.${RUN_ID}.publish-lock"
readonly STAGING_DIRECTORY="${EVIDENCE_ROOT}/.${RUN_ID}.staging.$$"
[[ ! -e "${FINAL_DIRECTORY}" && ! -L "${FINAL_DIRECTORY}" ]] || \
  Fail "evidence run already exists and will not be overwritten: ${RUN_ID}"
[[ ! -e "${LOCK_DIRECTORY}" && ! -L "${LOCK_DIRECTORY}" ]] || \
  Fail "evidence run is already being published: ${RUN_ID}"
mkdir "${LOCK_DIRECTORY}" 2>/dev/null || \
  Fail "evidence run is already being published: ${RUN_ID}"

STAGING_CREATED=0
PUBLISHED=0

Cleanup() {
  if ((STAGING_CREATED == 1 && PUBLISHED == 0)) &&
    [[ -d "${STAGING_DIRECTORY}" && ! -L "${STAGING_DIRECTORY}" ]]; then
    chmod -R u+w "${STAGING_DIRECTORY}" 2>/dev/null || true
    rm -rf -- "${STAGING_DIRECTORY}"
  fi
  rmdir "${LOCK_DIRECTORY}" 2>/dev/null || true
}
trap Cleanup EXIT

[[ ! -e "${STAGING_DIRECTORY}" && ! -L "${STAGING_DIRECTORY}" ]] || \
  Fail "staging directory already exists: ${STAGING_DIRECTORY}"
mkdir "${STAGING_DIRECTORY}"
STAGING_CREATED=1
for file in "${REQUIRED_PAYLOAD_FILES[@]}"; do
  cp "${SOURCE_DIRECTORY}/${file}" "${STAGING_DIRECTORY}/${file}"
done
WriteSha256Manifest "${STAGING_DIRECTORY}" "${REQUIRED_PAYLOAD_FILES[@]}"
chmod a-w "${STAGING_DIRECTORY}"/*
chmod a-w "${STAGING_DIRECTORY}"
"${VERIFIER}" --expected-run-id "${RUN_ID}" "${STAGING_DIRECTORY}" \
  >/dev/null

[[ ! -e "${FINAL_DIRECTORY}" && ! -L "${FINAL_DIRECTORY}" ]] || \
  Fail "evidence run appeared during publication: ${RUN_ID}"
mv "${STAGING_DIRECTORY}" "${FINAL_DIRECTORY}"
PUBLISHED=1

"${VERIFIER}" "${FINAL_DIRECTORY}" >/dev/null
echo "${FINAL_DIRECTORY}"
