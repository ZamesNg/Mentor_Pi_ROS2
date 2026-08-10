#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly VERIFIER="${SCRIPT_DIR}/verify_microros_agent_build_container.sh"

Fail() {
  echo "micro-ROS Agent container-build test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  local expected="$1"
  shift
  local output
  if output="$("$@" 2>&1)"; then
    Fail "command unexpectedly succeeded: $*"
  fi
  [[ "${output}" == *"${expected}"* ]] || \
    Fail "failure did not contain '${expected}': ${output}"
}

[[ -x "${VERIFIER}" ]] || Fail "Agent build verifier is not executable"

for architecture in amd64 arm64; do
  image="$(${VERIFIER} --print-default-image \
    --architecture "${architecture}")"
  [[ "${image}" =~ ^mentor-pi/rrclite:humble-${architecture}-[0-9a-f]{16}$ ]] || \
    Fail "${architecture} Agent builder is not the unified project image"
  output="$(${VERIFIER} --architecture "${architecture}" \
    --evidence-id dry-run --dry-run)"
  [[ "${output}" == *"image=${image}"* ]] || \
    Fail "dry run omitted the pinned image"
  [[ "${output}" == *"platform=linux/${architecture}"* ]] || \
    Fail "dry run omitted architecture ${architecture}"
  [[ "${output}" == *"dry-run-${architecture}"* ]] || \
    Fail "dry run omitted its bounded evidence destination"
done

readonly AMD64_IMAGE="$(${VERIFIER} --print-default-image \
  --architecture amd64)"
readonly ARM64_IMAGE="$(${VERIFIER} --print-default-image \
  --architecture arm64)"
[[ "${AMD64_IMAGE}" != "${ARM64_IMAGE}" ]] || \
  Fail "Agent project images must differ by architecture"

ExpectFailure 'architecture must be amd64 or arm64' \
  "${VERIFIER}" --architecture riscv64 --dry-run
ExpectFailure 'evidence ID must be 1-64 safe characters' \
  "${VERIFIER}" --architecture arm64 --evidence-id '../escape' --dry-run
ExpectFailure 'Usage:' "${VERIFIER}" --unknown

bash -n "${VERIFIER}" \
  "${SCRIPT_DIR}/verify_microros_agent_build_in_container.sh"
echo "micro-ROS Agent container-build contract tests passed."
