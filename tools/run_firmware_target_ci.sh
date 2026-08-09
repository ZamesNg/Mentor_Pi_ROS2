#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${SCRIPT_DIR}/docker/firmware-analysis.Dockerfile"
readonly ARTIFACT_VERIFIER="${SCRIPT_DIR}/verify_firmware_artifact.sh"
readonly IMAGE="mentor-pi/rrclite-firmware-analysis:clang-18-gcc-13.2.1"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/firmware-target-debug"

Fail() {
  echo "Firmware target CI error: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || Fail "usage: ./tools/run_firmware_target_ci.sh"
[[ -f "${DOCKERFILE}" ]] || Fail "analysis Dockerfile is missing"
[[ -x "${ARTIFACT_VERIFIER}" ]] || Fail "artifact verifier is missing"

"${ARTIFACT_VERIFIER}" PID "${PROJECT_ROOT}" >/dev/null

command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker Desktop/Engine is not running"

docker build --file "${DOCKERFILE}" --tag "${IMAGE}" \
  "${PROJECT_ROOT}/tools/docker"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  /workspace/tools/run_firmware_target_ci_container.sh

echo "STM32 Debug build and cross-target clang-tidy passed."
echo "Report: ${BUILD_ROOT}/firmware-target-analysis.txt"
