#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ARTIFACT_VERIFIER="${SCRIPT_DIR}/verify_firmware_artifact.sh"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/firmware-target-debug"
readonly BUILD_IMAGE_PREPARER="${SCRIPT_DIR}/prepare_build_images.sh"
readonly JOB_SELECTOR="${SCRIPT_DIR}/select_build_jobs.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"

Fail() {
  echo "Firmware target CI error: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || Fail "usage: ./tools/run_firmware_target_ci.sh"
[[ -x "${ARTIFACT_VERIFIER}" ]] || Fail "artifact verifier is missing"
if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi

"${ARTIFACT_VERIFIER}" PID "${PROJECT_ROOT}" >/dev/null

command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || Fail "Docker Desktop/Engine is not running"
case "$(uname -m)" in
  x86_64 | amd64) readonly ARCHITECTURE=amd64 ;;
  aarch64 | arm64) readonly ARCHITECTURE=arm64 ;;
  *) Fail "unsupported native architecture" ;;
esac
"${BUILD_IMAGE_PREPARER}" --architecture "${ARCHITECTURE}"
readonly IMAGE="$("${BUILD_IMAGE_PREPARER}" \
  --architecture "${ARCHITECTURE}" --print project)"
readonly IMAGE_ID="$(docker image inspect "${IMAGE}" --format '{{.Id}}' \
  2>/dev/null || true)"
readonly IMAGE_PLATFORM="$(docker image inspect "${IMAGE}" \
  --format '{{.Os}}/{{.Architecture}}' 2>/dev/null || true)"
[[ "${IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ && \
   "${IMAGE_PLATFORM}" == "linux/${ARCHITECTURE}" ]] || \
  Fail "the prepared firmware analysis image identity or platform is invalid"
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"

docker run --rm --network=none \
  --platform "linux/${ARCHITECTURE}" \
  --user "$(id -u):$(id -g)" \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --env "CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  /workspace/tools/run_firmware_target_ci_container.sh

echo "STM32 Debug build and cross-target clang-tidy passed."
echo "Builder image: ${IMAGE_ID} (${IMAGE_PLATFORM})"
echo "Report: ${BUILD_ROOT}/firmware-target-analysis.txt"
