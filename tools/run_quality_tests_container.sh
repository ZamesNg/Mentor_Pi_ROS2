#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
readonly JOB_SELECTOR="${SCRIPT_DIR}/select_build_jobs.sh"
readonly BUILD_IMAGE_PREPARER="${SCRIPT_DIR}/prepare_build_images.sh"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"

if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi

profile=""

Fail() {
  echo "RRCLite quality-test container error: $*" >&2
  exit 1
}

[[ "$#" -eq 2 && "$1" == --profile ]] || \
  Fail "usage: ./tools/run_quality_tests_container.sh --profile rdk|full"
profile="$2"
[[ "${profile}" == rdk || "${profile}" == full ]] || \
  Fail "profile must be rdk or full"
[[ -f "${DOCKERFILE}" ]] || Fail "quality-test Dockerfile is missing"
command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || \
  Fail "Docker Desktop/Engine is not running or is not accessible"

readonly BASE_IMAGE="$(awk '$1 == "FROM" {print $2; exit}' "${DOCKERFILE}")"
[[ "${BASE_IMAGE}" =~ ^ubuntu:24[.]04@sha256:[0-9a-f]{64}$ ]] || \
  Fail "quality-test base must be Ubuntu 24.04 pinned by a sha256 digest"

case "$(uname -m)" in
  x86_64 | amd64) readonly ARCHITECTURE=amd64 ;;
  aarch64 | arm64) readonly ARCHITECTURE=arm64 ;;
  *) Fail "unsupported host architecture" ;;
esac
if [[ "${profile}" == rdk ]]; then
  "${BUILD_IMAGE_PREPARER}" --architecture "${ARCHITECTURE}"
  IMAGE="$("${BUILD_IMAGE_PREPARER}" \
    --architecture "${ARCHITECTURE}" --print firmware)"
else
  "${BUILD_IMAGE_PREPARER}" --architecture "${ARCHITECTURE}" --include-quality
  IMAGE="$("${BUILD_IMAGE_PREPARER}" \
    --architecture "${ARCHITECTURE}" --print quality)"
fi
readonly IMAGE
readonly BUILD_JOBS="$("${JOB_SELECTOR}")"

readonly CALLER_UID="$(id -u)"
readonly CALLER_GID="$(id -g)"
[[ "${CALLER_UID}" =~ ^[0-9]+$ ]] || Fail "could not determine caller UID"
[[ "${CALLER_GID}" =~ ^[0-9]+$ ]] || Fail "could not determine caller GID"

docker run --rm --network=none \
  --platform "linux/${ARCHITECTURE}" \
  --user "${CALLER_UID}:${CALLER_GID}" \
  --env ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:strict_string_checks=1 \
  --env UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  --env CLANG_FORMAT=clang-format-18 \
  --env HOME=/tmp/rrclite-quality-home \
  --env TMPDIR=/tmp \
  --env "RRCLITE_BUILD_JOBS=${BUILD_JOBS}" \
  --env RRCLITE_BUILD_LOCK_HELD=1 \
  --env "CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}" \
  --env "RRCLITE_QUALITY_PROFILE=${profile}" \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  /bin/bash -euc '
    umask 0022
    mkdir -p "${HOME}"
    ./tools/check_framework_docs.py
    ./tools/run_native_ci_tests.sh --build-type Debug --sanitizers on
    ./tools/run_native_ci_tests.sh --build-type Release --sanitizers off
    ./tools/run_tsan_tests.sh
    if [[ "${RRCLITE_QUALITY_PROFILE}" == full ]]; then
      ./tools/run_coverage_tests.sh
      ./tools/check_cpp_format.sh
      CLANG_TIDY=clang-tidy-18 RUN_CLANG_TIDY=run-clang-tidy-18 \
        ./tools/run_clang_tidy.sh
    fi
  '

echo "RRCLite ${profile} quality tests passed without container network or host toolchain access."
