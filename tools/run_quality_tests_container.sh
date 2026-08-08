#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${SCRIPT_DIR}/docker/quality-tests.Dockerfile"
readonly IMAGE="mentor-pi/rrclite-quality-tests:ubuntu-24.04"

Fail() {
  echo "RRCLite quality-test container error: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || \
  Fail "usage: ./tools/run_quality_tests_container.sh"
[[ -f "${DOCKERFILE}" ]] || Fail "quality-test Dockerfile is missing"
command -v docker >/dev/null 2>&1 || Fail "Docker is not installed"
docker info >/dev/null 2>&1 || \
  Fail "Docker Desktop/Engine is not running or is not accessible"

readonly BASE_IMAGE="$(awk '$1 == "FROM" {print $2; exit}' "${DOCKERFILE}")"
[[ "${BASE_IMAGE}" =~ ^ubuntu:24[.]04@sha256:[0-9a-f]{64}$ ]] || \
  Fail "quality-test base must be Ubuntu 24.04 pinned by a sha256 digest"

docker build --file "${DOCKERFILE}" --tag "${IMAGE}" \
  "${PROJECT_ROOT}/tools/docker"

readonly CALLER_UID="$(id -u)"
readonly CALLER_GID="$(id -g)"
[[ "${CALLER_UID}" =~ ^[0-9]+$ ]] || Fail "could not determine caller UID"
[[ "${CALLER_GID}" =~ ^[0-9]+$ ]] || Fail "could not determine caller GID"

docker run --rm --network=none \
  --user "${CALLER_UID}:${CALLER_GID}" \
  --env ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:strict_string_checks=1 \
  --env UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  --env CLANG_FORMAT=clang-format-18 \
  --env HOME=/tmp/rrclite-quality-home \
  --env TMPDIR=/tmp \
  --volume "${PROJECT_ROOT}:/workspace" \
  --workdir /workspace \
  "${IMAGE}" \
  /bin/bash -euc '
    umask 0022
    mkdir -p "${HOME}"
    ./tools/run_native_ci_tests.sh --build-type Debug --sanitizers on
    ./tools/test_gitignore_contract.sh
    ./tools/test_ros_workspace_layout.sh
    ./tools/check_cpp_format.sh
    ./tools/check_framework_docs.py
  '

echo "RRCLite quality tests passed without container network or host toolchain access."
