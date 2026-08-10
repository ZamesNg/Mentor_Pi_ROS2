#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PULL_TOOL="${SCRIPT_DIR}/pull_pinned_build_images.sh"
readonly IMAGE_SELECTOR="${SCRIPT_DIR}/select_pinned_build_image.sh"

Fail() {
  echo "pinned-image pull test failure: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

[[ -x "${PULL_TOOL}" ]] || Fail "pinned-image pull tool is missing"
[[ -x "${IMAGE_SELECTOR}" ]] || Fail "pinned-image selector is missing"

for architecture in amd64 arm64; do
  output="$(${PULL_TOOL} --dry-run --profile normal --architecture "${architecture}")"
  pull_count="$(grep -Fc "docker pull --platform linux/${architecture}" \
    <<<"${output}")"
  [[ "${pull_count}" == "2" ]] || \
    Fail "expected two unique ${architecture} image pulls, got ${pull_count}"
  [[ "${output}" != *"docker pull --platform linux/amd64"* || \
     "${architecture}" == "amd64" ]] || \
    Fail "arm64 plan contains an amd64 pull"
  [[ "${output}" != *"docker pull --platform linux/arm64"* || \
     "${architecture}" == "arm64" ]] || \
    Fail "amd64 plan contains an arm64 pull"
  [[ "${output}" == *"microros/micro_ros_static_library_builder:humble@sha256:"* ]] || \
    Fail "${architecture} plan omits the unified Humble base"
  [[ "${output}" == *"prepare_build_images.sh --architecture ${architecture} --include-quality"* ]] || \
    Fail "${architecture} normal-computer setup does not prepare the quality image"
  [[ "${output}" == *"TARGETARCH=${architecture}"* ]] || \
    Fail "${architecture} setup plan does not forward the Docker build architecture"
  [[ "${output}" == *"image pull plan is valid for linux/${architecture}"* ]] || \
    Fail "${architecture} dry-run completion message is missing"
  [[ "${output}" != *'sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4'* && \
     "${output}" != *'sha256:e291f74890e81b31eb1d70731cb79b2d767dd585269325031effc72952b24b9d'* && \
     "${output}" != *'sha256:561618e2c15bf2397621dd04f96926663a3b5616c189cf7e38db7e82f5c538ea'* ]] || \
    Fail "${architecture} pull plan uses a multi-platform index digest"
done

rdk_output="$(${PULL_TOOL} --dry-run --profile rdk-x5 --architecture arm64)"
[[ "$(grep -Fc 'docker pull --platform linux/arm64' <<<"${rdk_output}")" == 1 ]] || \
  Fail "RDK setup must omit the normal-computer quality image"
[[ "${rdk_output}" != *'--include-quality'* ]] || \
  Fail "RDK setup unexpectedly prepares the full quality image"
[[ "${rdk_output}" == *'Pinned RRCLite rdk-x5 image pull plan is valid'* ]] || \
  Fail "RDK dry-run completion message is missing"
[[ "${rdk_output}" == *'TARGETARCH=arm64'* ]] || \
  Fail "RDK dry-run does not forward TARGETARCH=arm64"

grep -Fq -- '--build-arg "TARGETARCH=${architecture}"' \
  "${SCRIPT_DIR}/prepare_build_images.sh" || \
  Fail "prepared Docker builds do not pass TARGETARCH explicitly"
grep -Fqx 'ARG TARGETARCH' "${SCRIPT_DIR}/docker/rrclite.Dockerfile" || \
  Fail "unified Dockerfile does not declare TARGETARCH"
[[ "$(grep -Fc 'arm-gnu-toolchain-13.2.rel1' \
  "${SCRIPT_DIR}/docker/rrclite.Dockerfile")" == 1 ]] || \
  Fail "unified Dockerfile must contain one Arm toolchain download"

readonly AMD64_AGENT_IMAGE="$("${IMAGE_SELECTOR}" microros amd64)"
readonly ARM64_AGENT_IMAGE="$("${IMAGE_SELECTOR}" microros arm64)"
readonly AMD64_UBUNTU_IMAGE="$("${IMAGE_SELECTOR}" ubuntu amd64)"
readonly ARM64_UBUNTU_IMAGE="$("${IMAGE_SELECTOR}" ubuntu arm64)"
[[ "${AMD64_AGENT_IMAGE}" == \
   'microros/micro_ros_static_library_builder:humble@sha256:8dbeecd73df7a36327259321596755eebda27c1c760eded49720745bf909516a' && \
   "${ARM64_AGENT_IMAGE}" == \
   'microros/micro_ros_static_library_builder:humble@sha256:460b3ea2cd41d6256e5f09f9e7bf543f63a04890719abcc10d58acad12f33fa7' && \
   "${AMD64_UBUNTU_IMAGE}" == \
   'ubuntu:24.04@sha256:019e8eb29a85e74d64925745884f2ec79aa27e3feab36353d24656f4d6b89467' && \
   "${ARM64_UBUNTU_IMAGE}" == \
   'ubuntu:24.04@sha256:b17516cd982bf06bdd5d5600253d12a8de017b9eb831cc052b532a0363d294f9' ]] || \
  Fail "the reviewed architecture-specific image map changed"
[[ "${AMD64_AGENT_IMAGE}" != "${ARM64_AGENT_IMAGE}" ]] || \
  Fail "architecture-specific child manifests must be distinct"
ExpectFailure "${IMAGE_SELECTOR}" microros riscv64
ExpectFailure "${IMAGE_SELECTOR}" unknown arm64
ExpectFailure "${IMAGE_SELECTOR}" microros

ExpectFailure "${PULL_TOOL}" --dry-run
ExpectFailure "${PULL_TOOL}" --architecture
ExpectFailure "${PULL_TOOL}" --architecture riscv64 --dry-run
ExpectFailure "${PULL_TOOL}" --architecture amd64 --profile rdk-x5 --dry-run
ExpectFailure "${PULL_TOOL}" --architecture arm64 --profile invalid --dry-run
ExpectFailure "${PULL_TOOL}" --architecture amd64 --unknown

echo "Pinned-image architecture and dry-run tests passed."
