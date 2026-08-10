#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly DOCKERFILE="${SCRIPT_DIR}/docker/rrclite.Dockerfile"
readonly ROS_LOCK="${SCRIPT_DIR}/docker/ros-humble-packages.lock"
readonly IMAGE_FINGERPRINT="${SCRIPT_DIR}/docker_image_source_fingerprint.sh"

Fail() {
  echo "Unified-image policy test failure: $*" >&2
  exit 1
}

for obsolete in firmware-builder.Dockerfile microros-builder.Dockerfile \
    host-runtime.Dockerfile; do
  [[ ! -e "${SCRIPT_DIR}/docker/${obsolete}" ]] || \
    Fail "obsolete Dockerfile remains: ${obsolete}"
done
[[ ! -e "${SCRIPT_DIR}/build_host_runtime_image.sh" ]] || \
  Fail "obsolete host-runtime image builder remains"
[[ -x "${IMAGE_FINGERPRINT}" ]] || Fail "image fingerprint tool is unavailable"

[[ "$(rg -c '^RUN case "\$\{TARGETARCH\}"' "${DOCKERFILE}")" == 1 ]] || \
  Fail "Arm GNU architecture selection must occur exactly once"
[[ "$(rg -c 'toolchain_archive="arm-gnu-toolchain-13\.2\.rel1' \
    "${DOCKERFILE}")" == 1 ]] || \
  Fail "Arm GNU download must occur exactly once"
rg -Fq 'snapshots.ros.org/humble/${ROS_SNAPSHOT_DATE}/ubuntu' \
  "${DOCKERFILE}" || Fail "signed Humble snapshot source is absent"

readonly expected_lock="$(cat <<'EOF'
ROS_SNAPSHOT_DATE=2026-08-07

AMD64_ROS2_CONTROL_VERSION=2.54.0-1jammy.20260804.213423
AMD64_ROS2_CONTROLLERS_VERSION=2.53.3-1jammy.20260804.212633
AMD64_MECANUM_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211244
AMD64_ACKERMANN_CONTROLLER_VERSION=2.53.3-1jammy.20260804.211824
AMD64_FOXGLOVE_BRIDGE_VERSION=3.4.3-2jammy.20260726.140144
AMD64_XACRO_VERSION=2.1.1-1jammy.20260304.195513

ARM64_ROS2_CONTROL_VERSION=2.54.0-1jammy.20260805.082815
ARM64_ROS2_CONTROLLERS_VERSION=2.53.3-1jammy.20260805.002729
ARM64_MECANUM_CONTROLLER_VERSION=2.53.3-1jammy.20260804.235733
ARM64_ACKERMANN_CONTROLLER_VERSION=2.53.3-1jammy.20260805.000825
ARM64_FOXGLOVE_BRIDGE_VERSION=3.4.3-2jammy.20260725.213251
ARM64_XACRO_VERSION=2.1.1-1jammy.20260307.132851
EOF
)"
[[ "$(cat "${ROS_LOCK}")" == "${expected_lock}" ]] || \
  Fail "ROS package lock differs from the approved architecture matrix"

for name in rrclite.Dockerfile host-runtime.zshrc ros-humble-packages.lock \
    altro_source.lock microros_agent_source.lock microros_sources.lock; do
  rg -Fq "${name}" "${SCRIPT_DIR}/prepare_build_images.sh" || \
    Fail "project image identity omits ${name}"
done

readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "${TEST_ROOT}"' EXIT
for clone in first second; do
  for relative in tools/docker/rrclite.Dockerfile \
      tools/docker/host-runtime.zshrc \
      tools/docker/quality-tests.Dockerfile \
      tools/docker/ros-humble-packages.lock tools/altro_source.lock \
      tools/microros_agent_source.lock \
      firmware/mentor_pi_mcu/config/microros_sources.lock; do
    mkdir -p "${TEST_ROOT}/${clone}/$(dirname "${relative}")"
    cp "${PROJECT_ROOT}/${relative}" "${TEST_ROOT}/${clone}/${relative}"
  done
done
readonly test_base='example.invalid/base@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'
readonly first_fingerprint="$(
  "${IMAGE_FINGERPRINT}" project "${test_base}" "${TEST_ROOT}/first"
)"
readonly second_fingerprint="$(
  "${IMAGE_FINGERPRINT}" project "${test_base}" "${TEST_ROOT}/second"
)"
[[ "${first_fingerprint}" == "${second_fingerprint}" ]] || \
  Fail "image identity depends on the checkout path"
readonly first_quality_fingerprint="$(
  "${IMAGE_FINGERPRINT}" quality "${test_base}" "${TEST_ROOT}/first"
)"
readonly second_quality_fingerprint="$(
  "${IMAGE_FINGERPRINT}" quality "${test_base}" "${TEST_ROOT}/second"
)"
[[ "${first_quality_fingerprint}" == "${second_quality_fingerprint}" ]] || \
  Fail "quality-image identity depends on the checkout path"
printf '\n# changed\n' >>"${TEST_ROOT}/second/tools/altro_source.lock"
[[ "$("${IMAGE_FINGERPRINT}" project "${test_base}" \
      "${TEST_ROOT}/second")" != "${first_fingerprint}" ]] || \
  Fail "image identity ignores input content changes"

if rg -n 'firmware-builder|microros-builder|host-runtime\.Dockerfile|build_host_runtime_image' \
    "${PROJECT_ROOT}/Makefile" "${PROJECT_ROOT}/tools" \
    --glob '!test_unified_image_policy.sh' >/dev/null; then
  Fail "an active path still names an obsolete project image role"
fi

echo "Unified project-image and ROS lock policies passed."
