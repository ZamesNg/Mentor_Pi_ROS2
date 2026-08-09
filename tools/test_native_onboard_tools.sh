#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() {
  echo "Native onboard tooling test failure: $*" >&2
  exit 1
}

plan="$(${SCRIPT_DIR}/bootstrap_native_arm_toolchain.sh --print-plan)"
grep -Fqx 'version=13.2.1' <<<"${plan}" || Fail "toolchain version is not pinned"
case "$(uname -m)" in
  x86_64 | amd64)
    grep -Fqx 'host=x86_64' <<<"${plan}" || Fail "amd64 toolchain host is wrong"
    grep -Fqx 'sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb' \
      <<<"${plan}" || Fail "amd64 toolchain hash is wrong"
    ;;
  aarch64 | arm64)
    grep -Fqx 'host=aarch64' <<<"${plan}" || Fail "arm64 toolchain host is wrong"
    grep -Fqx 'sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a' \
      <<<"${plan}" || Fail "arm64 toolchain hash is wrong"
    ;;
  *) Fail "test host architecture is unsupported" ;;
esac

"${SCRIPT_DIR}/install_onboard_stm32cubeprogrammer.sh" --verify-archive \
  >/dev/null
grep -Fq 'install_onboard_stm32cubeprogrammer.sh' \
  "${SCRIPT_DIR}/prepare_host_build_dependencies.sh" || \
  Fail "the onboard dependency helper does not install STM32CubeProgrammer"

grep -Fq 'bootstrap_native_arm_toolchain.sh' "${PROJECT_ROOT}/Makefile" || \
  Fail "make setup does not prepare the native toolchain"
grep -Fq 'native-ubuntu-22.04' "${SCRIPT_DIR}/build_firmware.sh" || \
  Fail "firmware metadata does not distinguish the native builder"
if grep -Eq 'RRCLITE_BUILD_LOCAL|native-explicit' \
    "${SCRIPT_DIR}/build_firmware.sh" \
    "${SCRIPT_DIR}/verify_firmware_artifact.sh" \
    "${SCRIPT_DIR}/package_board_handoff.sh"; then
  Fail "unsupported local firmware builds remain release-capable"
fi
grep -Fq 'MICROROS_RESTORE_OWNERSHIP=0' \
  "${SCRIPT_DIR}/build_microros_library.sh" || \
  Fail "native micro-ROS generation is not selected"
grep -Fq 'MICROROS_GENERATOR_WORKSPACE' \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_library_generation.sh" || \
  Fail "micro-ROS generation still assumes only the container workspace"
grep -Fq 'onboard_colcon_state.sh" verify' \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.sh" || \
  Fail "direct colcon runtime does not verify its source state"
if grep -ERq 'make start' "${PROJECT_ROOT}/docs/tutorials/onboard-computer"; then
  Fail "onboard tutorials must retain their direct ros2 launch"
fi

if "${SCRIPT_DIR}/setup_onboard_ros_environment.sh" >/dev/null 2>&1; then
  Fail "onboard environment helper succeeded when executed instead of sourced"
fi

echo "Native onboard tooling contract tests passed."
