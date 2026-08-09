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
grep -Fq 'install_onboard_microros_setup.sh' \
  "${SCRIPT_DIR}/prepare_host_build_dependencies.sh" || \
  Fail "the onboard dependency helper does not build pinned micro_ros_setup"
if grep -Fq 'ros-humble-micro-ros-setup' \
    "${SCRIPT_DIR}/prepare_host_build_dependencies.sh"; then
  Fail "the onboard dependency helper still requires the unavailable arm64 Debian package"
fi
grep -Fqx 'commit=58dc7f84851c8c274b253db9779899eb551b1458' \
  "${SCRIPT_DIR}/microros_setup_source.lock" || \
  Fail "the onboard micro_ros_setup source commit is not pinned"
grep -Fq -- '--skip-keys=clang-tidy' \
  "${SCRIPT_DIR}/install_onboard_microros_setup.sh" || \
  Fail "the onboard micro_ros_setup installer does not exclude its unused clang-tidy dependency"
grep -Fq -- '-DBUILD_TESTING=OFF' \
  "${SCRIPT_DIR}/install_onboard_microros_setup.sh" || \
  Fail "the onboard micro_ros_setup installer does not disable upstream tests"
[[ "$("${SCRIPT_DIR}/install_onboard_microros_setup.sh" --print-overlay)" == \
   '/opt/mentor_pi/micro_ros_setup-3.1.3/install/local_setup.bash' ]] || \
  Fail "the onboard micro_ros_setup overlay path changed"

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
grep -Fq 'EXTERNAL_SKIP=clang-tidy' \
  "${SCRIPT_DIR}/build_microros_library.sh" || \
  Fail "native generated-workspace rosdep does not exclude clang-tidy"
grep -Fq 'MICROROS_NATIVE_COLCON_EXECUTABLE=' \
  "${SCRIPT_DIR}/build_microros_library.sh" || \
  Fail "native micro-ROS generation does not preserve the absolute colcon executable"
grep -Fq 'mentor-pi-micro-ros-setup-scripts' \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_library_generation.sh" || \
  Fail "native micro-ROS generation does not use a private upstream launcher"
grep -Fq 'MICROROS_GENERATOR_WORKSPACE' \
  "${PROJECT_ROOT}/firmware/mentor_pi_mcu/config/microros_library_generation.sh" || \
  Fail "micro-ROS generation still assumes only the container workspace"
grep -Fq 'onboard_colcon_state.sh" verify' \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.sh" || \
  Fail "Bash direct colcon runtime does not verify its source state"
grep -Fq 'onboard_colcon_state.sh" verify' \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.zsh" || \
  Fail "zsh direct colcon runtime does not verify its source state"
grep -Fq 'local_setup.zsh' \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.zsh" || \
  Fail "zsh direct colcon runtime does not require the Agent overlay"
grep -Fq 'verify_firmware_artifact.sh" PID' \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.zsh" || \
  Fail "zsh direct colcon runtime does not require PID firmware provenance"
if grep -ERq 'make start' "${PROJECT_ROOT}/docs/tutorials/onboard-computer"; then
  Fail "onboard tutorials must retain their direct ros2 launch"
fi
if grep -ERq 'source [^`]*setup[.]bash|setup_onboard_ros_environment[.]sh' \
    "${PROJECT_ROOT}/docs/tutorials/onboard-computer"; then
  Fail "onboard tutorials contain a user-facing Bash environment command"
fi
if grep -ERqi 'oh[- ]my[- ]zsh|zsh-autosuggestions|zsh-syntax-highlighting|chsh' \
    "${PROJECT_ROOT}/docs/tutorials/onboard-computer" \
    "${SCRIPT_DIR}/prepare_host_build_dependencies.sh"; then
  Fail "onboard setup attempts to replace or enhance the user's zsh configuration"
fi

grep -Fqx $'shell:\n\t@./tools/open_runtime_shell.sh --ros-domain-id "$(ROS_DOMAIN_ID)"' \
  <(grep -A1 '^shell:' "${PROJECT_ROOT}/Makefile") || \
  Fail "make shell no longer uses the adaptive shell helper"
[[ "$(grep -Fc 'exec /usr/bin/zsh -d -i' \
  "${SCRIPT_DIR}/open_runtime_shell.sh")" == "2" ]] || \
  Fail "make shell does not open interactive zsh on both host paths"
grep -Fq 'CMD ["/usr/bin/zsh"]' \
  "${SCRIPT_DIR}/docker/host-runtime.Dockerfile" || \
  Fail "the runtime image default command is not zsh"
grep -Fq '97b27bb2ec0701330b18c2d3e340b22e742b3fa8' \
  "${SCRIPT_DIR}/docker/host-runtime.Dockerfile" || \
  Fail "Oh My Zsh is not pinned to the approved commit"
for package in zsh zsh-autosuggestions zsh-syntax-highlighting; do
  grep -Eq "^[[:space:]]+\"${package}=\\\$\{[^}]+_VERSION\}\"[[:space:]]+\\\\$" \
    "${SCRIPT_DIR}/docker/host-runtime.Dockerfile" || \
    Fail "the runtime image does not install a pinned ${package}"
done
grep -Fq 'plugins=(git sudo)' "${SCRIPT_DIR}/docker/host-runtime.zshrc" || \
  Fail "the runtime zsh plugin contract changed"
grep -Fq 'ZSH_THEME=robbyrussell' "${SCRIPT_DIR}/docker/host-runtime.zshrc" || \
  Fail "the runtime zsh theme contract changed"
grep -Fq 'export ZDOTDIR="${_mentor_pi_user_zdotdir}"' \
  "${SCRIPT_DIR}/zsh/native/.zshrc" || \
  Fail "native make shell does not restore the user's ZDOTDIR"
autosuggestion_line="$(grep -n '^source /usr/share/zsh-autosuggestions/' \
  "${SCRIPT_DIR}/docker/host-runtime.zshrc" | cut -d: -f1)"
highlight_line="$(grep -n '^source /usr/share/zsh-syntax-highlighting/' \
  "${SCRIPT_DIR}/docker/host-runtime.zshrc" | cut -d: -f1)"
[[ -n "${autosuggestion_line}" && -n "${highlight_line}" && \
   "${autosuggestion_line}" -lt "${highlight_line}" ]] || \
  Fail "autosuggestions must load before syntax highlighting"
if tail -n "+$((highlight_line + 1))" \
    "${SCRIPT_DIR}/docker/host-runtime.zshrc" | grep -Eq '^[[:space:]]*source '; then
  Fail "syntax highlighting must be the last sourced zsh integration"
fi
grep -Fq 'org.mentor-pi.host-runtime.zshrc-sha256' \
  "${SCRIPT_DIR}/build_host_runtime_image.sh" || \
  Fail "runtime image reuse is not bound to the tracked zsh configuration"
grep -Fq 'HISTFILE="${HOME}/.zsh_history"' \
  "${SCRIPT_DIR}/docker/host-runtime.zshrc" || \
  Fail "runtime zsh history is not confined to its temporary home"
grep -Fq 'ZSH_CACHE_DIR="${HOME}/.cache/oh-my-zsh"' \
  "${SCRIPT_DIR}/docker/host-runtime.zshrc" || \
  Fail "Oh My Zsh cache is not confined to the temporary home"
grep -Fq -- '--env HOME=/tmp/mentor-pi-home' \
  "${SCRIPT_DIR}/run_runtime.sh" || \
  Fail "the runtime does not provide an ephemeral home"
grep -Fq 'ENV HOME=/tmp/mentor-pi-home' \
  "${SCRIPT_DIR}/docker/host-runtime.Dockerfile" || \
  Fail "the runtime image does not default to an ephemeral home"
grep -Fq -- '--entrypoint /bin/bash' "${SCRIPT_DIR}/run_runtime.sh" || \
  Fail "the non-interactive runtime entrypoint must remain Bash"

bash -n "${SCRIPT_DIR}/open_runtime_shell.sh" \
  "${SCRIPT_DIR}/setup_onboard_ros_environment.sh" \
  "${SCRIPT_DIR}/build_host_runtime_image.sh" \
  "${SCRIPT_DIR}/install_onboard_microros_setup.sh"
zsh -n "${SCRIPT_DIR}/setup_onboard_ros_environment.zsh" \
  "${SCRIPT_DIR}/docker/host-runtime.zshrc" \
  "${SCRIPT_DIR}/zsh/native/.zshrc"

if "${SCRIPT_DIR}/setup_onboard_ros_environment.sh" >/dev/null 2>&1; then
  Fail "onboard environment helper succeeded when executed instead of sourced"
fi
if zsh "${SCRIPT_DIR}/setup_onboard_ros_environment.zsh" >/dev/null 2>&1; then
  Fail "zsh onboard environment helper succeeded when executed instead of sourced"
fi

echo "Native onboard tooling contract tests passed."
