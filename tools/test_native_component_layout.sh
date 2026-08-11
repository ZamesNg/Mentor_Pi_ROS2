#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

Fail() { echo "Native component layout failure: $*" >&2; exit 1; }

for component in firmware micro_ros_agent ros2_ws; do
  [[ -f "${PROJECT_ROOT}/${component}/Makefile" ]] || \
    Fail "${component} has no component Makefile"
done
[[ -x "${PROJECT_ROOT}/micro_ros_agent/tools/find_device.sh" ]] || \
  Fail "CH9102F identity-based discovery helper is missing"
[[ -x "${PROJECT_ROOT}/firmware/tools/build_boot_control.sh" && \
   -f "${PROJECT_ROOT}/firmware/mentor_pi_mcu/host/ch9102_boot_control.cc" ]] || \
  Fail "firmware CH9102F automatic boot-control tooling is missing"
grep -Fqx 'AUTOMATIC_BOOT_CONTROL ?= 1' \
  "${PROJECT_ROOT}/firmware/Makefile" || \
  Fail "firmware automatic boot control is not the default"
[[ ! -e "${PROJECT_ROOT}/mentor_pi_ros2" && \
   ! -L "${PROJECT_ROOT}/mentor_pi_ros2" ]] || \
  Fail "the legacy mentor_pi_ros2 path exists"
[[ ! -e "${PROJECT_ROOT}/.gitmodules" ]] || \
  Fail "component Git submodules are forbidden"

mapfile -t packages < <(
  find "${PROJECT_ROOT}/ros2_ws/src" -mindepth 2 -maxdepth 2 \
    -name package.xml -printf '%h\n' | sed 's|.*/||' | sort
)
readonly -a expected_packages=(
  mentor_pi_bringup
  mentor_pi_hardwares
  mentor_pi_interfaces
  mentor_pi_tracking
  mentor_pi_tracking_interfaces
)
[[ "${packages[*]}" == "${expected_packages[*]}" ]] || \
  Fail "ros2_ws/src does not contain exactly the five project packages"
grep -Fq -- '--base-paths src' "${PROJECT_ROOT}/ros2_ws/tools/colcon.sh" || \
  Fail "colcon is not explicitly limited to ros2_ws/src"
[[ -f "${PROJECT_ROOT}/ros2_ws/third_party/COLCON_IGNORE" ]] || \
  Fail "plain colcon discovery does not ignore third-party sources"

[[ -f "${PROJECT_ROOT}/firmware/mentor_pi_mcu/sdk/humble/libmicroros.tar.xz" && \
   -f "${PROJECT_ROOT}/firmware/mentor_pi_mcu/sdk/humble/manifest.txt" ]] || \
  Fail "the checked firmware SDK is incomplete"
"${PROJECT_ROOT}/tools/check_compatibility.sh" >/dev/null
if rg -n 'ros2_ws/src/mentor_pi_interfaces' \
    "${PROJECT_ROOT}/firmware/mentor_pi_mcu/CMakeLists.txt" \
    "${PROJECT_ROOT}/firmware/mentor_pi_mcu/target/stm32/CMakeLists.txt" \
    >/dev/null; then
  Fail "firmware CMake still directly includes the ROS workspace"
fi

mapfile -t dockerfiles < <(
  find "${PROJECT_ROOT}" \( -path "${PROJECT_ROOT}/.git" -o \
    -path "${PROJECT_ROOT}/docs/reference" -o -name build -o \
    -name third_party \) -prune -o \
    -type f \( -iname 'Dockerfile' -o -iname '*.Dockerfile' \) -print
)
[[ "${#dockerfiles[@]}" == 1 && \
   "${dockerfiles[0]}" == "${PROJECT_ROOT}/.devcontainer/Dockerfile" ]] || \
  Fail ".devcontainer/Dockerfile must be the sole Dockerfile"
[[ -f "${PROJECT_ROOT}/.devcontainer/devcontainer.json" ]] || \
  Fail "VS Code Dev Container configuration is missing"
if grep -Fq 'ros-humble-micro-ros-setup' \
    "${PROJECT_ROOT}/.devcontainer/Dockerfile"; then
  Fail "the unavailable Humble micro-ROS setup apt package is requested"
fi
grep -Fq 'ENTRYPOINT []' "${PROJECT_ROOT}/.devcontainer/Dockerfile" || \
  Fail "the Dev Container inherits the builder image entrypoint"
grep -Fq '/uros_ws/install/setup.sh' \
    "${PROJECT_ROOT}/.devcontainer/Dockerfile" || \
  Fail "the Dev Container does not activate the bundled micro-ROS overlay"
bash "${PROJECT_ROOT}/.devcontainer/test/contract_test.sh" >/dev/null
[[ ! -e "${PROJECT_ROOT}/docker" && ! -e "${PROJECT_ROOT}/tools/docker" ]] || \
  Fail "legacy Docker directories remain"
for obsolete_path in \
    thirdpart \
    firmware/mentor_pi_mcu/config/microros_artifact.sha256 \
    firmware/mentor_pi_mcu/config/microros_artifact_tree.sha256 \
    tools/install_onboard_stm32cubeprogrammer.sh \
    tools/rrclite_characterization.gdb \
    tools/run_native_ci_tests.sh \
    tools/select_build_jobs.sh; do
  [[ ! -e "${PROJECT_ROOT}/${obsolete_path}" && \
     ! -L "${PROJECT_ROOT}/${obsolete_path}" ]] || \
    Fail "obsolete refactor artifact remains: ${obsolete_path}"
done
if rg -n --hidden \
    --glob '!docs/framework/adr/0002-docker-everywhere-host-runtime.md' \
    --glob '!tools/test_native_component_layout.sh' \
    --glob '!.devcontainer/**' --glob '!build/**' --glob '!.git/**' \
    'docker (run|build|load|pull|exec)|qemu-user-static|binfmt|OCI archive|make rdk-handoff' \
    "${PROJECT_ROOT}" >/dev/null; then
  Fail "an active production Docker, OCI, or QEMU workflow remains"
fi

if grep -Eq '^(start|host|firmware|agent|build|shell):' \
    "${PROJECT_ROOT}/Makefile"; then
  Fail "the root Makefile exposes a component build/runtime target"
fi
if rg -n '/dev/tty(USB|ACM)[0-9]+' \
    "${PROJECT_ROOT}/README.md" \
    "${PROJECT_ROOT}/docs/tutorials" \
    "${PROJECT_ROOT}/micro_ros_agent/README.md" \
    "${PROJECT_ROOT}/micro_ros_agent/Makefile" >/dev/null; then
  Fail "operator instructions assume a transient Linux tty number"
fi
for target in doctor check-compatibility find-device passive-check characterize-board \
    qualification-preflight; do
  grep -Eq "^${target}:" "${PROJECT_ROOT}/Makefile" || \
    Fail "root integration target is missing: ${target}"
done

readonly -a host_tutorials=(
  01-prerequisites-and-safety.md 02-firmware-setup-and-build.md
  03-passive-hardware-checks.md 04-firmware-flash.md 05-ros-environment.md
  06-ros-apps-build-and-test.md 07-connect-and-run.md
  08-evidence-and-qualification.md
)
readonly -a onboard_tutorials=(
  01-prerequisites-and-safety.md 02-firmware-setup-and-build.md
  03-passive-checks-and-flash.md 04-agent-build.md
  05-agent-service-installation.md 06-ros-apps-build-and-test.md
  07-integrated-runtime-and-recovery.md 08-evidence-and-qualification.md
)
mapfile -t actual_host < <(find "${PROJECT_ROOT}/docs/tutorials/host" \
  -maxdepth 1 -type f -printf '%f\n' | sort)
mapfile -t actual_onboard < <(find "${PROJECT_ROOT}/docs/tutorials/onboard" \
  -maxdepth 1 -type f -printf '%f\n' | sort)
[[ "${actual_host[*]}" == "${host_tutorials[*]}" ]] || \
  Fail "host tutorial track is not the exact ordered 01-08 set"
[[ "${actual_onboard[*]}" == "${onboard_tutorials[*]}" ]] || \
  Fail "onboard tutorial track is not the exact ordered 01-08 set"
[[ ! -e "${PROJECT_ROOT}/docs/tutorials/host_computer" && \
   ! -e "${PROJECT_ROOT}/docs/tutorials/rdk_deploy" ]] || \
  Fail "legacy tutorial trees remain"

readonly CONTROLLER_LAUNCH="${PROJECT_ROOT}/ros2_ws/src/mentor_pi_bringup/launch/controller.launch.py"
if rg -n 'micro_ros_agent|agent_executable|serial_device|ExecuteProcess' \
    "${CONTROLLER_LAUNCH}" >/dev/null; then
  Fail "controller.launch.py still owns the Agent"
fi
grep -Fq 'configuration_supervisor' "${CONTROLLER_LAUNCH}" || \
  Fail "controller.launch.py does not own the supervisor"
if rg -n 'start_bringup|IfCondition' \
    "${PROJECT_ROOT}/ros2_ws/src/mentor_pi_hardwares/launch" >/dev/null; then
  Fail "vehicle launch can bypass the fail-coupled supervisor"
fi
if rg -n 'mentor_pi_bringup|configuration_supervisor|vehicle.launch' \
    "${PROJECT_ROOT}/micro_ros_agent/systemd/mentor-pi-agent.service" >/dev/null; then
  Fail "the Agent service starts ROS applications"
fi
for native_only_script in \
    "${PROJECT_ROOT}/micro_ros_agent/tools/install_service.sh" \
    "${PROJECT_ROOT}/ros2_ws/tools/run.sh" \
    "${PROJECT_ROOT}/tools/install_evidence_tools.sh" \
    "${PROJECT_ROOT}/tools/run_runtime_action.sh"; do
  grep -Fq '/.dockerenv' "${native_only_script}" || \
    Fail "native-only operation does not reject the Dev Container: ${native_only_script}"
done

echo "Native component, path, Docker-removal, and tutorial contracts passed."
