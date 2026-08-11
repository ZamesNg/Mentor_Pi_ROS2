#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() { echo "Tutorial action contract failure: $*" >&2; exit 1; }

for command in \
    'make -C firmware setup' 'make -C firmware build' \
    'make -C ros2_ws build'; do
  rg -Fq "${command}" "${PROJECT_ROOT}/docs/tutorials/host" || \
    Fail "host tutorial track omits ${command}"
done
for command in \
    'make -C firmware build' 'make -C micro_ros_agent build' \
    'make -C micro_ros_agent install-service' 'make -C ros2_ws build' \
    'systemctl is-active mentor-pi-agent.service'; do
  rg -Fq "${command}" "${PROJECT_ROOT}/docs/tutorials/onboard" || \
    Fail "onboard tutorial track omits ${command}"
done
rg -Fq 'VS Code Dev Container' "${PROJECT_ROOT}/docs/tutorials/host" || \
  Fail "host track omits the cross-platform Dev Container boundary"
rg -Fq 'Dev Container' "${PROJECT_ROOT}/docs/tutorials/onboard/01-prerequisites-and-safety.md" && \
  grep -Fq 'Do not use' \
    "${PROJECT_ROOT}/docs/tutorials/onboard/01-prerequisites-and-safety.md" || \
  Fail "onboard track does not reject Dev Container runtime"

grep -Fq 'DEFAULT_EVIDENCE_ROOT="/var/log/mentor-pi/actions"' \
  "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "native production evidence root is missing"
grep -Fq 'DEFAULT_CAPTURE_TOOL="/opt/mentor_pi/tools/capture_board_diagnostics"' \
  "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "installed native capture tool is not the production default"
grep -Fq 'PACKAGED_FIRMWARE_SHA256' "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "production actions do not bind a packaged firmware hash"
grep -Fq 'production firmware identity does not match the verified PID artifact' \
  "${SCRIPT_DIR}/run_runtime_action.sh" || \
  Fail "production actions do not compare the claimed and verified firmware hashes"
grep -Fq '/var/log/mentor-pi/actions' "${SCRIPT_DIR}/install_evidence_tools.sh" || \
  Fail "evidence installer does not create the production log tree"

readonly CONTROLLER_LAUNCH="${PROJECT_ROOT}/ros2_ws/src/mentor_pi_bringup/launch/controller.launch.py"
grep -Fq 'RRCLITE_RUNTIME_ACK' "${CONTROLLER_LAUNCH}" || \
  Fail "manual supervisor launch lacks its safety acknowledgement"
grep -Fq 'OnProcessExit' "${CONTROLLER_LAUNCH}" || \
  Fail "manual launch no longer shuts down on supervisor exit"
PYTHONPYCACHEPREFIX="${TEST_ROOT}/pycache" python3 -m py_compile \
  "${CONTROLLER_LAUNCH}" \
  "${PROJECT_ROOT}"/ros2_ws/src/*/launch/*.py

output=""
if output="$("${SCRIPT_DIR}/tutorial_action.sh" unsupported-action \
    </dev/null 2>&1)"; then
  Fail "unsupported tutorial action unexpectedly succeeded"
fi
[[ "${output}" == *'unsupported tutorial action'* ]] || \
  Fail "unsupported tutorial action did not fail precisely"

mkdir -p "${TEST_ROOT}/tools"
cp "${SCRIPT_DIR}/tutorial_action.sh" "${TEST_ROOT}/tools/tutorial_action.sh"
printf '%s\n' '#!/usr/bin/env bash' 'printf "%s\t%s\n" "${ROS_DOMAIN_ID}" "$*" >"${FAKE_RUNTIME_LOG}"' \
  >"${TEST_ROOT}/tools/run_runtime_action.sh"
chmod +x "${TEST_ROOT}/tools/"*.sh
ROS_DOMAIN_ID=37 PERIPHERAL_SMOKE_ACK=PASSIVE_OUTPUTS_GUARDED OLED_PRESENT=1 \
FAKE_RUNTIME_LOG="${TEST_ROOT}/runtime.log" \
  "${TEST_ROOT}/tools/tutorial_action.sh" peripheral-smoke
grep -Fqx $'37\tperipheral-smoke 1' "${TEST_ROOT}/runtime.log" || \
  Fail "tutorial action did not forward the ROS domain and passive arguments"

echo "Native tutorial, launch, and evidence action contracts passed."
