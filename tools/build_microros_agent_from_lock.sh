#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCE_LOCK="${SCRIPT_DIR}/microros_agent_source.lock"
readonly STATE_VALIDATOR="${SCRIPT_DIR}/verify_microros_agent_install_state.sh"

mode=""
work_root=""
dependency_mode=""

Fail() {
  echo "micro-ROS Agent source build failed: $*" >&2
  exit 1
}

Usage() {
  cat >&2 <<'EOF'
Usage: build_microros_agent_from_lock.sh fetch --work-root ABSOLUTE_PATH
       build_microros_agent_from_lock.sh build --work-root ABSOLUTE_PATH \
         --dependency-mode preinstalled|install
EOF
  exit 2
}

ReadSingleValue() {
  local key="$1"
  local count
  local line
  count="$(grep -Ec "^${key}=" "${SOURCE_LOCK}" || true)"
  [[ "${count}" == "1" ]] || \
    Fail "Agent source lock must contain exactly one ${key}= entry"
  line="$(grep -E "^${key}=" "${SOURCE_LOCK}")"
  printf '%s' "${line#*=}"
}

[[ "$#" -ge 1 ]] || Usage
mode="$1"
shift
while (($# > 0)); do
  case "$1" in
    --work-root)
      (($# >= 2)) || Usage
      work_root="$2"
      shift 2
      ;;
    --dependency-mode)
      (($# >= 2)) || Usage
      dependency_mode="$2"
      shift 2
      ;;
    *) Usage ;;
  esac
done
[[ "${mode}" == "fetch" || "${mode}" == "build" ]] || Usage
[[ "${work_root}" == /* && "${work_root}" != "/" && \
   "${work_root}" != *$'\n'* ]] || \
  Fail "work root must be an absolute, non-root path without newlines"
if [[ "${mode}" == "fetch" ]]; then
  [[ -z "${dependency_mode}" ]] || \
    Fail "fetch mode does not accept a dependency mode"
else
  [[ "${dependency_mode}" == "preinstalled" || \
     "${dependency_mode}" == "install" ]] || \
    Fail "build dependency mode must be preinstalled or install"
fi

[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] || \
  Fail "Agent source lock is missing or symbolic"
[[ -x "${STATE_VALIDATOR}" ]] || Fail "Agent source validator is unavailable"
[[ "${ROS_DISTRO:-}" == "humble" ]] || \
  Fail "the loaded ROS environment must identify ROS_DISTRO=humble"

readonly LOCK_FORMAT="$(ReadSingleValue format)"
readonly LOCK_ROS_DISTRO="$(ReadSingleValue ros_distro)"
readonly AGENT_REPOSITORY="$(ReadSingleValue agent_repository)"
readonly AGENT_COMMIT="$(ReadSingleValue agent_commit)"
readonly MSGS_REPOSITORY="$(ReadSingleValue messages_repository)"
readonly MSGS_COMMIT="$(ReadSingleValue messages_commit)"
readonly XRCE_AGENT_REPOSITORY="$(ReadSingleValue xrce_agent_repository)"
readonly XRCE_AGENT_COMMIT="$(ReadSingleValue xrce_agent_commit)"
[[ "${LOCK_FORMAT}" == "mentor-pi-micro-ros-agent-source-lock-v2" && \
   "${LOCK_ROS_DISTRO}" == "humble" ]] || \
  Fail "Agent source lock is not the supported Humble schema"
[[ "${AGENT_REPOSITORY}" == \
   "https://github.com/micro-ROS/micro-ROS-Agent.git" && \
   "${MSGS_REPOSITORY}" == \
   "https://github.com/micro-ROS/micro_ros_msgs.git" && \
   "${XRCE_AGENT_REPOSITORY}" == \
   "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" ]] || \
  Fail "Agent source lock contains an unexpected repository"
[[ "${AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ && \
   "${MSGS_COMMIT}" =~ ^[0-9a-f]{40}$ && \
   "${XRCE_AGENT_COMMIT}" =~ ^[0-9a-f]{40}$ ]] || \
  Fail "Agent source lock contains a malformed commit"

readonly SOURCE_ROOT="${work_root}/src"
readonly INSTALL_ROOT="${work_root}/install"
readonly HOST_ARCHITECTURE="$(dpkg --print-architecture)"

CloneAndVerify() {
  local repository="$1"
  local commit="$2"
  local destination="$3"
  if [[ ! -d "${destination}/.git" ]]; then
    [[ ! -e "${destination}" && ! -L "${destination}" ]] || \
      Fail "refusing to replace non-Git source path ${destination}"
    git init "${destination}"
    git -C "${destination}" remote add origin "${repository}"
    git -C "${destination}" fetch --depth 1 origin "${commit}"
    git -C "${destination}" checkout --detach FETCH_HEAD
  fi
}

ValidateSources() {
  "${STATE_VALIDATOR}" \
    --os-release /etc/os-release \
    --architecture "${HOST_ARCHITECTURE}" \
    --repository "${SOURCE_ROOT}/micro-ROS-Agent" \
    --origin "${AGENT_REPOSITORY}" \
    --commit "${AGENT_COMMIT}" \
    --repository "${SOURCE_ROOT}/micro_ros_msgs" \
    --origin "${MSGS_REPOSITORY}" \
    --commit "${MSGS_COMMIT}" \
    --repository "${SOURCE_ROOT}/Micro-XRCE-DDS-Agent" \
    --origin "${XRCE_AGENT_REPOSITORY}" \
    --commit "${XRCE_AGENT_COMMIT}"
}

if [[ "${mode}" == "fetch" ]]; then
  [[ ! -L "${work_root}" ]] || Fail "work root must not be symbolic"
  mkdir -p "${SOURCE_ROOT}"
  CloneAndVerify "${AGENT_REPOSITORY}" "${AGENT_COMMIT}" \
    "${SOURCE_ROOT}/micro-ROS-Agent"
  CloneAndVerify "${MSGS_REPOSITORY}" "${MSGS_COMMIT}" \
    "${SOURCE_ROOT}/micro_ros_msgs"
  CloneAndVerify "${XRCE_AGENT_REPOSITORY}" "${XRCE_AGENT_COMMIT}" \
    "${SOURCE_ROOT}/Micro-XRCE-DDS-Agent"
  ValidateSources
  echo "Fetched and verified pinned Humble micro-ROS Agent sources."
  exit 0
fi

[[ -d "${work_root}" && ! -L "${work_root}" ]] || \
  Fail "Agent build work root is missing or symbolic"
ValidateSources
case "${dependency_mode}" in
  preinstalled)
    echo "Using the immutable builder's preinstalled dependency set."
    ;;
  install)
    rosdep install --rosdistro humble --from-paths "${SOURCE_ROOT}" \
      --ignore-src --as-root pip:false -y
    ;;
esac

cmake -S "${SOURCE_ROOT}/Micro-XRCE-DDS-Agent" \
  -B "${work_root}/build-xrce-agent" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_ROOT}" \
  -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_P2P_PROFILE=OFF \
  -DUAGENT_BUILD_EXECUTABLE=OFF \
  -DUAGENT_ISOLATED_INSTALL=OFF
cmake --build "${work_root}/build-xrce-agent" --parallel \
  --target install

colcon --log-base "${work_root}/log" build \
  --merge-install \
  --base-paths "${SOURCE_ROOT}" \
  --build-base "${work_root}/build" \
  --install-base "${INSTALL_ROOT}" \
  --packages-up-to micro_ros_agent \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DMICROROSAGENT_SUPERBUILD=OFF \
    -DCMAKE_PREFIX_PATH="${INSTALL_ROOT}"

readonly AGENT_EXECUTABLE="${INSTALL_ROOT}/lib/micro_ros_agent/micro_ros_agent"
[[ -x "${AGENT_EXECUTABLE}" && ! -L "${AGENT_EXECUTABLE}" ]] || \
  Fail "native Agent executable was not produced"
echo "Built pinned Humble micro-ROS Agent: ${AGENT_EXECUTABLE}"
