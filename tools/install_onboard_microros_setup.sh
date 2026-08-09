#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCE_LOCK="${SCRIPT_DIR}/microros_setup_source.lock"
readonly WORKSPACE_ROOT="/opt/mentor_pi/micro_ros_setup-3.1.3"
readonly SOURCE_ROOT="${WORKSPACE_ROOT}/src/micro_ros_setup"
readonly INSTALL_ROOT="${WORKSPACE_ROOT}/install"
readonly OVERLAY="${INSTALL_ROOT}/local_setup.bash"
readonly STATE_FILE="${INSTALL_ROOT}/MENTOR-PI-MICRO-ROS-SETUP.txt"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"

Fail() {
  echo "Onboard micro_ros_setup installation error: $*" >&2
  exit 1
}

ReadLockValue() {
  local key="$1"
  local count
  local line
  count="$(grep -Ec "^${key}=" "${SOURCE_LOCK}" || true)"
  [[ "${count}" == "1" ]] || \
    Fail "source lock must contain exactly one ${key}= entry"
  line="$(grep -E "^${key}=" "${SOURCE_LOCK}")"
  printf '%s' "${line#*=}"
}

# The managed checkout is intentionally installed by root under /opt, while
# native firmware builds verify it as the unprivileged developer. Trust only
# this fixed checkout for each Git invocation instead of modifying the user's
# or system's safe.directory configuration.
SourceGit() {
  git -c "safe.directory=${SOURCE_ROOT}" -C "${SOURCE_ROOT}" "$@"
}

[[ "$#" == 1 ]] || \
  Fail "usage: ./tools/install_onboard_microros_setup.sh --install|--verify|--print-overlay"
case "$1" in
  --install | --verify | --print-overlay) ;;
  *) Fail "usage: ./tools/install_onboard_microros_setup.sh --install|--verify|--print-overlay" ;;
esac

[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] || \
  Fail "source lock is missing or symbolic"
readonly LOCK_FORMAT="$(ReadLockValue format)"
readonly REPOSITORY="$(ReadLockValue repository)"
readonly COMMIT="$(ReadLockValue commit)"
readonly VERSION="$(ReadLockValue version)"
[[ "${LOCK_FORMAT}" == "mentor-pi-micro-ros-setup-source-lock-v1" ]] || \
  Fail "source lock format is unsupported"
[[ "${REPOSITORY}" == "https://github.com/micro-ROS/micro_ros_setup.git" ]] || \
  Fail "source lock repository is unexpected"
[[ "${COMMIT}" =~ ^[0-9a-f]{40}$ && "${VERSION}" == "3.1.3" ]] || \
  Fail "source lock identity is malformed"

if [[ "$1" == "--print-overlay" ]]; then
  printf '%s\n' "${OVERLAY}"
  exit 0
fi

VerifyInstall() {
  [[ -d "${SOURCE_ROOT}/.git" && ! -L "${SOURCE_ROOT}" ]] || return 1
  [[ "$(SourceGit remote get-url origin 2>/dev/null)" == \
      "${REPOSITORY}" ]] || return 1
  [[ "$(SourceGit rev-parse HEAD 2>/dev/null)" == \
      "${COMMIT}" ]] || return 1
  [[ -z "$(SourceGit status --porcelain --untracked-files=all)" ]] || \
    return 1
  grep -Fq "<version>${VERSION}</version>" \
    "${SOURCE_ROOT}/package.xml" || return 1
  [[ -r "${OVERLAY}" && -f "${STATE_FILE}" && ! -L "${STATE_FILE}" ]] || \
    return 1
  grep -Fqx 'format=mentor-pi-micro-ros-setup-install-v1' \
    "${STATE_FILE}" || return 1
  grep -Fqx "repository=${REPOSITORY}" "${STATE_FILE}" || return 1
  grep -Fqx "commit=${COMMIT}" "${STATE_FILE}" || return 1
  grep -Fqx "version=${VERSION}" "${STATE_FILE}" || return 1

  set +u
  source "${ROS_SETUP}"
  source "${OVERLAY}"
  set -u
  [[ "$(ros2 pkg prefix micro_ros_setup 2>/dev/null)" == \
      "${INSTALL_ROOT}/micro_ros_setup" ]]
}

[[ -r "${ROS_SETUP}" ]] || Fail "ROS 2 Humble Bash setup is missing"
if [[ "$1" == "--verify" ]]; then
  VerifyInstall || Fail "the pinned micro_ros_setup source installation is invalid"
  echo "Verified source-built micro_ros_setup ${VERSION}: ${INSTALL_ROOT}"
  exit 0
fi

[[ "$(id -u)" == "0" ]] || Fail "run --install as root"
grep -Eq '^ID=ubuntu$' /etc/os-release || Fail "the onboard host must be Ubuntu"
grep -Eq '^VERSION_ID="?22[.]04"?$' /etc/os-release || \
  Fail "the onboard host must run Ubuntu 22.04"
[[ "$(dpkg --print-architecture)" == "arm64" ]] || \
  Fail "the onboard source installation requires arm64"
set +u
source "${ROS_SETUP}"
set -u
for tool in colcon git rosdep ros2; do
  command -v "${tool}" >/dev/null 2>&1 || Fail "required tool is missing: ${tool}"
done

if VerifyInstall; then
  echo "Reusing source-built micro_ros_setup ${VERSION}: ${INSTALL_ROOT}"
  exit 0
fi

install -d -m 0755 "${WORKSPACE_ROOT}/src"
[[ ! -L "${SOURCE_ROOT}" ]] || \
  Fail "refusing symbolic source path ${SOURCE_ROOT}"
if [[ ! -d "${SOURCE_ROOT}/.git" ]]; then
  [[ ! -e "${SOURCE_ROOT}" && ! -L "${SOURCE_ROOT}" ]] || \
    Fail "refusing to replace non-Git source path ${SOURCE_ROOT}"
  git init "${SOURCE_ROOT}"
  SourceGit remote add origin "${REPOSITORY}"
fi
[[ "$(SourceGit remote get-url origin)" == "${REPOSITORY}" ]] || \
  Fail "existing source checkout has the wrong origin"
SourceGit fetch --depth 1 origin "${COMMIT}"
SourceGit checkout --detach FETCH_HEAD
[[ "$(SourceGit rev-parse HEAD)" == "${COMMIT}" ]] || \
  Fail "source checkout did not resolve to the locked commit"
[[ -z "$(SourceGit status --porcelain --untracked-files=all)" ]] || \
  Fail "source checkout is modified"
grep -Fq "<version>${VERSION}</version>" "${SOURCE_ROOT}/package.xml" || \
  Fail "source checkout has the wrong package version"

# micro_ros_setup 3.1.3 declares clang-tidy as a general dependency even
# though neither its install rules nor this repository's generate_lib workflow
# invokes it.  The RDK X5 image cannot resolve Jammy's clang-tidy dependency
# chain, and onboard analysis intentionally stays outside this production
# package build.  Skip only that rosdep key; resolve every other dependency.
rosdep install --from-paths "${WORKSPACE_ROOT}/src" --ignore-src \
  --rosdistro humble --as-root pip:false --skip-keys=clang-tidy -y
set +u
source "${ROS_SETUP}"
set -u
colcon --log-base "${WORKSPACE_ROOT}/log" build \
  --base-paths "${WORKSPACE_ROOT}/src" \
  --build-base "${WORKSPACE_ROOT}/build" \
  --install-base "${INSTALL_ROOT}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

readonly TEMPORARY_STATE="${STATE_FILE}.tmp.$$"
[[ ! -e "${TEMPORARY_STATE}" && ! -L "${TEMPORARY_STATE}" ]] || \
  Fail "unexpected temporary state exists: ${TEMPORARY_STATE}"
printf '%s\n' \
  'format=mentor-pi-micro-ros-setup-install-v1' \
  "repository=${REPOSITORY}" \
  "commit=${COMMIT}" \
  "version=${VERSION}" \
  >"${TEMPORARY_STATE}"
mv -- "${TEMPORARY_STATE}" "${STATE_FILE}"
VerifyInstall || Fail "built micro_ros_setup installation failed verification"
echo "Installed source-built micro_ros_setup ${VERSION}: ${INSTALL_ROOT}"
