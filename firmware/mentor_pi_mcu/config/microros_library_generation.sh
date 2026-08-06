#!/usr/bin/env bash

# Project-owned, deterministic replacement for the upstream static-library
# generator. The container and compiler are pinned separately; this script
# also detaches every fetched ROS repository at microros_sources.lock.
set -euo pipefail

: "${MICROROS_LIBRARY_FOLDER:?MICROROS_LIBRARY_FOLDER is required}"
: "${MICROROS_GEOMETRY2_COMMIT:?MICROROS_GEOMETRY2_COMMIT is required}"
: "${MICROROS_LIBYAML_REPOSITORY:?MICROROS_LIBYAML_REPOSITORY is required}"
: "${MICROROS_LIBYAML_COMMIT:?MICROROS_LIBYAML_COMMIT is required}"
: "${MICROROS_CAPTURE_SOURCE_LOCK:=0}"
: "${MICROROS_SOURCE_LOCK_CANDIDATE:=build/microros_sources.humble.candidate.lock}"
: "${MICROROS_CALLER_UID:?MICROROS_CALLER_UID is required}"
: "${MICROROS_CALLER_GID:?MICROROS_CALLER_GID is required}"
[[ "${MICROROS_CALLER_UID}" =~ ^[0-9]+$ ]]
[[ "${MICROROS_CALLER_GID}" =~ ^[0-9]+$ ]]

readonly BASE_PATH="/project/${MICROROS_LIBRARY_FOLDER}"
export BASE_PATH

# The official builder runs as root because it owns /uros_ws.  Its /project
# tree is a host bind mount, however, so leave every generated path removable
# by the invoking developer on native Linux as well as through Docker Desktop.
# Apply this on failure too: a partially generated tree must not require sudo
# before the next clean, deterministic regeneration attempt.
RestoreHostBuildTree() {
  local original_status=$?
  trap - EXIT
  local restoration_failed=0
  if [[ -d /project/build/microros ]]; then
    chown -R -- "${MICROROS_CALLER_UID}:${MICROROS_CALLER_GID}" \
      /project/build/microros || restoration_failed=1
    chmod -R u+rwX,go-w /project/build/microros || restoration_failed=1
  fi
  local source_lock_candidate="/project/${MICROROS_SOURCE_LOCK_CANDIDATE}"
  if [[ -e "${source_lock_candidate}" ]]; then
    chown -- "${MICROROS_CALLER_UID}:${MICROROS_CALLER_GID}" \
      "${source_lock_candidate}" || restoration_failed=1
    chmod u+rw,go-w "${source_lock_candidate}" || restoration_failed=1
  fi
  if [[ "${restoration_failed}" == "1" ]]; then
    echo "Failed to restore generated micro-ROS ownership to the caller." >&2
    exit 1
  fi
  exit "${original_status}"
}
trap RestoreHostBuildTree EXIT

if [[ -f "${BASE_PATH}/libmicroros/libmicroros.a" ]]; then
  echo "micro-ROS library found. Skipping generation."
  exit 0
fi

readonly REVIEWED_FLAGS_FILE="/project/config/firmware_flags.mk"
readonly PINNED_TOOLCHAIN_ROOT="/opt/arm-gnu-toolchain/bin"
[[ -f "${REVIEWED_FLAGS_FILE}" ]]
if [[ "${MICROROS_CAPTURE_SOURCE_LOCK}" == "0" ]]; then
  RET_CFLAGS="$(PYTHONHASHSEED=0 \
    python3 "${BASE_PATH}/library_generation/extract_flags.py" \
    <"${REVIEWED_FLAGS_FILE}")"
  export RET_CFLAGS
  echo "Cross-compiler flags: ${RET_CFLAGS}"
  [[ "$(command -v arm-none-eabi-gcc)" == \
      "${PINNED_TOOLCHAIN_ROOT}/arm-none-eabi-gcc" ]]
  [[ "$(command -v arm-none-eabi-g++)" == \
      "${PINNED_TOOLCHAIN_ROOT}/arm-none-eabi-g++" ]]
  [[ "$(arm-none-eabi-gcc -dumpfullversion)" == "13.2.1" ]]
  [[ "$(arm-none-eabi-g++ -dumpfullversion)" == "13.2.1" ]]
  arm-none-eabi-gcc --version
  arm-none-eabi-g++ --version
fi

cd /uros_ws
# ROS environment setup scripts are not nounset-safe: the generated
# setup.bash probes optional variables such as AMENT_TRACE_SETUP_FILES. Keep
# strict mode for this script, but suspend nounset only while sourcing the
# upstream environment.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/local_setup.bash
set -u
[[ "${ROS_DISTRO}" == "humble" ]] || {
  echo "The static library generator must run in ROS 2 Humble." >&2
  exit 1
}
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

pushd firmware/mcu_ws >/dev/null
git clone --branch humble --no-tags https://github.com/ros2/geometry2
git -C geometry2 checkout --detach "${MICROROS_GEOMETRY2_COMMIT}"
cp -R geometry2/tf2_msgs ros2/tf2_msgs
rm -rf geometry2

mkdir extra_packages
pushd extra_packages >/dev/null
readonly USER_CUSTOM_PACKAGES_DIR="${BASE_PATH}/../../microros_component/extra_packages"
if [[ -d "${USER_CUSTOM_PACKAGES_DIR}" ]]; then
  cp -R "${USER_CUSTOM_PACKAGES_DIR}/." .
fi
if [[ -f "${USER_CUSTOM_PACKAGES_DIR}/extra_packages.repos" ]]; then
  vcs import --input "${USER_CUSTOM_PACKAGES_DIR}/extra_packages.repos"
fi
cp -R "${BASE_PATH}/library_generation/extra_packages/." .
vcs import --input extra_packages.repos
popd >/dev/null
popd >/dev/null

CaptureSourceLock() {
  local output="/project/${MICROROS_SOURCE_LOCK_CANDIDATE}"
  local rows
  rows="$(mktemp)"
  while IFS= read -r git_directory; do
    local repository="${git_directory%/.git}"
    local repository_url
    repository_url="$(git -C "${repository}" config --get remote.origin.url)"
    repository_url="${repository_url%/}"
    repository_url="${repository_url%.git}"
    printf '%s %s\n' "${repository_url}" \
      "$(git -C "${repository}" rev-parse HEAD)" >>"${rows}"
  done < <(find /uros_ws/firmware -type d -name .git -print | sort)
  printf '%s %s\n' "${MICROROS_LIBYAML_REPOSITORY}" \
    "${MICROROS_LIBYAML_COMMIT}" >>"${rows}"
  if [[ -n "$(sort "${rows}" | awk 'previous == $1 {print $1; exit} {previous=$1}')" ]]; then
    rm -f "${rows}"
    echo "Generated workspace contains duplicate repository origins." >&2
    exit 1
  fi
  mkdir -p "$(dirname "${output}")"
  {
    printf '%s\n' \
      '# Canonical repository URL (without a trailing .git), then detached commit.' \
      '# Captured from the pinned Humble builder; keep this sorted.' \
      '# tools/apply_microros_source_lock.sh rejects missing and unexpected repositories.'
    LC_ALL=C sort "${rows}"
  } >"${output}"
  rm -f "${rows}"
  chmod a+rw "${output}"
}

if [[ "${MICROROS_CAPTURE_SOURCE_LOCK}" == "1" ]]; then
  CaptureSourceLock
  exit 0
fi
[[ "${MICROROS_CAPTURE_SOURCE_LOCK}" == "0" ]] || {
  echo "MICROROS_CAPTURE_SOURCE_LOCK must be 0 or 1." >&2
  exit 2
}

# micro_ros_setup deliberately places untracked, zero-byte COLCON_IGNORE
# markers in desktop repositories that are replaced by micro-ROS forks. They
# are workspace control state, not source. Temporarily suspend only that exact
# generated shape so the source-lock tool can still require every Git checkout
# to be completely clean, then restore the markers before colcon runs.
readonly GENERATED_COLCON_IGNORES="$(mktemp)"
while IFS= read -r -d '' colcon_ignore; do
  repository="$(git -C "$(dirname "${colcon_ignore}")" \
    rev-parse --show-toplevel 2>/dev/null || true)"
  [[ -n "${repository}" ]] || continue
  relative_ignore="${colcon_ignore#"${repository}/"}"
  [[ "${relative_ignore}" != "${colcon_ignore}" ]] || {
    echo "COLCON_IGNORE path is outside its Git repository." >&2
    exit 1
  }
  if git -C "${repository}" ls-files --error-unmatch \
      -- "${relative_ignore}" >/dev/null 2>&1; then
    continue
  fi
  [[ -f "${colcon_ignore}" && ! -L "${colcon_ignore}" && \
      ! -s "${colcon_ignore}" ]] || {
    echo "Unexpected generated COLCON_IGNORE shape: ${colcon_ignore}" >&2
    exit 1
  }
  case "${colcon_ignore}" in
    *$'\n'*)
      echo "Newlines in generated paths are unsupported." >&2
      exit 1
      ;;
  esac
  printf '%s\n' "${colcon_ignore}" >>"${GENERATED_COLCON_IGNORES}"
  rm -f -- "${colcon_ignore}"
done < <(find /uros_ws/firmware -type f -name COLCON_IGNORE -print0)

bash /rrclite_tools/apply_microros_source_lock.sh \
  /uros_ws/firmware /project/config/microros_sources.lock \
  --deferred-repository "${MICROROS_LIBYAML_REPOSITORY}"

while IFS= read -r colcon_ignore; do
  [[ -n "${colcon_ignore}" ]] || continue
  [[ ! -e "${colcon_ignore}" && ! -L "${colcon_ignore}" ]] || {
    echo "Source locking unexpectedly created ${colcon_ignore}." >&2
    exit 1
  }
  : >"${colcon_ignore}"
done <"${GENERATED_COLCON_IGNORES}"
rm -f -- "${GENERATED_COLCON_IGNORES}"

export TOOLCHAIN_PREFIX="${PINNED_TOOLCHAIN_ROOT}/arm-none-eabi-"
ros2 run micro_ros_setup build_firmware.sh \
  "${BASE_PATH}/library_generation/toolchain.cmake" \
  "${BASE_PATH}/library_generation/colcon.meta"

find firmware/build/include -name '*.c' -delete
mkdir -p "${BASE_PATH}/libmicroros/include"
cp -R firmware/build/include/. "${BASE_PATH}/libmicroros/include/"
cp firmware/build/libmicroros.a "${BASE_PATH}/libmicroros/libmicroros.a"
printf '%s\n' "${ROS_DISTRO}" >"${BASE_PATH}/libmicroros/ros_distro"

pushd firmware/mcu_ws >/dev/null
INCLUDE_ROS2_PACKAGES="$(colcon list | awk '{print $1}' | \
  awk -v delimiter=' ' '{result=(NR==1?result:result delimiter)$0} END {print result}')"
popd >/dev/null
for package in ${INCLUDE_ROS2_PACKAGES}; do
  nested_include="${BASE_PATH}/libmicroros/include/${package}/${package}"
  if [[ -d "${nested_include}" ]]; then
    rsync -r "${nested_include}/" \
      "${BASE_PATH}/libmicroros/include/${package}/"
    rm -rf "${nested_include}"
  fi
done

find firmware/mcu_ws/ros2 \
  \( -name '*.srv' -o -name '*.msg' -o -name '*.action' \) | \
  awk -F/ '{print $(NF-2) "/" $NF}' | sort \
  >"${BASE_PATH}/libmicroros/available_ros2_types"
find firmware/mcu_ws/extra_packages \
  \( -name '*.srv' -o -name '*.msg' -o -name '*.action' \) | \
  awk -F/ '{print $(NF-2) "/" $NF}' | sort \
  >>"${BASE_PATH}/libmicroros/available_ros2_types"

: >"${BASE_PATH}/libmicroros/built_packages"
while IFS= read -r git_directory; do
  repository="${git_directory%/.git}"
  printf '%s %s\n' \
    "$(git -C "${repository}" config --get remote.origin.url)" \
    "$(git -C "${repository}" rev-parse HEAD)" \
    >>"${BASE_PATH}/libmicroros/built_packages"
done < <(find /uros_ws/firmware -type d -name .git -print | sort)

chmod -R a+rwX "${BASE_PATH}/libmicroros"
