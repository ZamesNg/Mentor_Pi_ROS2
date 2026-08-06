#!/usr/bin/env bash

# Project-owned, deterministic replacement for the upstream static-library
# generator. The container and compiler are pinned separately; this script
# also detaches every fetched ROS repository at microros_sources.lock.
set -euo pipefail

: "${MICROROS_LIBRARY_FOLDER:?MICROROS_LIBRARY_FOLDER is required}"
: "${MICROROS_GEOMETRY2_COMMIT:?MICROROS_GEOMETRY2_COMMIT is required}"

readonly BASE_PATH="/project/${MICROROS_LIBRARY_FOLDER}"
export BASE_PATH

# The official builder runs as root because it owns /uros_ws.  Its /project
# tree is a host bind mount, however, so leave every generated path removable
# by the invoking developer on native Linux as well as through Docker Desktop.
# Apply this on failure too: a partially generated tree must not require sudo
# before the next clean, deterministic regeneration attempt.
MakeHostBuildTreeWritable() {
  if [[ -d /project/build/microros ]]; then
    # Copied Git object databases can contain intentionally read-only objects,
    # and Docker Desktop may reject chmod on those host-owned inodes even for
    # container root. They are input-only and already owned by the invoking
    # developer. Make generated paths writable while pruning every .git tree.
    find /project/build/microros -type d -name .git -prune -o \
      -exec chmod a+rwX {} +
  fi
}
trap MakeHostBuildTreeWritable EXIT

if [[ -f "${BASE_PATH}/libmicroros/libmicroros.a" ]]; then
  echo "micro-ROS library found. Skipping generation."
  exit 0
fi

readonly REVIEWED_FLAGS_FILE="/project/config/firmware_flags.mk"
[[ -f "${REVIEWED_FLAGS_FILE}" ]]
RET_CFLAGS="$(PYTHONHASHSEED=0 \
  python3 "${BASE_PATH}/library_generation/extract_flags.py" \
  <"${REVIEWED_FLAGS_FILE}")"
export RET_CFLAGS
echo "Cross-compiler flags: ${RET_CFLAGS}"
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version

cd /uros_ws
# ROS environment setup scripts are not nounset-safe: Jazzy's generated
# setup.bash probes optional variables such as AMENT_TRACE_SETUP_FILES. Keep
# strict mode for this script, but suspend nounset only while sourcing the
# upstream environment.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/local_setup.bash
set -u
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

pushd firmware/mcu_ws >/dev/null
git clone --branch jazzy --no-tags https://github.com/ros2/geometry2
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

bash /rrclite_tools/apply_microros_source_lock.sh \
  /uros_ws/firmware /project/config/microros_sources.lock

export TOOLCHAIN_PREFIX=/usr/bin/arm-none-eabi-
ros2 run micro_ros_setup build_firmware.sh \
  "${BASE_PATH}/library_generation/toolchain.cmake" \
  "${BASE_PATH}/library_generation/colcon.meta"

find firmware/build/include -name '*.c' -delete
mkdir -p "${BASE_PATH}/libmicroros/include"
cp -R firmware/build/include/. "${BASE_PATH}/libmicroros/include/"
cp firmware/build/libmicroros.a "${BASE_PATH}/libmicroros/libmicroros.a"

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
