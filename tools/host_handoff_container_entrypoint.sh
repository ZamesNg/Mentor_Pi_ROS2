#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Host container entrypoint error: $*" >&2
  exit 1
}

[[ "$(id -u)" == 0 ]] || Fail "entrypoint must start as container root"
for variable in MENTOR_PI_CALLER_UID MENTOR_PI_CALLER_GID \
    MENTOR_PI_OUTPUT_RELATIVE MENTOR_PI_PREFIX_RELATIVE \
    MENTOR_PI_BUILD_RELATIVE MENTOR_PI_RELEASE_ID \
    MENTOR_PI_HOST_BUILDER_IMAGE MENTOR_PI_AGENT_RELATIVE \
    MENTOR_PI_RUNTIME_IMAGE_ARCHIVE_RELATIVE \
    MENTOR_PI_RUNTIME_IMAGE_ID MENTOR_PI_RUNTIME_IMAGE_SOURCE_ID \
    RRCLITE_BUILD_JOBS; do
  [[ -n "${!variable:-}" ]] || Fail "missing ${variable}"
done
[[ "${MENTOR_PI_CALLER_UID}" =~ ^[0-9]+$ ]] || Fail "invalid caller UID"
[[ "${MENTOR_PI_CALLER_GID}" =~ ^[0-9]+$ ]] || Fail "invalid caller GID"
[[ "${RRCLITE_BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "invalid RRCLITE_BUILD_JOBS"
[[ -f /root/.ros/rosdep/sources.cache/index ]] ||
  Fail "pinned builder image does not contain an offline rosdep cache"
command -v setpriv >/dev/null 2>&1 || Fail "setpriv is missing"

readonly BUILD_HOME=/tmp/mentor-pi-host-home
install -d -o "${MENTOR_PI_CALLER_UID}" -g "${MENTOR_PI_CALLER_GID}" \
  "${BUILD_HOME}" "${BUILD_HOME}/.ros" "${BUILD_HOME}/ros-log"
cp -a /root/.ros/rosdep "${BUILD_HOME}/.ros/rosdep"
chown -R "${MENTOR_PI_CALLER_UID}:${MENTOR_PI_CALLER_GID}" "${BUILD_HOME}"

exec setpriv \
  --reuid="${MENTOR_PI_CALLER_UID}" \
  --regid="${MENTOR_PI_CALLER_GID}" \
  --clear-groups \
  env \
    HOME="${BUILD_HOME}" \
    ROS_HOME="${BUILD_HOME}/.ros" \
    ROS_LOG_DIR="${BUILD_HOME}/ros-log" \
    SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-0}" \
    MENTOR_PI_HOST_BUILDER_IMAGE="${MENTOR_PI_HOST_BUILDER_IMAGE}" \
    MENTOR_PI_HOST_BUILDER_IMAGE_ID="${MENTOR_PI_HOST_BUILDER_IMAGE_ID}" \
    MENTOR_PI_OUTPUT_RELATIVE="${MENTOR_PI_OUTPUT_RELATIVE}" \
    MENTOR_PI_PREFIX_RELATIVE="${MENTOR_PI_PREFIX_RELATIVE}" \
    MENTOR_PI_BUILD_RELATIVE="${MENTOR_PI_BUILD_RELATIVE}" \
    MENTOR_PI_RELEASE_ID="${MENTOR_PI_RELEASE_ID}" \
    MENTOR_PI_AGENT_RELATIVE="${MENTOR_PI_AGENT_RELATIVE}" \
    MENTOR_PI_RUNTIME_IMAGE_ARCHIVE_RELATIVE="${MENTOR_PI_RUNTIME_IMAGE_ARCHIVE_RELATIVE}" \
    MENTOR_PI_RUNTIME_IMAGE_ID="${MENTOR_PI_RUNTIME_IMAGE_ID}" \
    MENTOR_PI_RUNTIME_IMAGE_SOURCE_ID="${MENTOR_PI_RUNTIME_IMAGE_SOURCE_ID}" \
    RRCLITE_BUILD_JOBS="${RRCLITE_BUILD_JOBS}" \
    CMAKE_BUILD_PARALLEL_LEVEL=1 \
    /bin/bash -lc '
      set -euo pipefail
      cd /workspace
      ./tools/build_host_release.sh \
        --project-root /workspace \
        --output-prefix "/workspace/${MENTOR_PI_PREFIX_RELATIVE}" \
        --work-directory "/workspace/${MENTOR_PI_BUILD_RELATIVE}"
      ./tools/package_host_handoff.sh \
        --host-prefix "/workspace/${MENTOR_PI_PREFIX_RELATIVE}" \
        --agent-prefix "/workspace/${MENTOR_PI_AGENT_RELATIVE}" \
        --runtime-image-archive "/workspace/${MENTOR_PI_RUNTIME_IMAGE_ARCHIVE_RELATIVE}" \
        --runtime-image-id "${MENTOR_PI_RUNTIME_IMAGE_ID}" \
        --runtime-image-source-id "${MENTOR_PI_RUNTIME_IMAGE_SOURCE_ID}" \
        --output-directory "/workspace/${MENTOR_PI_OUTPUT_RELATIVE}" \
        --release-id "${MENTOR_PI_RELEASE_ID}"
    '
