#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "Host build container entrypoint error: $*" >&2
  exit 1
}

[[ "$(id -u)" == 0 ]] || Fail "entrypoint must start as container root"
for variable in MENTOR_PI_CALLER_UID MENTOR_PI_CALLER_GID \
    MENTOR_PI_OUTPUT_RELATIVE MENTOR_PI_WORK_RELATIVE \
    MENTOR_PI_HOST_BUILDER_IMAGE MENTOR_PI_SKIP_TESTS; do
  [[ -n "${!variable:-}" ]] || Fail "missing ${variable}"
done
[[ "${RRCLITE_BUILD_JOBS:-}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "missing or invalid RRCLITE_BUILD_JOBS"
[[ "${MENTOR_PI_CALLER_UID}" =~ ^[0-9]+$ && \
  "${MENTOR_PI_CALLER_GID}" =~ ^[0-9]+$ ]] || \
  Fail "caller UID/GID is invalid"
[[ "${MENTOR_PI_SKIP_TESTS}" == 0 || "${MENTOR_PI_SKIP_TESTS}" == 1 ]] || \
  Fail "MENTOR_PI_SKIP_TESTS must be 0 or 1"
[[ -f /root/.ros/rosdep/sources.cache/index ]] || \
  Fail "pinned builder image has no offline rosdep cache"
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
    MENTOR_PI_HOST_BUILDER_IMAGE="${MENTOR_PI_HOST_BUILDER_IMAGE}" \
    MENTOR_PI_SKIP_TESTS="${MENTOR_PI_SKIP_TESTS}" \
    RRCLITE_BUILD_JOBS="${RRCLITE_BUILD_JOBS}" \
    CMAKE_BUILD_PARALLEL_LEVEL=1 \
    /bin/bash -lc '
      set -euo pipefail
      cd /workspace
      args=(
        --project-root /workspace \
        --output-prefix "/workspace/${MENTOR_PI_OUTPUT_RELATIVE}" \
        --work-directory "/workspace/${MENTOR_PI_WORK_RELATIVE}"
      )
      [[ "${MENTOR_PI_SKIP_TESTS}" == 0 ]] || args+=(--skip-tests)
      exec ./tools/build_host_release.sh "${args[@]}"
    '
