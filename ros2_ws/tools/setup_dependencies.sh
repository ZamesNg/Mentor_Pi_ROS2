#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly ALTO_ROOT="${WORKSPACE_ROOT}/third_party/altro-cpp"
readonly ALTO_URL="https://github.com/ZamesNg/altro-cpp.git"
readonly ALTO_COMMIT="7336800baa3f2f6e0c8edfad472c1ea51c54321a"

Fail() {
  echo "ROS dependency setup error: $*" >&2
  exit 1
}

"${SCRIPT_DIR}/check_environment.sh" >/dev/null
mkdir -p "${WORKSPACE_ROOT}/third_party"
if [[ ! -d "${ALTO_ROOT}/.git" ]]; then
  [[ ! -e "${ALTO_ROOT}" ]] || Fail "refusing to replace ${ALTO_ROOT}"
  vcs import --input "${WORKSPACE_ROOT}/dependencies.repos" \
    "${WORKSPACE_ROOT}"
fi
[[ "$(git -C "${ALTO_ROOT}" config --get remote.origin.url)" == \
  "${ALTO_URL}" ]] || Fail "ALTO origin differs from dependencies.repos"
[[ "$(git -C "${ALTO_ROOT}" rev-parse HEAD)" == "${ALTO_COMMIT}" ]] || \
  Fail "ALTO is not at the pinned commit"
[[ -z "$(git -C "${ALTO_ROOT}" status --porcelain=v1 \
  --untracked-files=all)" ]] || Fail "ALTO checkout is dirty"
grep -Fq 'GNU GENERAL PUBLIC LICENSE' "${ALTO_ROOT}/LICENSE" || \
  Fail "ALTO GPL license is missing"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
rosdep install --rosdistro humble --from-paths "${WORKSPACE_ROOT}/src" \
  --ignore-src --as-root pip:false -y
echo "Pinned ROS workspace dependencies are ready."
