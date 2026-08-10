#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly CHECKOUT="${PROJECT_ROOT}/mentor_pi_ros2/third_party/altro-cpp"
readonly SOURCE_LOCK="${SCRIPT_DIR}/altro_source.lock"
readonly FIRMWARE_BOOTSTRAP="${SCRIPT_DIR}/bootstrap_firmware_dependencies.sh"

[[ -f "${SOURCE_LOCK}" && ! -L "${SOURCE_LOCK}" ]] || {
  echo "ALTO source lock is missing or symbolic" >&2
  exit 1
}
if grep -Ev '^[A-Z0-9_]+=[A-Za-z0-9./:+_-]+$|^$' "${SOURCE_LOCK}" | grep -q .; then
  echo "ALTO source lock contains unsupported syntax" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${SOURCE_LOCK}"
readonly REPOSITORY="${ALTO_REPOSITORY}"
readonly COMMIT="${ALTO_COMMIT}"
[[ "${ALTO_LICENSE}" == GPL-2.0-or-later ]] || {
  echo "ALTO source lock has the wrong license identity" >&2
  exit 1
}

if [[ "$#" -gt 1 || ("$#" -eq 1 && "$1" != --verify-existing) ]]; then
  echo "Usage: bootstrap_host_dependencies.sh [--verify-existing]" >&2
  exit 2
fi

if [[ "$#" -eq 0 ]]; then
  mkdir -p "$(dirname "${CHECKOUT}")"
  if [[ ! -d "${CHECKOUT}/.git" ]]; then
    [[ ! -e "${CHECKOUT}" ]] || {
      echo "Refusing to replace non-Git path: ${CHECKOUT}" >&2
      exit 1
    }
    git init "${CHECKOUT}"
    git -C "${CHECKOUT}" remote add origin "${REPOSITORY}"
    git -C "${CHECKOUT}" fetch --depth 1 origin "${COMMIT}"
    git -C "${CHECKOUT}" checkout --detach FETCH_HEAD
  fi
fi

"${FIRMWARE_BOOTSTRAP}" --verify-existing \
  "${REPOSITORY}" "${COMMIT}" "${CHECKOUT}"
grep -Fq 'GNU GENERAL PUBLIC LICENSE' "${CHECKOUT}/LICENSE" || {
  echo "ALTO checkout does not contain the reviewed GPL license" >&2
  exit 1
}
echo "Host dependency ALTO is present at ${COMMIT}."
