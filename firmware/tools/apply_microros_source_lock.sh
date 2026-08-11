#!/usr/bin/env bash

set -euo pipefail

# Repository completeness is a bytewise set comparison. Do not let the
# caller's locale order prefix-related URLs such as ros2/rcl and ros2/rclc
# differently during normalization and final verification.
export LC_ALL=C

if [[ "$#" -lt 2 ]]; then
  echo "Usage: $0 <micro-ROS workspace> <source lock> [--deferred-repository URL ...]" >&2
  exit 2
fi

readonly WORKSPACE="$1"
readonly SOURCE_LOCK="$2"
shift 2

[[ -d "${WORKSPACE}" ]] || {
  echo "micro-ROS workspace does not exist: ${WORKSPACE}" >&2
  exit 1
}
[[ -f "${SOURCE_LOCK}" ]] || {
  echo "micro-ROS source lock does not exist: ${SOURCE_LOCK}" >&2
  exit 1
}

NormalizeUrl() {
  local url="$1"
  url="${url%/}"
  url="${url%.git}"
  printf '%s\n' "${url}"
}

readonly NORMALIZED_LOCK="$(mktemp)"
readonly CHECKED_REPOSITORIES="$(mktemp)"
readonly DEFERRED_REPOSITORIES="$(mktemp)"
trap 'rm -f "${NORMALIZED_LOCK}" "${CHECKED_REPOSITORIES}" "${DEFERRED_REPOSITORIES}"' EXIT

while [[ "$#" -gt 0 ]]; do
  [[ "$#" -ge 2 && "$1" == "--deferred-repository" ]] || {
    echo "Usage: $0 <micro-ROS workspace> <source lock> [--deferred-repository URL ...]" >&2
    exit 2
  }
  NormalizeUrl "$2" >>"${DEFERRED_REPOSITORIES}"
  shift 2
done
sort -o "${DEFERRED_REPOSITORIES}" "${DEFERRED_REPOSITORIES}"
duplicate_deferred="$(awk \
  'previous == $1 {print $1; exit} {previous=$1}' \
  "${DEFERRED_REPOSITORIES}")"
[[ -z "${duplicate_deferred}" ]] || {
  echo "Duplicate deferred source-lock repository: ${duplicate_deferred}" >&2
  exit 1
}

while read -r repository_url commit extra; do
  if [[ -z "${repository_url}" || "${repository_url:0:1}" == "#" ]]; then
    continue
  fi
  if [[ -z "${commit}" || -n "${extra:-}" ||
        ! "${commit}" =~ ^[0-9a-f]{40}$ ]]; then
    echo "Invalid source-lock row: ${repository_url} ${commit} ${extra:-}" >&2
    exit 1
  fi
  normalized_url="$(NormalizeUrl "${repository_url}")"
  printf '%s %s\n' "${normalized_url}" "${commit}" >>"${NORMALIZED_LOCK}"
done <"${SOURCE_LOCK}"
sort -o "${NORMALIZED_LOCK}" "${NORMALIZED_LOCK}"
duplicate_url="$(awk 'previous == $1 {print $1; exit} {previous=$1}' \
  "${NORMALIZED_LOCK}")"
if [[ -n "${duplicate_url}" ]]; then
  echo "Duplicate source-lock repository: ${duplicate_url}" >&2
  exit 1
fi

while IFS= read -r git_directory; do
  repository="${git_directory%/.git}"
  remote_url="$(git -C "${repository}" config --get remote.origin.url)"
  normalized_url="$(NormalizeUrl "${remote_url}")"
  expected_commit="$(awk -v url="${normalized_url}" \
    '$1 == url {print $2}' "${NORMALIZED_LOCK}")"
  if [[ -z "${expected_commit}" ]]; then
    echo "Unpinned micro-ROS repository: ${normalized_url}" >&2
    exit 1
  fi
  if grep -Fqx "${normalized_url}" "${CHECKED_REPOSITORIES}"; then
    echo "Duplicate micro-ROS repository checkout: ${normalized_url}" >&2
    exit 1
  fi
  worktree_status="$(git -C "${repository}" status --porcelain=v1 \
    --untracked-files=all)"
  if [[ -n "${worktree_status}" ]]; then
    echo "Dirty micro-ROS repository before locking: ${normalized_url}" >&2
    printf '%s\n' "${worktree_status}" >&2
    git -C "${repository}" diff --stat >&2 || true
    exit 1
  fi
  if ! git -C "${repository}" cat-file -e "${expected_commit}^{commit}";
  then
    git -C "${repository}" fetch --depth=1 origin "${expected_commit}"
  fi
  git -C "${repository}" checkout --detach "${expected_commit}"
  actual_commit="$(git -C "${repository}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "Failed to lock ${normalized_url} at ${expected_commit}" >&2
    exit 1
  fi
  if git -C "${repository}" symbolic-ref -q HEAD >/dev/null 2>&1; then
    echo "micro-ROS repository is not detached: ${normalized_url}" >&2
    exit 1
  fi
  if [[ -n "$(git -C "${repository}" status --porcelain=v1 \
      --untracked-files=all)" ]]; then
    echo "Dirty micro-ROS repository after locking: ${normalized_url}" >&2
    exit 1
  fi
  printf '%s\n' "${normalized_url}" >>"${CHECKED_REPOSITORIES}"
done < <(find "${WORKSPACE}" -type d -name .git -print | sort)

readonly EXPECTED_REPOSITORIES="$(mktemp)"
trap 'rm -f "${NORMALIZED_LOCK}" "${CHECKED_REPOSITORIES}" "${DEFERRED_REPOSITORIES}" "${EXPECTED_REPOSITORIES}"' EXIT
while IFS= read -r deferred_repository; do
  [[ -n "${deferred_repository}" ]] || continue
  deferred_count="$(awk -v url="${deferred_repository}" \
    '$1 == url {count++} END {print count + 0}' "${NORMALIZED_LOCK}")"
  [[ "${deferred_count}" == "1" ]] || {
    echo "Deferred repository is not present exactly once in the lock: ${deferred_repository}" >&2
    exit 1
  }
done <"${DEFERRED_REPOSITORIES}"
awk 'NR == FNR {deferred[$1] = 1; next} !($1 in deferred) {print $1}' \
  "${DEFERRED_REPOSITORIES}" "${NORMALIZED_LOCK}" \
  >"${EXPECTED_REPOSITORIES}"
sort -o "${CHECKED_REPOSITORIES}" "${CHECKED_REPOSITORIES}"
if ! diff -u "${EXPECTED_REPOSITORIES}" "${CHECKED_REPOSITORIES}"; then
  echo "The generated micro-ROS workspace is incomplete." >&2
  exit 1
fi

repository_count="$(wc -l <"${CHECKED_REPOSITORIES}" | tr -d '[:space:]')"
deferred_count="$(wc -l <"${DEFERRED_REPOSITORIES}" | tr -d '[:space:]')"
echo "Locked ${repository_count} micro-ROS source repositories; ${deferred_count} deferred repository/repositories remain mandatory after generation"
