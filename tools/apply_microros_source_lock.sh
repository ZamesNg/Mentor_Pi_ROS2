#!/usr/bin/env bash

set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo "Usage: $0 <micro-ROS workspace> <source lock>" >&2
  exit 2
fi

readonly WORKSPACE="$1"
readonly SOURCE_LOCK="$2"

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
trap 'rm -f "${NORMALIZED_LOCK}" "${CHECKED_REPOSITORIES}"' EXIT

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
  printf '%s\n' "${normalized_url}" >>"${CHECKED_REPOSITORIES}"
done < <(find "${WORKSPACE}" -type d -name .git -print | sort)

readonly EXPECTED_REPOSITORIES="$(mktemp)"
trap 'rm -f "${NORMALIZED_LOCK}" "${CHECKED_REPOSITORIES}" "${EXPECTED_REPOSITORIES}"' EXIT
awk '{print $1}' "${NORMALIZED_LOCK}" >"${EXPECTED_REPOSITORIES}"
sort -o "${CHECKED_REPOSITORIES}" "${CHECKED_REPOSITORIES}"
if ! diff -u "${EXPECTED_REPOSITORIES}" "${CHECKED_REPOSITORIES}"; then
  echo "The generated micro-ROS workspace is incomplete." >&2
  exit 1
fi

repository_count="$(wc -l <"${CHECKED_REPOSITORIES}" | tr -d '[:space:]')"
echo "Locked ${repository_count} micro-ROS source repositories"
