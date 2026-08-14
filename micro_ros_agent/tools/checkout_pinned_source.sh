#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "micro-ROS Agent source checkout error: $*" >&2
  exit 1
}

[[ "$#" == 3 ]] || {
  echo "Usage: checkout_pinned_source.sh REPOSITORY COMMIT ABSOLUTE_DESTINATION" >&2
  exit 2
}

readonly REPOSITORY="$1"
readonly COMMIT="$2"
readonly DESTINATION="$3"

[[ -n "${REPOSITORY}" && "${REPOSITORY}" != *$'\n'* ]] || \
  Fail "repository must be non-empty and contain no newlines"
[[ "${COMMIT}" =~ ^[0-9a-f]{40}$ ]] || \
  Fail "commit must be a lowercase 40-hex SHA"
[[ "${DESTINATION}" == /* && "${DESTINATION}" != / && \
   "${DESTINATION}" != *$'\n'* ]] || \
  Fail "destination must be an absolute, non-root path without newlines"

if [[ -d "${DESTINATION}/.git" && ! -L "${DESTINATION}/.git" ]]; then
  actual_origin="$(git -C "${DESTINATION}" remote get-url origin 2>/dev/null)" || \
    Fail "existing checkout has no readable origin: ${DESTINATION}"
  [[ "${actual_origin}" == "${REPOSITORY}" ]] || \
    Fail "existing checkout origin mismatch at ${DESTINATION}: ${actual_origin}"

  if git -C "${DESTINATION}" rev-parse --verify HEAD >/dev/null 2>&1; then
    exit 0
  fi

  [[ -z "$(git -C "${DESTINATION}" status --porcelain=v1 \
      --untracked-files=all)" ]] || \
    Fail "incomplete checkout has working-tree state: ${DESTINATION}"
  echo "Recovering incomplete pinned checkout: ${DESTINATION}"
else
  [[ ! -e "${DESTINATION}" && ! -L "${DESTINATION}" ]] || \
    Fail "refusing to replace non-Git source path ${DESTINATION}"
  git init "${DESTINATION}"
  git -C "${DESTINATION}" remote add origin "${REPOSITORY}"
fi

git -C "${DESTINATION}" fetch --depth 1 origin "${COMMIT}"
git -C "${DESTINATION}" checkout --detach "${COMMIT}"
