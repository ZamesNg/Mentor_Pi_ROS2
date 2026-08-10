#!/usr/bin/env bash

set -euo pipefail

metadata=""
image_id=""
architecture=""
generation_sha=""
interfaces_sha=""
archive=""
expected_archive_sha=""
tree_sha=""
expected_tree_sha=""

Fail() {
  echo "micro-ROS cache validation failed: $*" >&2
  exit 1
}

ReadValue() {
  local key="$1"
  local count=""
  count="$(grep -Ec "^${key}=" "${metadata}" || true)"
  [[ "${count}" == 1 ]] || Fail "metadata must contain exactly one ${key} entry"
  sed -n "s/^${key}=//p" "${metadata}"
}

while (($# > 0)); do
  case "$1" in
    --metadata) metadata="${2:-}"; shift 2 ;;
    --image-id) image_id="${2:-}"; shift 2 ;;
    --architecture) architecture="${2:-}"; shift 2 ;;
    --generation-input-sha256) generation_sha="${2:-}"; shift 2 ;;
    --interfaces-sha256) interfaces_sha="${2:-}"; shift 2 ;;
    --archive) archive="${2:-}"; shift 2 ;;
    --expected-archive-sha256) expected_archive_sha="${2:-}"; shift 2 ;;
    --tree-sha256) tree_sha="${2:-}"; shift 2 ;;
    --expected-tree-sha256) expected_tree_sha="${2:-}"; shift 2 ;;
    *) Fail "unsupported argument: $1" ;;
  esac
done

[[ -f "${metadata}" && ! -L "${metadata}" ]] || Fail "metadata is missing"
[[ -s "${archive}" && ! -L "${archive}" ]] || Fail "archive is missing"
[[ "${image_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || Fail "image identity is invalid"
[[ "${architecture}" == amd64 || "${architecture}" == arm64 ]] || \
  Fail "architecture is invalid"
for digest in "${generation_sha}" "${interfaces_sha}" \
    "${expected_archive_sha}" "${tree_sha}" "${expected_tree_sha}"; do
  [[ "${digest}" =~ ^[0-9a-f]{64}$ ]] || Fail "a required digest is invalid"
done

[[ "$(ReadValue format)" == mentor-pi-microros-cache-v1 ]] || \
  Fail "metadata format differs"
[[ "$(ReadValue image_id)" == "${image_id}" ]] || Fail "image identity differs"
[[ "$(ReadValue architecture)" == "${architecture}" ]] || \
  Fail "image architecture differs"
[[ "$(ReadValue generation_input_sha256)" == "${generation_sha}" ]] || \
  Fail "generator inputs differ"
[[ "$(ReadValue interfaces_sha256)" == "${interfaces_sha}" ]] || \
  Fail "interface inputs differ"

actual_archive_sha="$(sha256sum "${archive}" | awk '{print $1}')"
[[ "${actual_archive_sha}" == "${expected_archive_sha}" ]] || \
  Fail "archive differs from its reviewed digest"
[[ "$(ReadValue archive_sha256)" == "${actual_archive_sha}" ]] || \
  Fail "archive differs from cache metadata"
[[ "${tree_sha}" == "${expected_tree_sha}" ]] || \
  Fail "header tree differs from its reviewed digest"
[[ "$(ReadValue tree_sha256)" == "${tree_sha}" ]] || \
  Fail "header tree differs from cache metadata"

echo "Verified reusable micro-ROS cache."
