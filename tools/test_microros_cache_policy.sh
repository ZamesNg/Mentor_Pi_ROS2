#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly VALIDATOR="${SCRIPT_DIR}/verify_microros_cache.sh"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "micro-ROS cache policy test failed: $*" >&2
  exit 1
}

ExpectFailure() {
  if "$@" >/dev/null 2>&1; then
    Fail "command unexpectedly succeeded: $*"
  fi
}

printf '%s\n' archive >"${TEST_ROOT}/libmicroros.a"
readonly ARCHIVE_SHA="$(sha256sum "${TEST_ROOT}/libmicroros.a" | awk '{print $1}')"
readonly IMAGE_ID="sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
readonly GENERATION_SHA="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
readonly INTERFACES_SHA="cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
readonly TREE_SHA="dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
cat >"${TEST_ROOT}/cache.txt" <<EOF
format=mentor-pi-microros-cache-v1
image_id=${IMAGE_ID}
architecture=arm64
generation_input_sha256=${GENERATION_SHA}
interfaces_sha256=${INTERFACES_SHA}
archive_sha256=${ARCHIVE_SHA}
tree_sha256=${TREE_SHA}
EOF

args=(
  --metadata "${TEST_ROOT}/cache.txt"
  --image-id "${IMAGE_ID}"
  --architecture arm64
  --generation-input-sha256 "${GENERATION_SHA}"
  --interfaces-sha256 "${INTERFACES_SHA}"
  --archive "${TEST_ROOT}/libmicroros.a"
  --expected-archive-sha256 "${ARCHIVE_SHA}"
  --tree-sha256 "${TREE_SHA}"
  --expected-tree-sha256 "${TREE_SHA}"
)
"${VALIDATOR}" "${args[@]}" >/dev/null
ExpectFailure "${VALIDATOR}" "${args[@]/${IMAGE_ID}/sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee}"
ExpectFailure "${VALIDATOR}" "${args[@]/arm64/amd64}"
ExpectFailure "${VALIDATOR}" "${args[@]/${GENERATION_SHA}/ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff}"
ExpectFailure "${VALIDATOR}" "${args[@]/${INTERFACES_SHA}/1111111111111111111111111111111111111111111111111111111111111111}"
printf '%s\n' corrupt >>"${TEST_ROOT}/libmicroros.a"
ExpectFailure "${VALIDATOR}" "${args[@]}"

grep -Fq 'RemoveBuildRoot' "${SCRIPT_DIR}/build_microros_library.sh" || \
  Fail "invalid caches are not removed before regeneration"
echo "micro-ROS cache reuse and invalidation tests passed."
