#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

Fail() {
  echo "Firmware fingerprint test failed: $*" >&2
  exit 1
}

LegacyFingerprint() {
  local root="$1" manifest="$2"
  shift 2
  : >"${manifest}"
  local relative
  for relative in "$@"; do
    printf '%s  %s\n' \
      "$("${SCRIPT_DIR}/sha256.sh" "${root}/${relative}")" \
      "${relative}" >>"${manifest}"
  done
  "${SCRIPT_DIR}/sha256.sh" "${manifest}"
}

source_root="${TEST_ROOT}/firmware"
mkdir -p "${source_root}/nested" "${source_root}/sdk/humble" \
  "${source_root}/build/ignored" "${source_root}/third_party/ignored" \
  "${source_root}/.deps/ignored"
printf '%s\n' 'source cmake' >"${source_root}/CMakeLists.txt"
printf '%s\n' 'source with spaces' >"${source_root}/nested/file with space.txt"
printf '%s\n' 'sdk manifest' >"${source_root}/sdk/humble/manifest.txt"
printf '%s\n' 'excluded archive' \
  >"${source_root}/sdk/humble/libmicroros.tar.xz"
printf '%s\n' 'excluded build' >"${source_root}/build/ignored/file"
printf '%s\n' 'excluded dependency' >"${source_root}/third_party/ignored/file"
printf '%s\n' 'excluded local dependency' >"${source_root}/.deps/ignored/file"
printf '%s\n' 'excluded mac metadata' >"${source_root}/.DS_Store"
printf '%s\n' 'excluded bytecode' >"${source_root}/ignored.pyc"
expected_source="$(LegacyFingerprint "${source_root}" \
  "${TEST_ROOT}/source.manifest" \
  CMakeLists.txt 'nested/file with space.txt' sdk/humble/manifest.txt)"
actual_source="$("${SCRIPT_DIR}/source_fingerprint.sh" "${source_root}")"
[[ "${actual_source}" == "${expected_source}" ]] || \
  Fail "batched source digest differs from the legacy aggregate"

interface_root="${TEST_ROOT}/mentor_pi_interfaces"
mkdir -p "${interface_root}/include/example" "${interface_root}/msg" \
  "${interface_root}/srv"
printf '%s\n' 'interface cmake' >"${interface_root}/CMakeLists.txt"
printf '%s\n' 'interface package' >"${interface_root}/package.xml"
printf '%s\n' 'header' >"${interface_root}/include/example/contract.hpp"
printf '%s\n' 'message' >"${interface_root}/msg/Motor.msg"
printf '%s\n' 'service with spaces' \
  >"${interface_root}/srv/Set Motor.srv"
expected_interface="$(LegacyFingerprint "${interface_root}" \
  "${TEST_ROOT}/interface.manifest" \
  CMakeLists.txt include/example/contract.hpp msg/Motor.msg package.xml \
  'srv/Set Motor.srv')"
actual_interface="$("${SCRIPT_DIR}/interface_fingerprint.sh" \
  "${interface_root}")"
[[ "${actual_interface}" == "${expected_interface}" ]] || \
  Fail "batched interface digest differs from the legacy aggregate"

sdk_root="${TEST_ROOT}/sdk"
mkdir -p "${sdk_root}/include/example"
printf '%s\n' 'library' >"${sdk_root}/libmicroros.a"
printf '%s\n' 'generated header' >"${sdk_root}/include/example/generated.h"
printf '%s\n' 'generated header with spaces' \
  >"${sdk_root}/include/example/header with space.h"
expected_sdk="$(LegacyFingerprint "${sdk_root}" \
  "${TEST_ROOT}/sdk.manifest" \
  'include/example/generated.h' 'include/example/header with space.h' \
  libmicroros.a)"
actual_sdk="$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${sdk_root}")"
[[ "${actual_sdk}" == "${expected_sdk}" ]] || \
  Fail "batched SDK tree digest differs from the legacy aggregate"

expected_batch="$(printf '%s\n%s' \
  "$("${SCRIPT_DIR}/sha256.sh" "${source_root}/CMakeLists.txt")" \
  "$("${SCRIPT_DIR}/sha256.sh" "${source_root}/nested/file with space.txt")")"
actual_batch="$("${SCRIPT_DIR}/sha256.sh" \
  "${source_root}/CMakeLists.txt" \
  "${source_root}/nested/file with space.txt")"
[[ "${actual_batch}" == "${expected_batch}" ]] || \
  Fail "multi-file SHA-256 backend changed digest order"

printf '%s\n' 'tampered generated header' \
  >"${sdk_root}/include/example/generated.h"
tampered_sdk="$("${SCRIPT_DIR}/sdk_tree_fingerprint.sh" "${sdk_root}")"
[[ "${tampered_sdk}" != "${actual_sdk}" ]] || \
  Fail "SDK tree tamper did not change its aggregate digest"

echo "Firmware batched fingerprint equivalence and tamper tests passed."
