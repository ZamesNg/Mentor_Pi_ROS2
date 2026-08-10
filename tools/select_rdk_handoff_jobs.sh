#!/usr/bin/env bash

set -euo pipefail

Fail() {
  echo "RDK handoff job selection error: $*" >&2
  exit 1
}

cpu_count="${RRCLITE_CPU_COUNT:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)}"
[[ "${cpu_count}" =~ ^[1-9][0-9]*$ ]] || Fail "online CPU count is unavailable"

if [[ -n "${RRCLITE_BUILD_JOBS:-}" ]]; then
  [[ "${RRCLITE_BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || \
    Fail "RRCLITE_BUILD_JOBS must be a positive integer"
  ((RRCLITE_BUILD_JOBS <= cpu_count)) || \
    Fail "RRCLITE_BUILD_JOBS=${RRCLITE_BUILD_JOBS} exceeds ${cpu_count} available CPUs"
  printf '%s\n' "${RRCLITE_BUILD_JOBS}"
  exit 0
fi

((cpu_count <= 8)) && printf '%s\n' "${cpu_count}" || printf '8\n'
