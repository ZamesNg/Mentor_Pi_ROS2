#!/usr/bin/env bash

set -euo pipefail

meminfo="${RRCLITE_MEMINFO_PATH:-/proc/meminfo}"
cpu_count="${RRCLITE_CPU_COUNT:-}"

Fail() {
  echo "Build-job selection error: $*" >&2
  exit 1
}

[[ "$#" -eq 0 ]] || Fail "usage: ./tools/select_build_jobs.sh"
if [[ -z "${cpu_count}" ]]; then
  cpu_count="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
fi
[[ "${cpu_count}" =~ ^[1-9][0-9]*$ ]] || \
  Fail "online CPU count is unavailable"

if [[ -n "${RRCLITE_BUILD_JOBS:-}" ]]; then
  [[ "${RRCLITE_BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || \
    Fail "RRCLITE_BUILD_JOBS must be a positive integer"
  ((RRCLITE_BUILD_JOBS <= cpu_count)) || \
    Fail "RRCLITE_BUILD_JOBS must not exceed the ${cpu_count} online CPUs"
  printf '%s\n' "${RRCLITE_BUILD_JOBS}"
  exit 0
fi

[[ -f "${meminfo}" && -r "${meminfo}" ]] || \
  Fail "memory information is unavailable: ${meminfo}"
available_kib="$(awk '$1 == "MemAvailable:" {print $2; exit}' "${meminfo}")"
[[ "${available_kib}" =~ ^[0-9]+$ ]] || \
  Fail "MemAvailable is missing from ${meminfo}"

# One job per 2 GiB of currently available memory, capped at four jobs.  This
# keeps the RDK X5 responsive and prevents colcon package parallelism from
# multiplying an inner Ninja/CMake job count.
memory_jobs=$((available_kib / 2097152))
((memory_jobs >= 1)) || memory_jobs=1
jobs="${cpu_count}"
((jobs <= 4)) || jobs=4
((jobs <= memory_jobs)) || jobs="${memory_jobs}"
printf '%s\n' "${jobs}"
