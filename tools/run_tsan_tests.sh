#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/ci-tsan"
readonly BUILD_LOCK="${SCRIPT_DIR}/run_with_build_lock.sh"
if [[ "${RRCLITE_BUILD_LOCK_HELD:-0}" != 1 ]]; then
  exec "${BUILD_LOCK}" "${BASH_SOURCE[0]}" "$@"
fi

declare -a generator_args=()
if command -v ninja >/dev/null 2>&1; then
  generator_args=(-G Ninja)
fi

cmake -E remove_directory "${BUILD_ROOT}"
cmake -S "${PROJECT_ROOT}/firmware/mentor_pi_mcu" -B "${BUILD_ROOT}" \
  "${generator_args[@]}" \
  -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Debug \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_ENABLE_TSAN=ON
cmake --build "${BUILD_ROOT}" --parallel "${RRCLITE_BUILD_JOBS:-1}" \
  --target mentor_pi_mcu_concurrency_tests
ctest --test-dir "${BUILD_ROOT}" --output-on-failure \
  -R '^mentor_pi_mcu_concurrency_tests$'

echo "ThreadSanitizer concurrency checks passed"
