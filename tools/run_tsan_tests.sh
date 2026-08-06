#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly BUILD_ROOT="${PROJECT_ROOT}/build/ci-tsan"

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
cmake --build "${BUILD_ROOT}" --parallel \
  --target mentor_pi_mcu_concurrency_tests
ctest --test-dir "${BUILD_ROOT}" --output-on-failure \
  -R '^mentor_pi_mcu_concurrency_tests$'

echo "ThreadSanitizer concurrency checks passed"
