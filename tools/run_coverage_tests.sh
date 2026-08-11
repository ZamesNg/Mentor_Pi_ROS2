#!/usr/bin/env bash
# Copyright 2026 Mentor Pi Maintainers
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

readonly REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly BUILD_ROOT="${COVERAGE_BUILD_ROOT:-${REPOSITORY_ROOT}/build/coverage}"
readonly PROFILE_DIRECTORY="${BUILD_ROOT}/profiles"
readonly REPORT_DIRECTORY="${BUILD_ROOT}/reports"

find_tool() {
  local configured="$1"
  local fallback="$2"
  if [[ -n "${configured}" ]]; then
    command -v "${configured}"
    return
  fi
  if command -v "${fallback}-18" >/dev/null 2>&1; then
    command -v "${fallback}-18"
    return
  fi
  if command -v "${fallback}" >/dev/null 2>&1; then
    command -v "${fallback}"
    return
  fi
  if command -v xcrun >/dev/null 2>&1; then
    xcrun --find "${fallback}"
    return
  fi
  echo "required tool not found: ${fallback}" >&2
  return 1
}

readonly CLANG_CXX_PATH="$(find_tool "${CLANG_CXX:-}" clang++)"
readonly LLVM_COV_PATH="$(find_tool "${LLVM_COV:-}" llvm-cov)"
readonly LLVM_PROFDATA_PATH="$(find_tool "${LLVM_PROFDATA:-}" llvm-profdata)"

collect_sources() {
  local destination_name="$1"
  shift
  local -a collected=()
  while IFS= read -r source; do
    collected+=("${source}")
  done < <(find "$@" -type f \( -name '*.cc' -o -name '*.h' \) | sort)
  eval "${destination_name}=(\"\${collected[@]}\")"
}

report_module() {
  local module="$1"
  local output="$2"
  shift 2
  "${LLVM_COV_PATH}" report "$@" >"${output}"
  awk -v module="${module}" '
    $1 == "TOTAL" {
      printf "%s,%s,%s,%s,%s,%s,%s\n", module, $8, $9, $10, $11, $12, $13
    }
  ' "${output}" >>"${REPORT_DIRECTORY}/summary.csv"
}

run_profiled_test() {
  local label="$1"
  local build_directory="$2"
  local test_name="$3"
  LLVM_PROFILE_FILE="${PROFILE_DIRECTORY}/${label}-%p-%m.profraw" \
    ctest --test-dir "${build_directory}" \
      --tests-regex "^${test_name}$" --output-on-failure
  "${LLVM_PROFDATA_PATH}" merge -sparse \
    "${PROFILE_DIRECTORY}/${label}-"*.profraw \
    -o "${PROFILE_DIRECTORY}/${label}.profdata"
}

cd "${REPOSITORY_ROOT}"
cmake -E remove_directory "${BUILD_ROOT}"
cmake -E make_directory "${PROFILE_DIRECTORY}" "${REPORT_DIRECTORY}"

readonly COVERAGE_COMPILE_FLAGS="-fprofile-instr-generate -fcoverage-mapping"
readonly COVERAGE_LINK_FLAGS="-fprofile-instr-generate"
readonly -a COMMON_CMAKE_ARGUMENTS=(
  -G Ninja
  -DBUILD_TESTING=ON
  -DCMAKE_BUILD_TYPE=Debug
  "-DCMAKE_CXX_COMPILER=${CLANG_CXX_PATH}"
  "-DCMAKE_CXX_FLAGS=${COVERAGE_COMPILE_FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS=${COVERAGE_LINK_FLAGS}"
)

cmake -S firmware/mentor_pi_mcu/app/controller \
  -B "${BUILD_ROOT}/controller" \
  "${COMMON_CMAKE_ARGUMENTS[@]}" \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_DRIVER_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_CONTROLLER_SANITIZERS=OFF
cmake --build "${BUILD_ROOT}/controller"

cmake -S firmware/mentor_pi_mcu/app/microros \
  -B "${BUILD_ROOT}/microros" \
  "${COMMON_CMAKE_ARGUMENTS[@]}"
cmake --build "${BUILD_ROOT}/microros"

cmake -S ros2_ws/src/mentor_pi_bringup \
  -B "${BUILD_ROOT}/bringup" \
  "${COMMON_CMAKE_ARGUMENTS[@]}" \
  -DMENTOR_PI_BUILD_ROS2=OFF
cmake --build "${BUILD_ROOT}/bringup"

run_profiled_test controller "${BUILD_ROOT}/controller" \
  mentor_pi_mcu_controller_tests
run_profiled_test domain "${BUILD_ROOT}/controller" \
  mentor_pi_mcu_domain_tests
# The concurrency test is part of the executed suite, but its counters are not
# credited to the coverage report. Keeping profiles executable-local avoids
# LLVM internal-linkage symbol collisions and makes the reported result
# conservative.
run_profiled_test concurrency "${BUILD_ROOT}/controller" \
  mentor_pi_mcu_concurrency_tests
run_profiled_test drivers "${BUILD_ROOT}/controller" \
  mentor_pi_mcu_driver_tests
run_profiled_test microros_core "${BUILD_ROOT}/microros" \
  mentor_pi_microros_core_tests
run_profiled_test configuration "${BUILD_ROOT}/bringup" configuration_test
run_profiled_test supervisor "${BUILD_ROOT}/bringup" supervisor_core_test
run_profiled_test qualification "${BUILD_ROOT}/bringup" \
  qualification_monitor_core_test
run_profiled_test commissioning "${BUILD_ROOT}/bringup" \
  motor_commissioning_core_test

declare -a domain_sources
declare -a driver_sources
declare -a controller_sources
collect_sources domain_sources \
  firmware/mentor_pi_mcu/src \
  firmware/mentor_pi_mcu/include
collect_sources driver_sources \
  firmware/mentor_pi_mcu/drivers/src \
  firmware/mentor_pi_mcu/drivers/include
collect_sources controller_sources \
  firmware/mentor_pi_mcu/app/controller/src \
  firmware/mentor_pi_mcu/app/controller/include
readonly -a microros_sources=(
  firmware/mentor_pi_mcu/app/microros/src/runtime_core.cc
  firmware/mentor_pi_mcu/app/microros/include/mentor_pi_mcu/app/microros/runtime_core.h
)
readonly -a configuration_sources=(
  ros2_ws/src/mentor_pi_bringup/src/configuration.cc
  ros2_ws/src/mentor_pi_bringup/include/mentor_pi_bringup/configuration.h
)
readonly -a supervisor_sources=(
  ros2_ws/src/mentor_pi_bringup/src/supervisor_core.cc
  ros2_ws/src/mentor_pi_bringup/include/mentor_pi_bringup/supervisor_core.h
)
readonly -a qualification_sources=(
  ros2_ws/src/mentor_pi_bringup/src/qualification_monitor_core.cc
  ros2_ws/src/mentor_pi_bringup/include/mentor_pi_bringup/qualification_monitor_core.h
)
readonly -a commissioning_sources=(
  ros2_ws/src/mentor_pi_bringup/src/motor_commissioning_core.cc
  ros2_ws/src/mentor_pi_bringup/include/mentor_pi_bringup/motor_commissioning_core.h
)
{
  echo "DOMAIN"
  printf '%s\n' "${domain_sources[@]}"
  echo "DRIVERS"
  printf '%s\n' "${driver_sources[@]}"
  echo "CONTROLLER"
  printf '%s\n' "${controller_sources[@]}"
  echo "MICROROS_CORE"
  printf '%s\n' "${microros_sources[@]}"
  echo "BRINGUP_CORE"
  printf '%s\n' "${configuration_sources[@]}" "${supervisor_sources[@]}" \
    "${qualification_sources[@]}" "${commissioning_sources[@]}"
} >"${REPORT_DIRECTORY}/source-manifest.txt"
{
  "${CLANG_CXX_PATH}" --version
  "${LLVM_COV_PATH}" --version
  "${LLVM_PROFDATA_PATH}" --version
} >"${REPORT_DIRECTORY}/tool-versions.txt"

printf '%s\n' "module,lines,missed_lines,line_coverage,branches,missed_branches,branch_coverage" \
  >"${REPORT_DIRECTORY}/summary.csv"

readonly controller_test="${BUILD_ROOT}/controller/mentor_pi_mcu_controller_tests"
readonly domain_test="${BUILD_ROOT}/controller/domain_build/mentor_pi_mcu_domain_tests"
readonly driver_test="${BUILD_ROOT}/controller/drivers_build/mentor_pi_mcu_driver_tests"
readonly microros_test="${BUILD_ROOT}/microros/mentor_pi_microros_core_tests"
readonly configuration_test="${BUILD_ROOT}/bringup/configuration_test"
readonly supervisor_test="${BUILD_ROOT}/bringup/supervisor_core_test"
readonly qualification_test="${BUILD_ROOT}/bringup/qualification_monitor_core_test"
readonly commissioning_test="${BUILD_ROOT}/bringup/motor_commissioning_core_test"

report_module domain "${REPORT_DIRECTORY}/domain.txt" \
  "${domain_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/domain.profdata" \
  "${domain_sources[@]}"
report_module drivers "${REPORT_DIRECTORY}/drivers.txt" \
  "${driver_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/drivers.profdata" \
  "${driver_sources[@]}"
report_module controller "${REPORT_DIRECTORY}/controller.txt" \
  "${controller_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/controller.profdata" \
  "${controller_sources[@]}"
report_module microros_core "${REPORT_DIRECTORY}/microros_core.txt" \
  "${microros_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/microros_core.profdata" \
  "${microros_sources[@]}"

"${LLVM_COV_PATH}" report "${configuration_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/configuration.profdata" \
  "${configuration_sources[@]}" >"${REPORT_DIRECTORY}/configuration.txt"
"${LLVM_COV_PATH}" report "${supervisor_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/supervisor.profdata" \
  "${supervisor_sources[@]}" >"${REPORT_DIRECTORY}/supervisor.txt"
"${LLVM_COV_PATH}" report "${qualification_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/qualification.profdata" \
  "${qualification_sources[@]}" >"${REPORT_DIRECTORY}/qualification.txt"
"${LLVM_COV_PATH}" report "${commissioning_test}" \
  -instr-profile="${PROFILE_DIRECTORY}/commissioning.profdata" \
  "${commissioning_sources[@]}" >"${REPORT_DIRECTORY}/commissioning.txt"
{
  echo "CONFIGURATION CORE"
  cat "${REPORT_DIRECTORY}/configuration.txt"
  echo
  echo "SUPERVISOR CORE"
  cat "${REPORT_DIRECTORY}/supervisor.txt"
  echo
  echo "QUALIFICATION MONITOR CORE"
  cat "${REPORT_DIRECTORY}/qualification.txt"
  echo
  echo "MOTOR COMMISSIONING CORE"
  cat "${REPORT_DIRECTORY}/commissioning.txt"
} >"${REPORT_DIRECTORY}/bringup_core.txt"
awk '
  $1 == "TOTAL" {
    lines += $8
    missed_lines += $9
    branches += $11
    missed_branches += $12
  }
  END {
    printf "bringup_core,%d,%d,%.2f%%,%d,%d,%.2f%%\n", lines,
           missed_lines, 100 * (lines - missed_lines) / lines, branches,
           missed_branches, 100 * (branches - missed_branches) / branches
  }
' "${REPORT_DIRECTORY}/configuration.txt" \
  "${REPORT_DIRECTORY}/supervisor.txt" \
  "${REPORT_DIRECTORY}/qualification.txt" \
  "${REPORT_DIRECTORY}/commissioning.txt" >>"${REPORT_DIRECTORY}/summary.csv"

awk -F, '
  NR > 1 {
    lines += $2
    missed_lines += $3
    branches += $5
    missed_branches += $6
  }
  END {
    printf "aggregate,%d,%d,%.2f%%,%d,%d,%.2f%%\n", lines, missed_lines,
           100 * (lines - missed_lines) / lines, branches, missed_branches,
           100 * (branches - missed_branches) / branches
  }
' "${REPORT_DIRECTORY}/summary.csv" >>"${REPORT_DIRECTORY}/summary.csv.tmp"
cat "${REPORT_DIRECTORY}/summary.csv.tmp" >>"${REPORT_DIRECTORY}/summary.csv"
{
  cat "${REPORT_DIRECTORY}/domain.txt"
  cat "${REPORT_DIRECTORY}/drivers.txt"
  cat "${REPORT_DIRECTORY}/controller.txt"
  cat "${REPORT_DIRECTORY}/microros_core.txt"
  cat "${REPORT_DIRECTORY}/bringup_core.txt"
  echo
  echo "AGGREGATE"
  tail -n 1 "${REPORT_DIRECTORY}/summary.csv"
} >"${REPORT_DIRECTORY}/aggregate.txt"

cat "${REPORT_DIRECTORY}/summary.csv"
echo
awk -F, '
  $1 == "aggregate" {
    found = 1
    line_coverage = 100 * ($2 - $3) / $2
    branch_coverage = 100 * ($5 - $6) / $5
  }
  END {
    if (!found) {
      print "coverage gate failed: aggregate row is missing" > "/dev/stderr"
      exit 1
    }
    if (line_coverage < 90.0 || branch_coverage < 80.0) {
      printf "coverage gate failed: line %.2f%% (minimum 90.00%%), " \
             "branch %.2f%% (minimum 80.00%%)\n", line_coverage, \
             branch_coverage > "/dev/stderr"
      exit 1
    }
  }
' "${REPORT_DIRECTORY}/summary.csv"
echo "Coverage gate passed: minimum 90% line and 80% branch."
echo "Scope is the explicit first-party production source manifest; test,"
echo "generated, third-party, and STM32-only sources are not in that manifest."
echo "Full per-file reports: ${REPORT_DIRECTORY}"
