#!/usr/bin/env bash

set -euo pipefail

readonly BUILD_ROOT="build/firmware-target-debug"
readonly LOCKED_ELF="firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf"
readonly TARGET_ROOT="firmware/mentor_pi_mcu/target/stm32"
readonly TOOLCHAIN_FILE="${TARGET_ROOT}/arm-none-eabi-toolchain.cmake"
readonly SOURCE_REGEX="^/workspace/firmware/mentor_pi_mcu/(src|drivers/src|platform/stm32/src|app/controller/src|app/microros/src|target/stm32)/[^/]+[.](c|cc)$"
readonly -a REQUIRED_STRONG_VECTOR_SYMBOLS=(
  ADC_IRQHandler
  DMA2_Stream0_IRQHandler
  DMA2_Stream2_IRQHandler
  DMA2_Stream3_IRQHandler
  DMA2_Stream7_IRQHandler
  EXTI15_10_IRQHandler
  SPI1_IRQHandler
  UART5_IRQHandler
  USART1_IRQHandler
  TIM7_IRQHandler
  TIM8_BRK_TIM12_IRQHandler
  TIM8_UP_TIM13_IRQHandler
  TIM8_TRG_COM_TIM14_IRQHandler
  NMI_Handler
  HardFault_Handler
  MemManage_Handler
  BusFault_Handler
  UsageFault_Handler
  SVC_Handler
  PendSV_Handler
  SysTick_Handler
)

Fail() {
  echo "Firmware target container error: $*" >&2
  exit 1
}

CheckVectorStrength() {
  local elf="$1"
  local manifest="$2"
  local symbol_table="${manifest%.txt}-defined-symbols.txt"
  [[ -s "${elf}" ]] || Fail "ELF for vector audit is missing: ${elf}"

  arm-none-eabi-nm --defined-only "${elf}" >"${symbol_table}"
  : >"${manifest}"
  local vector_symbol
  for vector_symbol in "${REQUIRED_STRONG_VECTOR_SYMBOLS[@]}"; do
    local strong_definition_count
    strong_definition_count="$(
      awk -v wanted="${vector_symbol}" '
        $2 == "T" && $3 == wanted { ++count }
        END { print count + 0 }
      ' "${symbol_table}"
    )"
    if [[ "${strong_definition_count}" -ne 1 ]]; then
      awk -v wanted="${vector_symbol}" '$3 == wanted { print }' \
        "${symbol_table}" >&2
      Fail "${vector_symbol} is not one strong global text definition"
    fi
    printf "T %s\n" "${vector_symbol}" >>"${manifest}"
  done
}

CheckWatchdogRetention() {
  local elf="$1"
  local manifest="$2"
  [[ -s "${elf}" ]] || Fail "ELF for watchdog audit is missing: ${elf}"

  local symbol_record
  symbol_record="$(
    arm-none-eabi-objdump -t "${elf}" \
      | awk '$NF == "g_watchdog_retention_record" {
          print $1, $2, $3, $4, $5
        }'
  )"
  local symbol_count
  symbol_count="$(wc -l <<<"${symbol_record}" | tr -d "[:space:]")"
  [[ -n "${symbol_record}" && "${symbol_count}" -eq 1 ]] || \
    Fail "g_watchdog_retention_record is not one exact ELF symbol"

  local symbol_address symbol_binding symbol_type symbol_section symbol_size
  read -r symbol_address symbol_binding symbol_type symbol_section symbol_size \
    <<<"${symbol_record}"
  [[ "${symbol_binding}" == "g" && "${symbol_type}" == "O" && \
     "${symbol_section}" == ".noinit" ]] || \
    Fail "watchdog record is not a global object in .noinit"
  (( 16#${symbol_size} == 12 )) || \
    Fail "watchdog record is not exactly 12 bytes"
  (( (16#${symbol_address} % 4) == 0 )) || \
    Fail "watchdog record address is not four-byte aligned"

  local noinit_nobits_count
  noinit_nobits_count="$(
    arm-none-eabi-readelf -SW "${elf}" \
      | awk '$0 ~ /\][[:space:]]+\.noinit[[:space:]]+NOBITS/ {
          ++count
        }
        END { print count + 0 }'
  )"
  [[ "${noinit_nobits_count}" -eq 1 ]] || \
    Fail ".noinit is not exactly one NOBITS section"

  local noinit_size noinit_address bss_size bss_address
  read -r noinit_size noinit_address <<<"$(
    arm-none-eabi-objdump -h "${elf}" \
      | awk '$2 == ".noinit" { print $3, $4 }'
  )"
  read -r bss_size bss_address <<<"$(
    arm-none-eabi-objdump -h "${elf}" \
      | awk '$2 == ".bss" { print $3, $4 }'
  )"
  [[ -n "${noinit_size}" && -n "${noinit_address}" && \
     -n "${bss_size}" && -n "${bss_address}" ]] || \
    Fail "cannot resolve .bss/.noinit ranges"

  local noinit_begin noinit_end bss_begin bss_end symbol_begin
  noinit_begin=$((16#${noinit_address}))
  noinit_end=$((noinit_begin + 16#${noinit_size}))
  bss_begin=$((16#${bss_address}))
  bss_end=$((bss_begin + 16#${bss_size}))
  symbol_begin=$((16#${symbol_address}))
  (( symbol_begin >= noinit_begin && symbol_begin + 12 <= noinit_end )) || \
    Fail "watchdog record lies outside .noinit"
  if (( noinit_begin < bss_end && bss_begin < noinit_end )); then
    Fail ".noinit overlaps .bss"
  fi

  printf "%s\n" \
    "symbol=g_watchdog_retention_record" \
    "symbol_size_bytes=12" \
    "symbol_alignment_bytes=4" \
    "section=.noinit" \
    "section_type=NOBITS" \
    "bss_disjoint=1" \
    "result=pass" \
    >"${manifest}"
}

[[ "$#" -eq 0 ]] || Fail "this helper does not accept arguments"
[[ "$(pwd -P)" == "/workspace" ]] || \
  Fail "this helper must run from the /workspace container mount"
[[ "$(arm-none-eabi-gcc -dumpfullversion)" == "13.2.1" ]] || \
  Fail "arm-none-eabi-gcc 13.2.1 is required"
clang-tidy-18 --version | grep -Fq "LLVM version 18." || \
  Fail "clang-tidy 18 is required"

cmake -E remove_directory "${BUILD_ROOT}"
cmake -S "${TARGET_ROOT}" -B "${BUILD_ROOT}" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="/workspace/${TOOLCHAIN_FILE}" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRRCLITE_MOTOR_COMMISSIONING=OFF \
  -DRRCLITE_MOTOR_COMMISSIONING_ACK=
cmake --build "${BUILD_ROOT}" --parallel

grep -Fqx "CMAKE_BUILD_TYPE:STRING=Debug" "${BUILD_ROOT}/CMakeCache.txt"
grep -Fqx "RRCLITE_MOTOR_COMMISSIONING:BOOL=OFF" \
  "${BUILD_ROOT}/CMakeCache.txt"
[[ -s "${BUILD_ROOT}/mentor_pi_mcu.elf" ]] || Fail "Debug ELF is missing"
[[ -s "${BUILD_ROOT}/compile_commands.json" ]] || \
  Fail "Debug compile database is missing"

CheckVectorStrength \
  "${LOCKED_ELF}" \
  "${BUILD_ROOT}/locked-required-strong-vector-symbols.txt"
CheckVectorStrength \
  "${BUILD_ROOT}/mentor_pi_mcu.elf" \
  "${BUILD_ROOT}/debug-required-strong-vector-symbols.txt"
CheckWatchdogRetention \
  "${LOCKED_ELF}" \
  "${BUILD_ROOT}/locked-watchdog-retention.txt"
CheckWatchdogRetention \
  "${BUILD_ROOT}/mentor_pi_mcu.elf" \
  "${BUILD_ROOT}/debug-watchdog-retention.txt"

{
  for source_root in \
      firmware/mentor_pi_mcu/src \
      firmware/mentor_pi_mcu/drivers/src \
      firmware/mentor_pi_mcu/platform/stm32/src \
      firmware/mentor_pi_mcu/app/controller/src \
      firmware/mentor_pi_mcu/app/microros/src \
      firmware/mentor_pi_mcu/target/stm32; do
    find "${source_root}" -maxdepth 1 -type f \
      \( -name "*.c" -o -name "*.cc" \) -print
  done
} | sed "s#^#/workspace/#" | LC_ALL=C sort -u \
  >"${BUILD_ROOT}/expected-first-party-sources.txt"

jq -r ".[].file" "${BUILD_ROOT}/compile_commands.json" \
  | awk -v pattern="${SOURCE_REGEX}" '$0 ~ pattern' \
  | LC_ALL=C sort -u \
  >"${BUILD_ROOT}/analyzed-first-party-sources.txt"

if ! cmp -s "${BUILD_ROOT}/expected-first-party-sources.txt" \
    "${BUILD_ROOT}/analyzed-first-party-sources.txt"; then
  diff -u "${BUILD_ROOT}/expected-first-party-sources.txt" \
    "${BUILD_ROOT}/analyzed-first-party-sources.txt" || true
  Fail "the STM32 compile database omits a first-party production source"
fi

SOURCE_COUNT="$(
  wc -l <"${BUILD_ROOT}/analyzed-first-party-sources.txt" \
    | tr -d "[:space:]"
)"
readonly SOURCE_COUNT
[[ "${SOURCE_COUNT}" -gt 0 ]] || Fail "no first-party sources were selected"

jq -e --arg pattern "${SOURCE_REGEX}" '
  [
    .[]
    | select(.file | test($pattern))
    | . as $entry
    | (.command // (.arguments | join(" "))) as $command
    | select(
        ($command | contains("-DMENTOR_PI_MOTOR_COMMISSIONING=0") | not) or
        ($command | contains("-Werror") | not) or
        ($command | contains("-DNDEBUG")) or
        (($entry.file | endswith(".cc")) and (
          ($command | contains("-fno-exceptions") | not) or
          ($command | contains("-fno-rtti") | not) or
          ($command | contains("-fno-threadsafe-statics") | not))))
  ]
  | length == 0
' "${BUILD_ROOT}/compile_commands.json" >/dev/null || \
  Fail "Debug compile commands violate locked/warning/assertion invariants"

clang-tidy-18 --verify-config
mapfile -t GCC_SYSTEM_INCLUDES < <(
  arm-none-eabi-g++ -E -x c++ - -v </dev/null 2>&1 \
    | awk '
        /search starts here:/ { in_list = 1; next }
        /End of search list./ { exit }
        in_list && /^[[:space:]]+\// {
          sub(/^[[:space:]]+/, "")
          print
        }
      ' \
    | while IFS= read -r include_path; do
        realpath "${include_path}"
      done
)
readonly GCC_SYSTEM_INCLUDES
[[ "${#GCC_SYSTEM_INCLUDES[@]}" -gt 0 ]] || \
  Fail "cannot discover the pinned Arm GNU C++ system includes"
declare -a CLANG_TIDY_TARGET_ARGS=(
  "--checks=-bugprone-dynamic-static-initializers,-bugprone-reserved-identifier,-modernize-avoid-c-arrays,-readability-inconsistent-declaration-parameter-name"
  "--extra-arg-before=--target=arm-none-eabi"
  "--extra-arg=-Wno-error"
)
for include_path in "${GCC_SYSTEM_INCLUDES[@]}"; do
  [[ -d "${include_path}" ]] || \
    Fail "Arm GNU reported a missing include directory: ${include_path}"
  CLANG_TIDY_TARGET_ARGS+=(
    "--extra-arg-before=-isystem${include_path}"
  )
done
readonly CLANG_TIDY_TARGET_ARGS

ANALYSIS_JOBS="$(nproc)"
if [[ "${ANALYSIS_JOBS}" -gt 4 ]]; then
  ANALYSIS_JOBS=4
fi
readonly ANALYSIS_JOBS
tr "\n" "\0" <"${BUILD_ROOT}/analyzed-first-party-sources.txt" \
  | xargs -0 -n 1 -P "${ANALYSIS_JOBS}" \
      clang-tidy-18 \
      --config-file=/workspace/.clang-tidy \
      "${CLANG_TIDY_TARGET_ARGS[@]}" \
      --quiet \
      -p "${BUILD_ROOT}"

CLANG_TIDY_VERSION="$(
  clang-tidy-18 --version \
    | sed -n "s/^.*LLVM version \([^[:space:]]*\).*$/\1/p" \
    | head -n 1
)"
readonly CLANG_TIDY_VERSION
[[ -n "${CLANG_TIDY_VERSION}" ]] || Fail "cannot read clang-tidy version"

printf "%s\n" \
  "target=STM32F407VET6" \
  "toolchain=arm-none-eabi-gcc-13.2.1" \
  "build_type=Debug" \
  "motor_mode=LOCKED" \
  "gcc_warnings_as_errors=1" \
  "required_strong_vector_symbols=${#REQUIRED_STRONG_VECTOR_SYMBOLS[@]}" \
  "locked_vector_symbol_strength=pass" \
  "debug_vector_symbol_strength=pass" \
  "locked_watchdog_retention=pass" \
  "debug_watchdog_retention=pass" \
  "first_party_translation_units=${SOURCE_COUNT}" \
  "clang_tidy=${CLANG_TIDY_VERSION}" \
  "clang_tidy_target_exclusions=bugprone-dynamic-static-initializers,bugprone-reserved-identifier,modernize-avoid-c-arrays,readability-inconsistent-declaration-parameter-name" \
  "result=pass" \
  >"${BUILD_ROOT}/firmware-target-analysis.txt"
