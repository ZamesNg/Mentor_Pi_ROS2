cmake_minimum_required(VERSION 3.16)
project(mentor_pi_mcu_domain LANGUAGES CXX)

include(CTest)

option(MENTOR_PI_MCU_ENABLE_SANITIZERS
       "Enable AddressSanitizer and UndefinedBehaviorSanitizer" ON)
option(MENTOR_PI_MCU_ENABLE_TSAN
       "Enable ThreadSanitizer on the concurrency stress test" OFF)
option(MENTOR_PI_MCU_ENABLE_FUZZING
       "Build opt-in Clang libFuzzer harnesses with ASan/UBSan" OFF)
set(MENTOR_PI_MCU_FUZZ_SANITIZERS "fuzzer,address,undefined" CACHE STRING
    "Comma-separated Clang sanitizers for fuzz targets")
set(MENTOR_PI_MCU_FUZZ_SMOKE_RUNS "10000" CACHE STRING
    "Per-harness input count for the bounded fuzz smoke target")

set(MENTOR_PI_MICROROS_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/build/microros-sdk" CACHE PATH
    "Verified checked Humble micro-ROS firmware SDK")
set(MENTOR_PI_INTERFACE_CONTRACT_INCLUDE_DIR
    "${MENTOR_PI_MICROROS_SDK_ROOT}/include")
set(MENTOR_PI_MOTOR_PROFILE_CONTRACT_HEADER
    "${MENTOR_PI_INTERFACE_CONTRACT_INCLUDE_DIR}/mentor_pi_interfaces/motor_profile_contract.hpp")
if(NOT EXISTS "${MENTOR_PI_MOTOR_PROFILE_CONTRACT_HEADER}")
  message(FATAL_ERROR
    "Missing checked SDK motor profile contract: ${MENTOR_PI_MOTOR_PROFILE_CONTRACT_HEADER}. Run make -C firmware extract-sdk.")
endif()
add_library(mentor_pi_mcu_interface_contract INTERFACE)
target_include_directories(mentor_pi_mcu_interface_contract INTERFACE
  "${MENTOR_PI_INTERFACE_CONTRACT_INCLUDE_DIR}"
)

add_library(mentor_pi_mcu_domain STATIC
  src/battery_monitor.cc
  src/bus_servo.cc
  src/button_controller.cc
  src/command_mailboxes.cc
  src/motor_controller.cc
  src/pattern_controller.cc
  src/pwm_servo_controller.cc
  src/state_merger.cc
  src/validation.cc
)
target_include_directories(mentor_pi_mcu_domain
  PUBLIC include
)
target_link_libraries(mentor_pi_mcu_domain
  PRIVATE mentor_pi_mcu_interface_contract)
target_compile_features(mentor_pi_mcu_domain PUBLIC cxx_std_17)
target_compile_options(mentor_pi_mcu_domain PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wshadow
  -Wdouble-promotion
  -Wformat=2
  -Werror
  -fno-exceptions
  -fno-rtti
)

if(BUILD_TESTING OR MENTOR_PI_MCU_ENABLE_FUZZING)
  add_executable(mentor_pi_mcu_validation_semantic_contract_tests
    tests/fuzz/validation_semantic_contract_tests.cc
    tests/fuzz/validation_oracle_fuzz.cc
  )
  target_include_directories(mentor_pi_mcu_validation_semantic_contract_tests
    PRIVATE include tests/fuzz)
  target_link_libraries(mentor_pi_mcu_validation_semantic_contract_tests
    PRIVATE mentor_pi_mcu_domain)
  target_compile_features(mentor_pi_mcu_validation_semantic_contract_tests
    PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_mcu_validation_semantic_contract_tests
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wshadow
      -Wdouble-promotion
      -Wformat=2
      -Werror
      -fno-exceptions
      -fno-rtti
  )
endif()

if(MENTOR_PI_MCU_ENABLE_FUZZING)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR
      "MENTOR_PI_MCU_ENABLE_FUZZING requires Clang libFuzzer support")
  endif()
  if(NOT MENTOR_PI_MCU_FUZZ_SANITIZERS MATCHES "(^|,)fuzzer($|,)")
    message(FATAL_ERROR
      "MENTOR_PI_MCU_FUZZ_SANITIZERS must include fuzzer")
  endif()

  function(add_mentor_pi_mcu_fuzzer target)
    add_executable(${target} ${ARGN})
    target_include_directories(${target} PRIVATE include tests/fuzz)
    target_link_libraries(${target} PRIVATE mentor_pi_mcu_interface_contract)
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_compile_options(${target} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wshadow
      -Wdouble-promotion
      -Wformat=2
      -Werror
      -fno-exceptions
      -fno-rtti
      -fno-omit-frame-pointer
      -fsanitize=${MENTOR_PI_MCU_FUZZ_SANITIZERS}
    )
    target_link_options(${target} PRIVATE
      -fsanitize=${MENTOR_PI_MCU_FUZZ_SANITIZERS}
    )
  endfunction()

  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_validation_state_merge_fuzz
    tests/fuzz/validation_state_merge_fuzz.cc
    src/state_merger.cc
    src/validation.cc
  )
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_bus_servo_codec_fuzz
    tests/fuzz/bus_servo_codec_fuzz.cc
    src/bus_servo.cc
    src/validation.cc
  )
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_validation_oracle_fuzz
    tests/fuzz/validation_oracle_fuzz.cc
    src/validation.cc
  )
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_command_mailboxes_fuzz
    tests/fuzz/command_mailboxes_fuzz.cc
    src/command_mailboxes.cc
    src/state_merger.cc
    src/validation.cc
  )
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_controller_state_fuzz
    tests/fuzz/controller_state_fuzz.cc
    src/battery_monitor.cc
    src/bus_servo.cc
    src/motor_controller.cc
    src/pattern_controller.cc
    src/pwm_servo_controller.cc
    src/validation.cc
  )
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_bus_servo_driver_fuzz
    tests/fuzz/bus_servo_driver_fuzz.cc
    drivers/src/bus_servo_uart.cc
    src/bus_servo.cc
    src/validation.cc
  )
  target_include_directories(mentor_pi_mcu_bus_servo_driver_fuzz
    PRIVATE drivers/include)
  add_mentor_pi_mcu_fuzzer(mentor_pi_mcu_usart1_rx_ring_fuzz
    tests/fuzz/usart1_rx_ring_fuzz.cc
  )

  set(MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS
      "${CMAKE_CURRENT_BINARY_DIR}/fuzz-smoke-corpus")
  add_custom_target(mentor_pi_mcu_fuzz_smoke
    COMMAND ${CMAKE_COMMAND} -E remove_directory
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}"
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_state_merge"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_codec"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_oracle"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/command_mailboxes"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/controller_state"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_driver"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/usart1_rx_ring"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/validation_state_merge"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_state_merge"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/bus_servo_codec"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_codec"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/validation_state_merge"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_oracle"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/validation_oracle"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_oracle"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/validation_state_merge"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/command_mailboxes"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/command_mailboxes"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/command_mailboxes"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/validation_state_merge"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/controller_state"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/controller_state"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/controller_state"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/bus_servo_codec"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_driver"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/bus_servo_driver"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_driver"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fuzz/corpus/usart1_rx_ring"
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/usart1_rx_ring"
    COMMAND $<TARGET_FILE:mentor_pi_mcu_validation_semantic_contract_tests>
    COMMAND $<TARGET_FILE:mentor_pi_mcu_validation_state_merge_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_state_merge"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=512
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_bus_servo_codec_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_codec"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=64
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_validation_oracle_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/validation_oracle"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=512
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_command_mailboxes_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/command_mailboxes"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=512
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_controller_state_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/controller_state"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=512
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_bus_servo_driver_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/bus_servo_driver"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=512
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    COMMAND $<TARGET_FILE:mentor_pi_mcu_usart1_rx_ring_fuzz>
            "${MENTOR_PI_MCU_FUZZ_SMOKE_CORPUS}/usart1_rx_ring"
            -runs=${MENTOR_PI_MCU_FUZZ_SMOKE_RUNS}
            -max_len=16384
            -seed=424242
            -timeout=5
            -print_final_stats=1
            -verbosity=0
    DEPENDS
      mentor_pi_mcu_validation_state_merge_fuzz
      mentor_pi_mcu_validation_semantic_contract_tests
      mentor_pi_mcu_bus_servo_codec_fuzz
      mentor_pi_mcu_validation_oracle_fuzz
      mentor_pi_mcu_command_mailboxes_fuzz
      mentor_pi_mcu_controller_state_fuzz
      mentor_pi_mcu_bus_servo_driver_fuzz
      mentor_pi_mcu_usart1_rx_ring_fuzz
    USES_TERMINAL
  )
endif()

if(BUILD_TESTING)
  if(MENTOR_PI_MCU_ENABLE_SANITIZERS AND
     NOT CMAKE_CROSSCOMPILING AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_domain PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mentor_pi_mcu_domain INTERFACE
      -fsanitize=address,undefined
    )
  endif()

  add_executable(mentor_pi_mcu_domain_tests tests/domain_tests.cc)
  target_sources(mentor_pi_mcu_domain_tests PRIVATE
    app/microros/src/reclaiming_arena.cc)
  target_link_libraries(mentor_pi_mcu_domain_tests PRIVATE mentor_pi_mcu_domain)
  target_include_directories(mentor_pi_mcu_domain_tests PRIVATE
    app/microros/include
    platform/stm32/include)
  target_compile_features(mentor_pi_mcu_domain_tests PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_mcu_domain_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wdouble-promotion
    -Wformat=2
    -Werror
  )
  if(MENTOR_PI_MCU_ENABLE_SANITIZERS AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_domain_tests PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mentor_pi_mcu_domain_tests PRIVATE
      -fsanitize=address,undefined
    )
  endif()
  add_test(NAME mentor_pi_mcu_domain_tests COMMAND mentor_pi_mcu_domain_tests)

  if(MENTOR_PI_MCU_ENABLE_SANITIZERS AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_validation_semantic_contract_tests
      PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer
    )
    target_link_options(mentor_pi_mcu_validation_semantic_contract_tests
      PRIVATE -fsanitize=address,undefined)
  endif()
  add_test(NAME mentor_pi_mcu_validation_semantic_contract_tests
           COMMAND mentor_pi_mcu_validation_semantic_contract_tests)

  find_package(Threads REQUIRED)
  add_executable(mentor_pi_mcu_concurrency_tests tests/concurrency_tests.cc)
  target_include_directories(mentor_pi_mcu_concurrency_tests PRIVATE include)
  target_compile_features(mentor_pi_mcu_concurrency_tests PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_mcu_concurrency_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Werror
  )
  target_link_libraries(mentor_pi_mcu_concurrency_tests PRIVATE Threads::Threads)
  if(MENTOR_PI_MCU_ENABLE_TSAN AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_concurrency_tests PRIVATE
      -fsanitize=thread
      -fno-omit-frame-pointer
    )
    target_link_options(mentor_pi_mcu_concurrency_tests PRIVATE
      -fsanitize=thread
    )
  endif()
  add_test(NAME mentor_pi_mcu_concurrency_tests
           COMMAND mentor_pi_mcu_concurrency_tests)
  add_executable(mentor_pi_mcu_watchdog_retention_tests
    tests/watchdog_retention_tests.cc
    platform/stm32/src/watchdog_retention.cc
  )
  target_include_directories(mentor_pi_mcu_watchdog_retention_tests PRIVATE
    platform/stm32/include)
  target_compile_features(mentor_pi_mcu_watchdog_retention_tests PRIVATE
    cxx_std_17)
  target_compile_options(mentor_pi_mcu_watchdog_retention_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wdouble-promotion
    -Wformat=2
    -Werror
  )
  if(MENTOR_PI_MCU_ENABLE_SANITIZERS AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_watchdog_retention_tests PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer)
    target_link_options(mentor_pi_mcu_watchdog_retention_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
  add_test(NAME mentor_pi_mcu_watchdog_retention_tests
           COMMAND mentor_pi_mcu_watchdog_retention_tests)
  add_test(
    NAME mentor_pi_mcu_stm32_rx_dma_contract
    COMMAND
      ${CMAKE_COMMAND}
      -DTRANSPORT_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/transport.cc
      -DINTERRUPTS_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/interrupts.cc
      -DMSP_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/hal_msp.cc
      -DADAPTER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/app/microros/src/transport_adapter.cc
      -P
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stm32_rx_dma_contract.cmake
  )
  add_test(
    NAME mentor_pi_mcu_stm32_peripheral_safety_contract
    COMMAND
      ${CMAKE_COMMAND}
      -DTARGET_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/target/stm32/main.cc
      -DPERIPHERAL_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/peripherals.cc
      -P
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stm32_peripheral_safety_contract.cmake
  )
  add_test(
    NAME mentor_pi_mcu_stm32_timebase_contract
    COMMAND
      ${CMAKE_COMMAND}
      -DTIMEBASE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/hal_timebase.cc
      -P
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stm32_timebase_contract.cmake
  )
  add_test(
    NAME mentor_pi_mcu_stm32_watchdog_retention_contract
    COMMAND
      ${CMAKE_COMMAND}
      -DPLATFORM_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32/src/platform.cc
      -DCONTROLLER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/app/controller/src/controller_runtime.cc
      -DLINKER_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/linker/rrclite_stm32f407ve.ld
      -DTARGET_CMAKE=${CMAKE_CURRENT_SOURCE_DIR}/target/stm32/CMakeLists.txt
      -DTARGET_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/target/stm32/main.cc
      -P
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stm32_watchdog_retention_contract.cmake
  )
endif()
