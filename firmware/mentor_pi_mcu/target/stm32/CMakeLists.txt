cmake_minimum_required(VERSION 3.22)

# This file also serves as the post-link heap audit script. Keeping the audit
# here avoids a second build-logic implementation outside the authoritative
# STM32 target.
if(DEFINED RRCLITE_HEAP_AUDIT_ELF)
  if(NOT DEFINED RRCLITE_NM OR NOT EXISTS "${RRCLITE_HEAP_AUDIT_ELF}")
    message(FATAL_ERROR "The RRCLite heap audit was invoked incorrectly")
  endif()
  execute_process(
    COMMAND "${RRCLITE_NM}" --defined-only --demangle
            "${RRCLITE_HEAP_AUDIT_ELF}"
    RESULT_VARIABLE audit_result
    OUTPUT_VARIABLE defined_symbols
    ERROR_VARIABLE audit_error
  )
  if(NOT audit_result EQUAL 0)
    message(FATAL_ERROR "arm-none-eabi-nm failed: ${audit_error}")
  endif()
  string(REGEX MATCH
    "[ \t](malloc|calloc|realloc|free|_malloc_r|_calloc_r|_realloc_r|_free_r|pvPortMalloc|vPortFree|operator new.*|operator delete.*)(\\n|$)"
    forbidden_heap_symbol "${defined_symbols}")
  if(forbidden_heap_symbol)
    string(STRIP "${forbidden_heap_symbol}" forbidden_heap_symbol)
    message(FATAL_ERROR
      "Heap-capable symbol linked into firmware: ${forbidden_heap_symbol}")
  endif()
  string(REGEX MATCH
    "[^\n]*(rosidl_typesupport_introspection|(message|service)_typesupport_(map|ids|symbol_names|data)|__(TYPE_DESCRIPTION|REFERENCED_TYPE_DESCRIPTIONS|FIELD_NAME__|TYPE_NAME))[^\n]*(\n|$)"
    forbidden_typesupport_symbol "${defined_symbols}")
  if(forbidden_typesupport_symbol)
    string(STRIP "${forbidden_typesupport_symbol}"
      forbidden_typesupport_symbol)
    message(FATAL_ERROR
      "Generic/introspection ROS typesupport linked into firmware: "
      "${forbidden_typesupport_symbol}")
  endif()
  return()
endif()

project(rrclite_v2_firmware LANGUAGES C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RRCLITE_MCU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
cmake_path(NORMAL_PATH RRCLITE_MCU_ROOT)
set(RRCLITE_REPOSITORY_ROOT "${RRCLITE_MCU_ROOT}/../..")
cmake_path(NORMAL_PATH RRCLITE_REPOSITORY_ROOT)

set(RRCLITE_CUBE_ROOT
  "${RRCLITE_MCU_ROOT}/third_party/stm32cube_f4"
  CACHE PATH "Pinned STM32CubeF4 v1.27.1 source tree")
set(RRCLITE_MICROROS_ROOT
  "${RRCLITE_MCU_ROOT}/build/microros-sdk"
  CACHE PATH "Verified checked Humble libmicroros directory")
set(RRCLITE_EXPECTED_GCC_VERSION "13.2.1" CACHE STRING
  "Exact Arm GNU compiler version used for qualified firmware builds")
set(RRCLITE_CUBE_COMMIT
  "52757b5e33259a088509a777a9e3a5b971194c7d")
set(RRCLITE_MICROROS_UTILS_COMMIT
  "bd531b273c1bcd070b3143c5642128ec75a6f04e")
set(RRCLITE_MICROROS_UTILS_ROOT
  "${RRCLITE_MCU_ROOT}/third_party/micro_ros_stm32cubemx_utils")
set(RRCLITE_LINKER_SCRIPT
  "${RRCLITE_MCU_ROOT}/linker/rrclite_stm32f407ve.ld")
set(RRCLITE_MAIN_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main.cc")
set(RRCLITE_MICROROS_LIBRARY
  "${RRCLITE_MICROROS_ROOT}/libmicroros.a")

function(rrclite_require_file required_file recovery_hint)
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR
      "Required firmware input is missing:\n"
      "  ${required_file}\n"
      "${recovery_hint}")
  endif()
endfunction()

find_package(Git REQUIRED)

function(rrclite_verify_detached_commit repository expected_commit)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${repository}" rev-parse --verify HEAD
    RESULT_VARIABLE revision_result
    OUTPUT_VARIABLE actual_commit
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT revision_result EQUAL 0 OR
     NOT actual_commit STREQUAL expected_commit)
    message(FATAL_ERROR
      "Pinned dependency revision mismatch:\n"
      "  repository: ${repository}\n"
      "  expected:   ${expected_commit}\n"
      "  actual:     ${actual_commit}\n"
      "Run make -C firmware setup.")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${repository}" status
            --porcelain=v1 --untracked-files=all
    RESULT_VARIABLE status_result
    OUTPUT_VARIABLE worktree_status
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT status_result EQUAL 0 OR NOT worktree_status STREQUAL "")
    message(FATAL_ERROR
      "Pinned dependency worktree is dirty:\n"
      "  repository: ${repository}\n"
      "Restore or re-bootstrap it before building firmware.")
  endif()
endfunction()

rrclite_verify_detached_commit(
  "${RRCLITE_CUBE_ROOT}" "${RRCLITE_CUBE_COMMIT}")
rrclite_verify_detached_commit(
  "${RRCLITE_MICROROS_UTILS_ROOT}"
  "${RRCLITE_MICROROS_UTILS_COMMIT}")
rrclite_require_file(
  "${RRCLITE_MICROROS_LIBRARY}"
  "Run make -C firmware extract-sdk before the firmware build.")
rrclite_require_file(
  "${RRCLITE_MICROROS_ROOT}/include/mentor_pi_interfaces/msg/motor_command.h"
  "Regenerate libmicroros with the project-owned interface package.")
rrclite_require_file(
  "${RRCLITE_MICROROS_ROOT}/include/mentor_pi_interfaces/srv/set_motor_model.h"
  "Regenerate libmicroros with the project-owned interface package.")
rrclite_require_file(
  "${RRCLITE_MICROROS_ROOT}/include/mentor_pi_interfaces/srv/set_motor_adrc.h"
  "Regenerate libmicroros with the project-owned interface package.")
rrclite_require_file(
  "${RRCLITE_LINKER_SCRIPT}"
  "Restore the release linker script under firmware/mentor_pi_mcu/linker.")
rrclite_require_file(
  "${RRCLITE_MAIN_SOURCE}"
  "The STM32 target main.cc must be supplied before building firmware.")

if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU" OR
   NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "RRCLite firmware requires the Arm GNU toolchain")
endif()
execute_process(
  COMMAND "${CMAKE_C_COMPILER}" -dumpfullversion
  RESULT_VARIABLE compiler_version_result
  OUTPUT_VARIABLE compiler_version
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT compiler_version_result EQUAL 0 OR
   NOT compiler_version STREQUAL RRCLITE_EXPECTED_GCC_VERSION)
  message(FATAL_ERROR
    "Arm GNU compiler mismatch: expected ${RRCLITE_EXPECTED_GCC_VERSION}, "
    "got '${compiler_version}'. Use make -C firmware build.")
endif()

foreach(required_tool CMAKE_OBJCOPY CMAKE_SIZE CMAKE_NM)
  if("${${required_tool}}" STREQUAL "")
    message(FATAL_ERROR
      "${required_tool} is not configured by the Arm toolchain file")
  endif()
  execute_process(
    COMMAND "${${required_tool}}" --version
    RESULT_VARIABLE required_tool_result
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(NOT required_tool_result EQUAL 0)
    message(FATAL_ERROR
      "Configured ${required_tool} cannot be executed: ${${required_tool}}")
  endif()
endforeach()

set(RRCLITE_DOMAIN_SOURCES
  "${RRCLITE_MCU_ROOT}/src/battery_monitor.cc"
  "${RRCLITE_MCU_ROOT}/src/bus_servo.cc"
  "${RRCLITE_MCU_ROOT}/src/button_controller.cc"
  "${RRCLITE_MCU_ROOT}/src/command_mailboxes.cc"
  "${RRCLITE_MCU_ROOT}/src/motor_controller.cc"
  "${RRCLITE_MCU_ROOT}/src/pattern_controller.cc"
  "${RRCLITE_MCU_ROOT}/src/pwm_servo_controller.cc"
  "${RRCLITE_MCU_ROOT}/src/state_merger.cc"
  "${RRCLITE_MCU_ROOT}/src/validation.cc"
)
set(RRCLITE_DRIVER_SOURCES
  "${RRCLITE_MCU_ROOT}/drivers/src/battery_adc.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/bus_servo_uart.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/gpio_peripherals.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/motor_encoder.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/pwm_servo.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/qmi8658.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/rgb_spi.cc"
  "${RRCLITE_MCU_ROOT}/drivers/src/ssd1306.cc"
)
set(RRCLITE_PLATFORM_SOURCES
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/fault_handlers.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/hal_handles.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/hal_msp.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/hal_timebase.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/interrupts.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/memory_regions.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/peripherals.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/platform.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/syscalls.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/task_entries.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/transport.cc"
  "${RRCLITE_MCU_ROOT}/platform/stm32/src/watchdog_retention.cc"
)
set(RRCLITE_MICROROS_SOURCES
  "${RRCLITE_MCU_ROOT}/app/microros/src/arena_allocator.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/reclaiming_arena.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/runtime.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/runtime_core.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/runtime_hooks.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/services.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/subscription_callbacks.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/telemetry.cc"
  "${RRCLITE_MCU_ROOT}/app/microros/src/transport_adapter.cc"
)
set(RRCLITE_MICROROS_PORT_SOURCE
  "${RRCLITE_MICROROS_UTILS_ROOT}/extra_sources/microros_time.c")
file(GLOB RRCLITE_CONTROLLER_SOURCES CONFIGURE_DEPENDS
  "${RRCLITE_MCU_ROOT}/app/controller/src/*.cc")
if(NOT RRCLITE_CONTROLLER_SOURCES)
  message(FATAL_ERROR
    "No controller sources were found under app/controller/src. "
    "The target controller implementation must exist before configuration.")
endif()

set(RRCLITE_HAL_ROOT
  "${RRCLITE_CUBE_ROOT}/Drivers/STM32F4xx_HAL_Driver")
set(RRCLITE_CMSIS_ROOT "${RRCLITE_CUBE_ROOT}/Drivers/CMSIS")
set(RRCLITE_FREERTOS_ROOT
  "${RRCLITE_CUBE_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")

set(RRCLITE_HAL_SOURCES
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_adc.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_adc_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_cortex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_dma.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_dma_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_exti.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_flash.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_flash_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_gpio.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_i2c.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_iwdg.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_pwr.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_pwr_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_rcc.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_rcc_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_spi.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_tim.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_tim_ex.c"
  "${RRCLITE_HAL_ROOT}/Src/stm32f4xx_hal_uart.c"
  "${RRCLITE_CMSIS_ROOT}/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c"
)
set(RRCLITE_FREERTOS_SOURCES
  "${RRCLITE_FREERTOS_ROOT}/list.c"
  "${RRCLITE_FREERTOS_ROOT}/queue.c"
  "${RRCLITE_FREERTOS_ROOT}/tasks.c"
  "${RRCLITE_FREERTOS_ROOT}/portable/GCC/ARM_CM4F/port.c"
)
set(RRCLITE_STARTUP_SOURCE
  "${RRCLITE_CMSIS_ROOT}/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s")
set_source_files_properties("${RRCLITE_STARTUP_SOURCE}" PROPERTIES
  COMPILE_OPTIONS -w)

foreach(source_file IN LISTS RRCLITE_DOMAIN_SOURCES RRCLITE_DRIVER_SOURCES
    RRCLITE_PLATFORM_SOURCES RRCLITE_MICROROS_SOURCES RRCLITE_HAL_SOURCES
    RRCLITE_FREERTOS_SOURCES RRCLITE_MICROROS_PORT_SOURCE)
  rrclite_require_file("${source_file}"
    "Restore the pinned source tree or the first-party implementation.")
endforeach()
rrclite_require_file("${RRCLITE_STARTUP_SOURCE}"
  "Restore the pinned STM32CubeF4 CMSIS templates.")
set_source_files_properties("${RRCLITE_MICROROS_PORT_SOURCE}" PROPERTIES
  COMPILE_OPTIONS -w)

set(RRCLITE_CPU_OPTIONS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)
set(RRCLITE_COMMON_OPTIONS
  ${RRCLITE_CPU_OPTIONS}
  -Os
  -g3
  -ffunction-sections
  -fdata-sections
  -fno-common
  -fno-unwind-tables
  -fno-asynchronous-unwind-tables
  "-ffile-prefix-map=${RRCLITE_REPOSITORY_ROOT}=."
  "-fmacro-prefix-map=${RRCLITE_REPOSITORY_ROOT}=."
)
set(RRCLITE_FIRST_PARTY_WARNINGS
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wshadow
  -Wdouble-promotion
  -Wformat=2
  -Wundef
  -Wcast-align
  -Wnull-dereference
  -Wdate-time
  -Werror
)

add_library(rrclite_hal STATIC ${RRCLITE_HAL_SOURCES})
target_compile_options(rrclite_hal PRIVATE ${RRCLITE_COMMON_OPTIONS} -w)
target_compile_definitions(rrclite_hal PUBLIC
  STM32F407xx
  USE_HAL_DRIVER
  MENTOR_PI_RRCLITE_V1
  HSE_VALUE=16000000U
  LSE_STARTUP_TIMEOUT=5000U
)

add_library(rrclite_freertos STATIC ${RRCLITE_FREERTOS_SOURCES})
target_compile_options(rrclite_freertos PRIVATE
  ${RRCLITE_COMMON_OPTIONS}
  -w
)
target_compile_definitions(rrclite_freertos PUBLIC
  STM32F407xx
  USE_HAL_DRIVER
  MENTOR_PI_RRCLITE_V1
  HSE_VALUE=16000000U
)

set(RRCLITE_VENDOR_INCLUDE_DIRECTORIES
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${RRCLITE_HAL_ROOT}/Inc"
  "${RRCLITE_HAL_ROOT}/Inc/Legacy"
  "${RRCLITE_CMSIS_ROOT}/Device/ST/STM32F4xx/Include"
  "${RRCLITE_CMSIS_ROOT}/Include"
  "${RRCLITE_FREERTOS_ROOT}/include"
  "${RRCLITE_FREERTOS_ROOT}/portable/GCC/ARM_CM4F"
  "${RRCLITE_FREERTOS_ROOT}/CMSIS_RTOS"
)
target_include_directories(rrclite_hal SYSTEM PUBLIC
  ${RRCLITE_VENDOR_INCLUDE_DIRECTORIES})
target_include_directories(rrclite_freertos SYSTEM PUBLIC
  ${RRCLITE_VENDOR_INCLUDE_DIRECTORIES})

add_library(rrclite_microros STATIC IMPORTED GLOBAL)
set_target_properties(rrclite_microros PROPERTIES
  IMPORTED_LOCATION "${RRCLITE_MICROROS_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${RRCLITE_MICROROS_ROOT}/include"
)

set(RRCLITE_EXECUTABLE_SOURCES
  "${RRCLITE_MAIN_SOURCE}"
  ${RRCLITE_DOMAIN_SOURCES}
  ${RRCLITE_DRIVER_SOURCES}
  ${RRCLITE_PLATFORM_SOURCES}
  ${RRCLITE_CONTROLLER_SOURCES}
  ${RRCLITE_MICROROS_SOURCES}
  "${RRCLITE_MICROROS_PORT_SOURCE}"
  "${RRCLITE_STARTUP_SOURCE}"
)
add_executable(mentor_pi_mcu ${RRCLITE_EXECUTABLE_SOURCES})
set(MENTOR_PI_ROS_NAMESPACE "" CACHE STRING
  "Absolute ROS namespace compiled into the MCU micro-ROS node")
if(NOT MENTOR_PI_ROS_NAMESPACE MATCHES
   "^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$")
  message(FATAL_ERROR "MENTOR_PI_ROS_NAMESPACE must be an absolute ROS namespace")
endif()
set_target_properties(mentor_pi_mcu PROPERTIES
  OUTPUT_NAME mentor_pi_mcu
  SUFFIX .elf
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
target_compile_features(mentor_pi_mcu PRIVATE cxx_std_17)
target_compile_options(mentor_pi_mcu PRIVATE
  ${RRCLITE_COMMON_OPTIONS}
  ${RRCLITE_FIRST_PARTY_WARNINGS}
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)
target_compile_definitions(mentor_pi_mcu PRIVATE
  STM32F407xx
  USE_HAL_DRIVER
  MENTOR_PI_RRCLITE_V1
  CLOCK_MONOTONIC=0
  MENTOR_PI_ROS_NAMESPACE="${MENTOR_PI_ROS_NAMESPACE}"
)
target_include_directories(mentor_pi_mcu PRIVATE
  "${RRCLITE_MICROROS_ROOT}/include"
  "${RRCLITE_MCU_ROOT}/include"
  "${RRCLITE_MCU_ROOT}/drivers/include"
  "${RRCLITE_MCU_ROOT}/app/controller/include"
  "${RRCLITE_MCU_ROOT}/app/microros/include"
  "${RRCLITE_MCU_ROOT}/platform/stm32/include"
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_include_directories(mentor_pi_mcu SYSTEM PRIVATE
  ${RRCLITE_VENDOR_INCLUDE_DIRECTORIES}
  "${RRCLITE_MICROROS_ROOT}/include"
)

target_link_libraries(mentor_pi_mcu PRIVATE
  -Wl,--start-group
  rrclite_hal
  rrclite_freertos
  rrclite_microros
  m
  -Wl,--end-group
)
target_link_options(mentor_pi_mcu PRIVATE
  ${RRCLITE_CPU_OPTIONS}
  --specs=nano.specs
  -nostdlib++
  -static
  "-T${RRCLITE_LINKER_SCRIPT}"
  -Wl,--gc-sections
  -Wl,--build-id=none
  -Wl,--fatal-warnings
  -Wl,--cref
  "-Wl,-Map=${CMAKE_BINARY_DIR}/mentor_pi_mcu.map"
  -Wl,--wrap=malloc
  -Wl,--wrap=calloc
  -Wl,--wrap=realloc
  -Wl,--wrap=free
  -Wl,--wrap=_malloc_r
  -Wl,--wrap=_calloc_r
  -Wl,--wrap=_realloc_r
  -Wl,--wrap=_free_r
  -Wl,--wrap=pvPortMalloc
  -Wl,--wrap=vPortFree
  -Wl,--wrap=_Znwj
  -Wl,--wrap=_Znaj
  -Wl,--wrap=_ZdlPv
  -Wl,--wrap=_ZdaPv
  -Wl,--wrap=_ZdlPvj
  -Wl,--wrap=_ZdaPvj
)
set_property(TARGET mentor_pi_mcu APPEND PROPERTY
  LINK_DEPENDS "${RRCLITE_LINKER_SCRIPT}")

add_custom_command(TARGET mentor_pi_mcu POST_BUILD
  COMMAND "${CMAKE_COMMAND}"
          "-DRRCLITE_HEAP_AUDIT_ELF=$<TARGET_FILE:mentor_pi_mcu>"
          "-DRRCLITE_NM=${CMAKE_NM}"
          -P "${CMAKE_CURRENT_LIST_FILE}"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex
          "$<TARGET_FILE:mentor_pi_mcu>"
          "${CMAKE_BINARY_DIR}/mentor_pi_mcu.hex"
  COMMAND "${CMAKE_OBJCOPY}" -O binary
          "$<TARGET_FILE:mentor_pi_mcu>"
          "${CMAKE_BINARY_DIR}/mentor_pi_mcu.bin"
  COMMAND "${CMAKE_SIZE}" --format=berkeley
          "$<TARGET_FILE:mentor_pi_mcu>"
  BYPRODUCTS
    "${CMAKE_BINARY_DIR}/mentor_pi_mcu.hex"
    "${CMAKE_BINARY_DIR}/mentor_pi_mcu.bin"
    "${CMAKE_BINARY_DIR}/mentor_pi_mcu.map"
  COMMENT "Auditing and generating RRCLite ELF/HEX/BIN artifacts"
  VERBATIM
)

message(STATUS "RRCLite target: STM32F407VET6")
message(STATUS "RRCLite motor mode: ADRC CLOSED_LOOP (6 RPS, ±1000 permille)")
message(STATUS "STM32CubeF4: ${RRCLITE_CUBE_COMMIT}")
message(STATUS "libmicroros: ${RRCLITE_MICROROS_LIBRARY}")
message(STATUS "linker script: ${RRCLITE_LINKER_SCRIPT}")
