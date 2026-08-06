cmake_minimum_required(VERSION 3.16)
project(mentor_pi_microros_runtime LANGUAGES CXX)

set(MENTOR_PI_MCU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(MENTOR_PI_MICROROS_INCLUDE_DIR
    "${MENTOR_PI_MCU_ROOT}/build/microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include"
    CACHE PATH "Path to generated Humble libmicroros headers")

set(MENTOR_PI_WARNING_OPTIONS
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

add_library(mentor_pi_microros_core STATIC src/runtime_core.cc)
target_include_directories(mentor_pi_microros_core PUBLIC
  include
  "${MENTOR_PI_MCU_ROOT}/platform/stm32/include"
)
target_compile_features(mentor_pi_microros_core PUBLIC cxx_std_17)
target_compile_options(mentor_pi_microros_core PRIVATE
  ${MENTOR_PI_WARNING_OPTIONS}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(mentor_pi_microros_core_tests tests/runtime_core_tests.cc)
  target_link_libraries(mentor_pi_microros_core_tests PRIVATE
    mentor_pi_microros_core
  )
  target_compile_features(mentor_pi_microros_core_tests PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_microros_core_tests PRIVATE
    ${MENTOR_PI_WARNING_OPTIONS}
  )
  add_test(NAME mentor_pi_microros_core_tests
           COMMAND mentor_pi_microros_core_tests)
endif()

# This object target checks every runtime translation unit against the actual
# generated C types and Humble rcl/rclc/rmw declarations. The final firmware
# target supplies libmicroros.a, HAL/FreeRTOS objects, and the linker script.
if(EXISTS "${MENTOR_PI_MICROROS_INCLUDE_DIR}/rcl/rcl.h" AND
   EXISTS "${MENTOR_PI_MICROROS_INCLUDE_DIR}/mentor_pi_interfaces/msg/motor_command.h")
  add_library(mentor_pi_microros_compile_check OBJECT
    src/arena_allocator.cc
    src/runtime.cc
    src/runtime_core.cc
    src/runtime_hooks.cc
    src/services.cc
    src/subscription_callbacks.cc
    src/telemetry.cc
    src/transport_adapter.cc
  )
  target_include_directories(mentor_pi_microros_compile_check PRIVATE
    include
    src
    "${MENTOR_PI_MCU_ROOT}/include"
    "${MENTOR_PI_MCU_ROOT}/platform/stm32/include"
    "${MENTOR_PI_MICROROS_INCLUDE_DIR}"
  )
  target_compile_features(mentor_pi_microros_compile_check PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_microros_compile_check PRIVATE
    ${MENTOR_PI_WARNING_OPTIONS}
  )
endif()
