cmake_minimum_required(VERSION 3.16)
project(mentor_pi_mcu_controller LANGUAGES CXX)

option(MENTOR_PI_MCU_CONTROLLER_SANITIZERS
       "Enable address and undefined-behavior sanitizers" ON)

if(NOT TARGET mentor_pi_mcu_domain)
  add_subdirectory(../.. domain_build)
endif()
if(NOT TARGET mentor_pi_mcu_drivers)
  add_subdirectory(../../drivers drivers_build)
endif()

add_library(mentor_pi_mcu_controller STATIC
  src/bus_servo_worker.cc
  src/command_gateway.cc
  src/controller_runtime.cc
  src/diagnostics.cc
  src/imu_characterization.cc
  src/microros_adapter.cc
  src/motor_worker.cc
  src/peripheral_worker.cc
  src/platform_hooks.cc
  src/sensor_worker.cc
)
target_include_directories(mentor_pi_mcu_controller PUBLIC
  include
  ../microros/include
)
target_link_libraries(mentor_pi_mcu_controller
  PUBLIC mentor_pi_mcu_domain mentor_pi_mcu_drivers)
target_compile_features(mentor_pi_mcu_controller PUBLIC cxx_std_17)

set(MENTOR_PI_MCU_CONTROLLER_WARNINGS
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
target_compile_options(mentor_pi_mcu_controller PRIVATE
  ${MENTOR_PI_MCU_CONTROLLER_WARNINGS}
  -fno-exceptions
  -fno-rtti
)

include(CTest)
if(BUILD_TESTING)
  add_executable(mentor_pi_mcu_controller_tests
    tests/controller_tests.cc
    ../microros/src/runtime_hooks.cc
  )
  target_link_libraries(mentor_pi_mcu_controller_tests
    PRIVATE mentor_pi_mcu_controller)
  target_compile_features(mentor_pi_mcu_controller_tests PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_mcu_controller_tests PRIVATE
    ${MENTOR_PI_MCU_CONTROLLER_WARNINGS})
  if(MENTOR_PI_MCU_CONTROLLER_SANITIZERS AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_controller_tests PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer)
    target_link_options(mentor_pi_mcu_controller_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
  add_test(NAME mentor_pi_mcu_controller_tests
           COMMAND mentor_pi_mcu_controller_tests)
  add_test(
    NAME mentor_pi_mcu_stm32_motor_transport_gate
    COMMAND
      ${CMAKE_COMMAND}
      -DTARGET_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/../../target/stm32/main.cc
      -DTRANSPORT_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/../../platform/stm32/src/transport.cc
      -DMOTOR_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/../../include/mentor_pi_mcu/domain/motor_controller.h
      -DBUILD_WRAPPER=${CMAKE_CURRENT_SOURCE_DIR}/../../../../tools/build_firmware.sh
      -DTARGET_CMAKE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/../../target/stm32
      -DPROFILE_PROBE_ROOT=${CMAKE_CURRENT_BINARY_DIR}/motor-profile-gate-probe
      -P
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stm32_motor_transport_gate.cmake
  )
endif()
