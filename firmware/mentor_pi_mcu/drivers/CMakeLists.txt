cmake_minimum_required(VERSION 3.16)
project(mentor_pi_mcu_drivers LANGUAGES CXX)

option(MENTOR_PI_MCU_DRIVER_SANITIZERS
       "Enable address and undefined-behavior sanitizers" ON)

if(NOT TARGET mentor_pi_mcu_domain)
  add_library(mentor_pi_mcu_driver_domain_support STATIC
    ../src/bus_servo.cc
    ../src/validation.cc
  )
  target_include_directories(mentor_pi_mcu_driver_domain_support
    PUBLIC ../include)
  target_compile_features(mentor_pi_mcu_driver_domain_support PUBLIC cxx_std_17)
  set(MENTOR_PI_MCU_DRIVER_DOMAIN_TARGET mentor_pi_mcu_driver_domain_support)
else()
  set(MENTOR_PI_MCU_DRIVER_DOMAIN_TARGET mentor_pi_mcu_domain)
endif()

add_library(mentor_pi_mcu_drivers STATIC
  src/battery_adc.cc
  src/bus_servo_uart.cc
  src/gpio_peripherals.cc
  src/motor_encoder.cc
  src/pwm_servo.cc
  src/qmi8658.cc
  src/rgb_spi.cc
  src/ssd1306.cc
)
target_include_directories(mentor_pi_mcu_drivers PUBLIC include ../include)
target_link_libraries(mentor_pi_mcu_drivers
  PUBLIC ${MENTOR_PI_MCU_DRIVER_DOMAIN_TARGET})
target_compile_features(mentor_pi_mcu_drivers PUBLIC cxx_std_17)

set(MENTOR_PI_MCU_DRIVER_WARNINGS
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
target_compile_options(mentor_pi_mcu_drivers PRIVATE
  ${MENTOR_PI_MCU_DRIVER_WARNINGS}
  -fno-exceptions
  -fno-rtti
)

include(CTest)
if(BUILD_TESTING)
  if(MENTOR_PI_MCU_DRIVER_SANITIZERS AND
     NOT CMAKE_CROSSCOMPILING AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_drivers PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer)
    target_link_options(mentor_pi_mcu_drivers INTERFACE
      -fsanitize=address,undefined)
    if(TARGET mentor_pi_mcu_driver_domain_support)
      target_compile_options(mentor_pi_mcu_driver_domain_support PRIVATE
        -fsanitize=address,undefined
        -fno-omit-frame-pointer)
      target_link_options(mentor_pi_mcu_driver_domain_support INTERFACE
        -fsanitize=address,undefined)
    endif()
  endif()

  add_executable(mentor_pi_mcu_driver_tests tests/driver_tests.cc)
  target_link_libraries(mentor_pi_mcu_driver_tests PRIVATE mentor_pi_mcu_drivers)
  target_compile_features(mentor_pi_mcu_driver_tests PRIVATE cxx_std_17)
  target_compile_options(mentor_pi_mcu_driver_tests PRIVATE
    ${MENTOR_PI_MCU_DRIVER_WARNINGS})
  if(MENTOR_PI_MCU_DRIVER_SANITIZERS AND
     CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mentor_pi_mcu_driver_tests PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer)
    target_link_options(mentor_pi_mcu_driver_tests PRIVATE
      -fsanitize=address,undefined)
  endif()
  add_test(NAME mentor_pi_mcu_driver_tests COMMAND mentor_pi_mcu_driver_tests)
endif()
