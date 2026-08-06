// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_CONTROLLER_PLATFORM_HOOKS_H_
#define MENTOR_PI_MCU_APP_CONTROLLER_PLATFORM_HOOKS_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/battery_adc.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi_mcu::app::controller {

// Values deliberately match ControllerDiagnostics.TASK_* and the STM32 task
// table. Portable controller code does not include FreeRTOS or STM32 headers.
enum class ControllerTask : std::uint8_t {
  kSafetySupervisor = 0,
  kMotorControl = 1,
  kMicroRos = 2,
  kBusServo = 3,
  kSensor = 4,
  kPeripheral = 5,
  kCount = 6,
};

inline constexpr std::size_t kControllerTaskCount =
    static_cast<std::size_t>(ControllerTask::kCount);

struct BatterySample {
  mentor_pi::mcu::Result result{};
  mentor_pi::mcu::drivers::BatteryAdcReading reading{};
  // false with OK/BUSY means conversion is in progress and is not an invalid
  // measurement. A non-OK terminal result is still fed to the validity logic.
  bool available{false};
};

// Every function is bounded and allocation-free. Deadlines are absolute
// monotonic values in the unit named by the field. Target glue may delegate to
// STM32 HAL/platform functions or to the portable driver HAL interfaces.
struct PlatformHooks {
  void* context{nullptr};

  std::uint32_t (*monotonic_milliseconds)(void* context){nullptr};
  std::uint32_t (*monotonic_microseconds)(void* context){nullptr};
  void (*wait_for_task)(void* context, ControllerTask task,
                        std::uint32_t maximum_ms){nullptr};
  void (*enter_critical)(void* context){nullptr};
  void (*exit_critical)(void* context){nullptr};

  // Exactly SafetySupervisorTask calls refresh_watchdog. Emergency stop is a
  // register-only override and must be safe from task or interrupt context.
  void (*emergency_stop_motors)(void* context){nullptr};
  mentor_pi::mcu::Result (*refresh_watchdog)(void* context){nullptr};
  void (*persist_watchdog_task)(void* context, ControllerTask task){nullptr};

  mentor_pi::mcu::Result (*initialize_motor_outputs)(void* context){nullptr};
  mentor_pi::mcu::Result (*arm_motor)(void* context,
                                      std::size_t motor_index){nullptr};
  void (*disarm_motor)(void* context, std::size_t motor_index){nullptr};
  bool (*read_encoder_counters)(
      void* context,
      std::array<std::uint32_t, mentor_pi::mcu::kMotorCount>* counters){
      nullptr};
  mentor_pi::mcu::Result (*apply_motor_duty)(
      void* context,
      const std::array<std::int16_t, mentor_pi::mcu::kMotorCount>&
          duty_permille){nullptr};

  mentor_pi::mcu::Result (*initialize_pwm_servos)(void* context){nullptr};
  mentor_pi::mcu::Result (*set_pwm_servo_shadow)(
      void* context,
      const std::array<std::uint16_t, mentor_pi::mcu::kPwmServoCount>&
          pulse_width_us){nullptr};
  std::uint32_t (*pwm_servo_frame_sequence)(void* context){nullptr};

  bool (*read_button_pressed)(void* context, std::size_t button_index){nullptr};
  // set_led is semantic on/off; the controller owns active-low polarity.
  void (*set_led)(void* context, std::size_t led_index, bool on){nullptr};
  // set_buzzer completes synchronously; a non-OK result means the requested
  // output was not established.
  mentor_pi::mcu::Result (*set_buzzer)(void* context,
                                       std::uint16_t frequency_hz,
                                       bool on){nullptr};
  BatterySample (*take_battery_sample)(void* context,
                                       std::uint32_t now_ms){nullptr};

  mentor_pi::mcu::drivers::IoStatus (*register_i2c_read)(
      void* context, std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
      std::size_t size, std::uint32_t deadline_us){nullptr};
  mentor_pi::mcu::drivers::IoStatus (*register_i2c_write)(
      void* context, std::uint8_t address, std::uint8_t reg,
      const std::uint8_t* data, std::size_t size,
      std::uint32_t deadline_us){nullptr};
  mentor_pi::mcu::drivers::IoStatus (*raw_i2c_write)(
      void* context, std::uint8_t address, const std::uint8_t* data,
      std::size_t size, std::uint32_t deadline_ms){nullptr};

  mentor_pi::mcu::drivers::IoStatus (*bus_uart_begin_exchange)(
      void* context, const std::uint8_t* tx, std::size_t tx_size,
      std::size_t max_reply_size, std::uint32_t deadline_ms){nullptr};
  mentor_pi::mcu::drivers::IoStatus (*bus_uart_poll_exchange)(
      void* context, std::uint32_t now_ms, std::uint8_t* reply,
      std::size_t capacity, std::size_t* reply_size){nullptr};
  void (*bus_uart_cancel)(void* context){nullptr};

  mentor_pi::mcu::drivers::IoStatus (*rgb_spi_begin_transmit)(
      void* context, const std::uint8_t* data, std::size_t size,
      std::uint32_t deadline_us){nullptr};
  mentor_pi::mcu::drivers::IoStatus (*rgb_spi_poll_transmit)(
      void* context, std::uint32_t now_us){nullptr};
  void (*rgb_spi_cancel)(void* context){nullptr};

  std::uint32_t (*task_stack_high_water_bytes)(void* context,
                                               ControllerTask task){nullptr};
  void (*read_memory_metrics)(
      void* context, std::array<std::uint32_t, 2>* free_ram_bytes,
      std::array<std::uint32_t, 2>* minimum_free_ram_bytes){nullptr};
  std::uint32_t (*flash_used_bytes)(void* context){nullptr};
  std::uint32_t (*flash_total_bytes)(void* context){nullptr};
  std::uint8_t (*last_reset_reason)(void* context){nullptr};
  std::uint8_t (*captured_watchdog_task)(void* context){nullptr};
};

bool PlatformHooksAreComplete(const PlatformHooks& hooks);

}  // namespace mentor_pi_mcu::app::controller

#endif  // MENTOR_PI_MCU_APP_CONTROLLER_PLATFORM_HOOKS_H_
