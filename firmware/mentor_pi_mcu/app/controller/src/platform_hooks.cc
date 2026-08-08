// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/controller/platform_hooks.h"

namespace mentor_pi_mcu::app::controller {

bool PlatformHooksAreComplete(const PlatformHooks& hooks) {
  return hooks.monotonic_milliseconds != nullptr &&
         hooks.monotonic_microseconds != nullptr &&
         hooks.wait_for_task != nullptr && hooks.enter_critical != nullptr &&
         hooks.exit_critical != nullptr &&
         hooks.emergency_stop_motors != nullptr &&
         hooks.refresh_watchdog != nullptr &&
         hooks.persist_watchdog_task != nullptr &&
         hooks.initialize_motor_outputs != nullptr &&
         hooks.arm_motor != nullptr && hooks.disarm_motor != nullptr &&
         hooks.read_encoder_counters != nullptr &&
         hooks.apply_motor_duty != nullptr &&
         hooks.initialize_pwm_servos != nullptr &&
         hooks.set_pwm_servo_shadow != nullptr &&
         hooks.pwm_servo_frame_sequence != nullptr &&
         hooks.read_button_pressed != nullptr && hooks.set_led != nullptr &&
         hooks.set_buzzer != nullptr && hooks.take_battery_sample != nullptr &&
         hooks.register_i2c_read != nullptr &&
         hooks.register_i2c_write != nullptr &&
         hooks.raw_i2c_write != nullptr &&
         hooks.bus_uart_begin_exchange != nullptr &&
         hooks.bus_uart_poll_exchange != nullptr &&
         hooks.bus_uart_cancel != nullptr &&
         hooks.rgb_spi_begin_transmit != nullptr &&
         hooks.rgb_spi_poll_transmit != nullptr &&
         hooks.rgb_spi_cancel != nullptr &&
         hooks.read_transport_activity != nullptr &&
         hooks.task_stack_high_water_bytes != nullptr &&
         hooks.read_memory_metrics != nullptr &&
         hooks.flash_used_bytes != nullptr &&
         hooks.flash_total_bytes != nullptr &&
         hooks.last_reset_reason != nullptr &&
         hooks.captured_watchdog_task != nullptr;
}

}  // namespace mentor_pi_mcu::app::controller
