// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/runtime_hooks.h"

namespace mentor_pi_mcu::app::microros {

bool RuntimeHooksAreComplete(const RuntimeHooks& hooks) {
  return hooks.monotonic_milliseconds != nullptr &&
         hooks.monotonic_microseconds != nullptr &&
         hooks.wait_milliseconds != nullptr &&
         hooks.advance_task_heartbeat != nullptr &&
         hooks.emergency_stop_motors != nullptr &&
         hooks.set_session_active != nullptr &&
         hooks.invalidate_session_work != nullptr &&
         hooks.motor_max_rps != nullptr &&
         hooks.publish_motor_command != nullptr &&
         hooks.publish_pwm_servo_command != nullptr &&
         hooks.publish_bus_servo_command != nullptr &&
         hooks.publish_led_command != nullptr &&
         hooks.publish_buzzer_command != nullptr &&
         hooks.publish_rgb_command != nullptr &&
         hooks.publish_oled_command != nullptr &&
         hooks.read_motor_telemetry != nullptr &&
         hooks.read_pwm_servo_telemetry != nullptr &&
         hooks.read_imu_telemetry != nullptr &&
         hooks.read_battery_telemetry != nullptr &&
         hooks.pop_button_event != nullptr && hooks.read_health != nullptr &&
         hooks.read_worker_diagnostics != nullptr &&
         hooks.dispatch_motor_model != nullptr &&
         hooks.poll_motor_model != nullptr &&
         hooks.dispatch_pwm_offsets != nullptr &&
         hooks.poll_pwm_offsets != nullptr &&
         hooks.dispatch_battery_threshold != nullptr &&
         hooks.poll_battery_threshold != nullptr &&
         hooks.dispatch_bus_get_state != nullptr &&
         hooks.poll_bus_get_state != nullptr &&
         hooks.dispatch_bus_configure != nullptr &&
         hooks.poll_bus_configure != nullptr &&
         hooks.dispatch_bus_stop != nullptr && hooks.poll_bus_stop != nullptr;
}

}  // namespace mentor_pi_mcu::app::microros
