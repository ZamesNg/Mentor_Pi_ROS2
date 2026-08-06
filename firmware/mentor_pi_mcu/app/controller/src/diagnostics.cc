// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstddef>
#include <cstdint>
#include <limits>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

namespace mentor_pi_mcu::app::controller {

void ControllerRuntime::ReadHealth(
    mentor_pi_mcu::app::microros::HealthSnapshot* output) const {
  if (output == nullptr) {
    return;
  }
  output->motor_watchdog_active =
      motor_watchdog_mask_.load(std::memory_order_acquire) != 0U;
  output->low_battery = low_battery_.load(std::memory_order_acquire);
  output->imu_healthy = imu_healthy_.load(std::memory_order_acquire);
  output->bus_servo_busy = bus_busy_.load(std::memory_order_acquire);
  output->output_processing_fault =
      fatal_output_fault_.load(std::memory_order_acquire) ||
      watchdog_withheld_.load(std::memory_order_acquire);
  bool peripheral_fault = false;
  for (const auto& counter : peripheral_errors_) {
    peripheral_fault = peripheral_fault || counter.value() != 0U;
  }
  output->nonfatal_degraded = !output->imu_healthy || peripheral_fault;
}

void ControllerRuntime::ReadWorkerDiagnostics(
    mentor_pi_mcu::app::microros::WorkerDiagnostics* output) const {
  if (output == nullptr) {
    return;
  }
  *output = {};
  output->button_event_drops = button_controller_.dropped_event_count();
  for (std::size_t motor = 0; motor < mentor_pi::mcu::kMotorCount; ++motor) {
    output->motor_lease_expiries[motor] =
        motor_controller_.lease_expiry_count(motor);
    output->motor_command_rejections[motor] =
        motor_controller_.command_rejection_count(motor);
  }
  output->motor_watchdog_trips = motor_watchdog_trips_.value();
  output->motor_command_consumptions = motor_command_consumptions_.value();
  output->motor_command_age_over_20_ms = motor_command_age_over_20_ms_.value();
  output->motor_command_max_age_us =
      motor_command_max_age_us_.load(std::memory_order_relaxed);
  for (std::size_t peripheral = 0; peripheral < peripheral_errors_.size();
       ++peripheral) {
    output->peripheral_errors[peripheral] =
        peripheral_errors_[peripheral].value();
    output->peripheral_timeouts[peripheral] =
        peripheral_timeouts_[peripheral].value();
  }

  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);
  for (std::size_t task = 0; task < kControllerTaskCount; ++task) {
    output->task_missed_releases[task] =
        task_missed_releases_[task].load(std::memory_order_acquire);
    output->task_max_execution_us[task] =
        task_max_execution_us_[task].load(std::memory_order_acquire);
    output->task_stack_high_water_bytes[task] =
        hooks_.task_stack_high_water_bytes(hooks_.context,
                                           static_cast<ControllerTask>(task));
    output->task_heartbeat_age_ms[task] =
        task_seen_[task].load(std::memory_order_acquire)
            ? now_ms - task_heartbeat_ms_[task].load(std::memory_order_acquire)
            : std::numeric_limits<std::uint32_t>::max();
  }
  hooks_.read_memory_metrics(hooks_.context, &output->free_ram_bytes,
                             &output->minimum_free_ram_bytes);
  output->flash_used_bytes = hooks_.flash_used_bytes(hooks_.context);
  output->flash_total_bytes = hooks_.flash_total_bytes(hooks_.context);
  output->last_reset_reason = hooks_.last_reset_reason(hooks_.context);
  output->last_watchdog_task =
      last_watchdog_task_.load(std::memory_order_acquire);
  output->last_error_uptime_ms =
      last_error_uptime_ms_.load(std::memory_order_acquire);
  output->last_error_detail =
      last_error_detail_.load(std::memory_order_acquire);
  output->last_error_code = last_error_code_.load(std::memory_order_acquire);
  output->last_error_source =
      last_error_source_.load(std::memory_order_acquire);
}

}  // namespace mentor_pi_mcu::app::controller
