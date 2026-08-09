// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::MotorCommand;
using mentor_pi::mcu::MotorCommandSnapshot;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi_mcu::app::microros::ErrorSource;

constexpr std::uint32_t kMaximumMotorReleaseIntervalUs = 2000U;
constexpr std::uint32_t kMotorCommandAgeThresholdUs = 20000U;

float MotorAdmissionLimit(const mentor_pi::mcu::MotorController& controller) {
  return controller.configuration_valid() ? controller.maximum_accepted_rps()
                                          : controller.profile().max_rps;
}

}  // namespace

void ControllerRuntime::DeactivateMotorOwnerLocked() {
  const std::uint8_t previous_mask = motor_controller_.watchdog_stop_mask();
  motor_controller_.SetSessionActive(false);
  motor_owner_session_generation_ = 0U;
  const std::uint8_t current_mask = motor_controller_.watchdog_stop_mask();
  motor_watchdog_mask_.store(current_mask, std::memory_order_release);
  IncrementMotorWatchdogTrips(previous_mask, current_mask);
}

void ControllerRuntime::RunMotorControlOnce() {
  if (!initialized()) {
    return;
  }
  const std::uint32_t started_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_us = started_us;
  const bool desired_active =
      desired_session_active_.load(std::memory_order_acquire);
  const std::uint32_t desired_generation =
      session_generation_.load(std::memory_order_acquire);
  const bool generation_changed =
      desired_active && desired_generation != motor_owner_session_generation_;
  if (motor_controller_.session_active() != desired_active ||
      generation_changed) {
    const std::uint8_t previous_mask = motor_controller_.watchdog_stop_mask();
    if (generation_changed && motor_controller_.session_active()) {
      motor_controller_.SetSessionActive(false);
      hooks_.emergency_stop_motors(hooks_.context);
    }
    motor_controller_.SetSessionActive(desired_active);
    motor_owner_session_generation_ = desired_active ? desired_generation : 0U;
    if (!desired_active) {
      for (std::size_t motor = 0; motor < mentor_pi::mcu::kMotorCount;
           ++motor) {
        hooks_.disarm_motor(hooks_.context, motor);
      }
    }
    IncrementMotorWatchdogTrips(previous_mask,
                                motor_controller_.watchdog_stop_mask());
  }

  ConsumeMotorCommand(now_us);
  ProcessMotorModelService();
  ProcessMotorPidService();

  const std::uint8_t previous_watchdog_mask =
      motor_controller_.watchdog_stop_mask();
  motor_controller_.EvaluateLeases(now_us);
  const std::uint8_t watchdog_mask = motor_controller_.watchdog_stop_mask();
  motor_watchdog_mask_.store(watchdog_mask, std::memory_order_release);
  IncrementMotorWatchdogTrips(previous_watchdog_mask, watchdog_mask);
  for (std::size_t motor = 0; motor < mentor_pi::mcu::kMotorCount; ++motor) {
    if (!motor_controller_.channels()[motor].armed) {
      hooks_.disarm_motor(hooks_.context, motor);
    }
  }

  ++motor_release_count_;
  if (motor_release_count_ >= 10U) {
    motor_release_count_ = 0U;
    std::array<std::uint32_t, mentor_pi::mcu::kMotorCount> counters{};
    if (!hooks_.read_encoder_counters(hooks_.context, &counters)) {
      const Result result{ResultCode::kIoError, 1U};
      {
        CriticalGuard guard(this);
        RevokeMotorAuthorityLocked(true);
        DeactivateMotorOwnerLocked();
      }
      RecordLastError(result, ErrorSource::kMotors);
    } else {
      const std::uint32_t control_period_us =
          motor_control_sample_initialized_
              ? now_us - last_motor_control_sample_us_
              : mentor_pi::mcu::kMotorControlPeriodUs;
      last_motor_control_sample_us_ = now_us;
      motor_control_sample_initialized_ = true;
      const auto output =
          motor_controller_.ControlStep(counters, control_period_us);
      const std::uint8_t post_control_watchdog_mask =
          motor_controller_.watchdog_stop_mask();
      motor_watchdog_mask_.store(post_control_watchdog_mask,
                                 std::memory_order_release);
      IncrementMotorWatchdogTrips(watchdog_mask, post_control_watchdog_mask);
      Result result{};
      bool duty_attempted = false;
      {
        CriticalGuard guard(this);
        const bool authority_current =
            !startup_motor_inhibited_.load(std::memory_order_relaxed) &&
            desired_session_active_.load(std::memory_order_relaxed) &&
            session_generation_.load(std::memory_order_relaxed) ==
                motor_owner_session_generation_;
        if (authority_current) {
          duty_attempted = true;
          result = hooks_.apply_motor_duty(hooks_.context, output);
          if (!result.ok()) {
            RevokeMotorAuthorityLocked(result.code != ResultCode::kBusy);
            DeactivateMotorOwnerLocked();
          }
        } else {
          DeactivateMotorOwnerLocked();
          hooks_.emergency_stop_motors(hooks_.context);
          result = {ResultCode::kBusy, 0U};
        }
      }
      if (duty_attempted && !result.ok()) {
        RecordLastError(result, ErrorSource::kMotors);
      }
      PublishMotorSnapshot(hooks_.monotonic_milliseconds(hooks_.context));
    }
  }

  RecordTaskProgress(ControllerTask::kMotorControl, started_us,
                     kMaximumMotorReleaseIntervalUs);
}

void ControllerRuntime::ConsumeMotorCommand(std::uint32_t now_us) {
  MotorCommandSnapshot snapshot{};
  MailboxTag tag{};
  bool available = false;
  {
    CriticalGuard guard(this);
    if (startup_motor_inhibited_.load(std::memory_order_relaxed)) {
      return;
    }
    available = motor_mailbox_.ConsumeLatest(&snapshot);
    tag = motor_mailbox_tag_;
  }
  if (!available) {
    return;
  }
  {
    // Keep the final generation check, controller mutation, and per-channel
    // hardware authority change indivisible with respect to MicroRosTask's
    // teardown callback. All operations in this section are register writes
    // or fixed-size copies; none wait for hardware.
    CriticalGuard guard(this);
    const std::uint32_t current_session =
        session_generation_.load(std::memory_order_relaxed);
    if (startup_motor_inhibited_.load(std::memory_order_relaxed) ||
        !desired_session_active_.load(std::memory_order_relaxed) ||
        tag.session_generation != current_session ||
        tag.command_generation != snapshot.generation) {
      return;
    }
    if (motor_consumed_session_generation_ != tag.session_generation) {
      consumed_motor_field_generation_.fill(0U);
      motor_consumed_session_generation_ = tag.session_generation;
    }

    bool consumed_accepted_field = false;
    std::uint32_t maximum_age_us = 0U;
    for (std::size_t motor = 0; motor < mentor_pi::mcu::kMotorCount; ++motor) {
      if (snapshot.field_generation[motor] == 0U ||
          snapshot.field_generation[motor] ==
              consumed_motor_field_generation_[motor]) {
        continue;
      }
      MotorCommand command{};
      command.update_mask = static_cast<std::uint8_t>(1U << motor);
      command.target_rps[motor] = snapshot.target_rps[motor];
      const Result accepted = motor_controller_.AcceptCommand(
          command, snapshot.accepted_at_us[motor]);
      consumed_motor_field_generation_[motor] =
          snapshot.field_generation[motor];
      if (!accepted.ok()) {
        RecordLastError(accepted, ErrorSource::kMotors);
        continue;
      }
      const std::uint32_t age_us = now_us - snapshot.accepted_at_us[motor];
      maximum_age_us = std::max(maximum_age_us, age_us);
      consumed_accepted_field = true;
      if (snapshot.target_rps[motor] == 0.0F) {
        hooks_.disarm_motor(hooks_.context, motor);
        continue;
      }
      const Result armed = hooks_.arm_motor(hooks_.context, motor);
      if (!armed.ok()) {
        // BUSY is the recoverable target transport-inhibit response. Other
        // failures are controller output faults and remain inhibited until
        // reset. Both revoke the current session before this critical section
        // can release to another motor iteration.
        RevokeMotorAuthorityLocked(armed.code != ResultCode::kBusy);
        DeactivateMotorOwnerLocked();
        RecordLastError(armed, ErrorSource::kMotors);
        break;
      }
    }
    if (consumed_accepted_field) {
      motor_command_consumptions_.Increment();
      if (maximum_age_us > kMotorCommandAgeThresholdUs) {
        motor_command_age_over_20_ms_.Increment();
      }
      std::uint32_t previous_maximum =
          motor_command_max_age_us_.load(std::memory_order_relaxed);
      while (maximum_age_us > previous_maximum &&
             !motor_command_max_age_us_.compare_exchange_weak(
                 previous_maximum, maximum_age_us, std::memory_order_relaxed,
                 std::memory_order_relaxed)) {
      }
    }
  }
}

void ControllerRuntime::ProcessMotorModelService() {
  mentor_pi_mcu::app::microros::ServiceToken token{};
  MotorModelRequest request{};
  if (!Take(&motor_model_slot_, &token, &request)) {
    return;
  }

  mentor_pi_mcu::app::microros::MotorModelReply reply{};
  {
    // Model mutation is constant-time. The critical section prevents a
    // teardown from invalidating the generation between the final check and
    // the owner-side state change.
    CriticalGuard guard(this);
    if (!TokenIsCurrent(token)) {
      motor_model_slot_.state.store(SlotState::kCanceled,
                                    std::memory_order_release);
    } else {
      const mentor_pi::mcu::MotorModelChange change =
          motor_controller_.SetModel(request.model);
      reply.result = change.result;
      reply.active_model = change.active_profile.model;
      reply.ticks_per_revolution = change.active_profile.ticks_per_revolution;
      reply.max_rps = change.active_profile.max_rps;
      if (reply.result.ok()) {
        motor_max_rps_bits_.store(
            FloatBits(MotorAdmissionLimit(motor_controller_)),
            std::memory_order_release);
      }
    }
  }
  Complete(&motor_model_slot_, token, reply);
}

void ControllerRuntime::ProcessMotorPidService() {
  mentor_pi_mcu::app::microros::ServiceToken token{};
  mentor_pi::mcu::SetMotorPidCommand request{};
  if (!Take(&motor_pid_slot_, &token, &request)) {
    return;
  }

  mentor_pi_mcu::app::microros::MotorPidReply reply{};
  {
    // PID validation and mutation are fixed-size and execute under the same
    // owner-side critical section as timeout cancellation, model changes, and
    // motor commands. Completion is committed before releasing the section so
    // timeout cancellation and gain application have one deterministic winner.
    CriticalGuard guard(this);
    if (motor_pid_slot_.state.load(std::memory_order_acquire) ==
            SlotState::kCanceled ||
        !TokenIsCurrent(token)) {
      motor_pid_slot_.state.store(SlotState::kCanceled,
                                  std::memory_order_release);
    } else {
      const mentor_pi::mcu::MotorPidUpdate update =
          motor_controller_.SetPid(request);
      reply.result = update.result;
      reply.applied_mask = update.applied_mask;
    }
    Complete(&motor_pid_slot_, token, reply);
  }
}

void ControllerRuntime::PublishMotorSnapshot(std::uint32_t now_ms) {
  mentor_pi_mcu::app::microros::MotorTelemetry telemetry{};
  telemetry.timestamp_ms = now_ms;
  telemetry.motor_model = motor_controller_.profile().model;
  telemetry.watchdog_stop_mask = motor_controller_.watchdog_stop_mask();
  for (std::size_t motor = 0; motor < mentor_pi::mcu::kMotorCount; ++motor) {
    const auto& channel = motor_controller_.channels()[motor];
    telemetry.target_rps[motor] = channel.target_rps;
    telemetry.measured_rps[motor] = channel.measured_rps;
    telemetry.encoder_count[motor] = channel.encoder_count;
    last_lease_expiry_count_[motor] =
        motor_controller_.lease_expiry_count(motor);
  }
  static_cast<void>(motor_telemetry_.Publish(telemetry));
}

void ControllerRuntime::IncrementMotorWatchdogTrips(std::uint8_t previous_mask,
                                                    std::uint8_t current_mask) {
  const std::uint8_t new_bits =
      static_cast<std::uint8_t>(current_mask & ~previous_mask);
  if (new_bits != 0U) {
    motor_watchdog_trips_.Increment();
  }
}

}  // namespace mentor_pi_mcu::app::controller
