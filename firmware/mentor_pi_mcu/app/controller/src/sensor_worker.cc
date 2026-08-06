// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"
#include "mentor_pi_mcu/app/controller/imu_characterization.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi_mcu::app::microros::ErrorSource;

constexpr std::uint32_t kSensorMaximumPeriodUs = 150000U;
constexpr std::uint32_t kImuDeadlineUs = 10000U;
constexpr std::uint32_t kImuRetryMs = 1000U;

bool DeadlineReached(std::uint32_t now_ms, std::uint32_t deadline_ms) {
  return static_cast<std::int32_t>(now_ms - deadline_ms) >= 0;
}

void AdvancePeriodicDeadline(std::uint32_t now_ms, std::uint32_t period_ms,
                             std::uint32_t* deadline_ms) {
  const std::uint32_t lateness_ms = now_ms - *deadline_ms;
  const std::uint32_t elapsed_periods = (lateness_ms / period_ms) + 1U;
  *deadline_ms += elapsed_periods * period_ms;
}

}  // namespace

void ControllerRuntime::RunSensorOnce() {
  if (!initialized()) {
    return;
  }
  const std::uint32_t started_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);

  ProcessBatteryThresholdService();
  SampleImu(now_ms, started_us);
  SampleButtons(now_ms);
  SampleBattery(now_ms);
  RecordTaskProgress(ControllerTask::kSensor, started_us,
                     kSensorMaximumPeriodUs);
}

void ControllerRuntime::SampleImu(std::uint32_t now_ms, std::uint32_t now_us) {
  if (!imu_initialized_) {
    if (static_cast<std::int32_t>(now_ms - next_imu_initialize_ms_) < 0) {
      return;
    }
    const Result initialized = imu_driver_.Initialize(now_us + kImuDeadlineUs);
    if (!initialized.ok()) {
      imu_healthy_.store(false, std::memory_order_release);
      RecordPeripheralResult(1U, initialized, ErrorSource::kImu);
      next_imu_initialize_ms_ = now_ms + kImuRetryMs;
      imu_state_.valid = false;
      if (!imu_transform_.verified) {
        UpdateImuCharacterizationSnapshot(now_ms, imu_driver_.address(),
                                          imu_driver_.revision(), initialized,
                                          nullptr);
      }
      static_cast<void>(imu_telemetry_.Publish(imu_state_));
      return;
    }
    imu_initialized_ = true;
  }

  if (!imu_transform_.verified) {
    imu_healthy_.store(false, std::memory_order_release);
    imu_state_.valid = false;
    mentor_pi::mcu::drivers::ImuSample raw_sample{};
    const Result raw_result =
        imu_driver_.ReadRawSample(now_us + kImuDeadlineUs, &raw_sample);
    UpdateImuCharacterizationSnapshot(now_ms, imu_driver_.address(),
                                      imu_driver_.revision(), raw_result,
                                      raw_result.ok() ? &raw_sample : nullptr);
    if (!raw_result.ok()) {
      // Data-not-ready is an expected sampling condition. A real I/O failure
      // must remain visible in diagnostics, and the intentional UNSUPPORTED
      // transform marker is emitted only after raw acquisition has succeeded.
      // That makes the first-board characterization condition positive
      // evidence of a working QMI8658 read path rather than merely a working
      // identity/configuration transaction.
      RecordPeripheralResult(1U, raw_result, ErrorSource::kImu);
      return;
    }
    if (!imu_transform_error_recorded_) {
      const Result result{ResultCode::kUnsupported, 1U};
      RecordPeripheralResult(1U, result, ErrorSource::kImu);
      imu_transform_error_recorded_ = true;
      static_cast<void>(imu_telemetry_.Publish(imu_state_));
    }
    return;
  }

  mentor_pi::mcu::drivers::ImuSample sample{};
  const Result result =
      imu_driver_.ReadSample(now_us + kImuDeadlineUs, imu_transform_, &sample);
  if (result.code == ResultCode::kBusy) {
    return;
  }
  if (!result.ok()) {
    imu_healthy_.store(false, std::memory_order_release);
    imu_state_.valid = false;
    RecordPeripheralResult(1U, result, ErrorSource::kImu);
    static_cast<void>(imu_telemetry_.Publish(imu_state_));
    return;
  }
  imu_state_.timestamp_ms = now_ms;
  imu_state_.angular_velocity_rad_s = sample.angular_velocity_rps;
  imu_state_.linear_acceleration_m_s2 = sample.acceleration_mps2;
  imu_state_.valid = true;
  imu_healthy_.store(true, std::memory_order_release);
  static_cast<void>(imu_telemetry_.Publish(imu_state_));
}

void ControllerRuntime::SampleButtons(std::uint32_t now_ms) {
  if (!button_sampling_started_) {
    button_sampling_started_ = true;
    next_button_sample_ms_ = now_ms + mentor_pi::mcu::kButtonScanPeriodMs;
  } else {
    if (!DeadlineReached(now_ms, next_button_sample_ms_)) {
      return;
    }
    AdvancePeriodicDeadline(now_ms, mentor_pi::mcu::kButtonScanPeriodMs,
                            &next_button_sample_ms_);
  }
  std::array<bool, mentor_pi::mcu::kButtonCount> pressed{};
  for (std::size_t button = 0; button < pressed.size(); ++button) {
    pressed[button] = gpio_driver_.ButtonPressed(button);
  }
  button_controller_.Sample(pressed, now_ms);
}

void ControllerRuntime::SampleBattery(std::uint32_t now_ms) {
  if (!battery_sampling_started_) {
    battery_sampling_started_ = true;
    next_battery_sample_ms_ = now_ms + kBatterySamplePeriodMs;
  } else {
    if (!DeadlineReached(now_ms, next_battery_sample_ms_)) {
      return;
    }
    AdvancePeriodicDeadline(now_ms, kBatterySamplePeriodMs,
                            &next_battery_sample_ms_);
  }

  const BatterySample sample =
      hooks_.take_battery_sample(hooks_.context, now_ms);
  if (!sample.available &&
      (sample.result.ok() || sample.result.code == ResultCode::kBusy)) {
    return;
  }
  if (!sample.result.ok()) {
    RecordPeripheralResult(3U, sample.result, ErrorSource::kBattery);
  }
  const bool valid =
      sample.available && sample.result.ok() && sample.reading.valid;
  const mentor_pi::mcu::BatteryUpdate update = battery_monitor_.AddSample(
      valid ? sample.reading.voltage_mv : 0U, valid, now_ms);
  battery_state_.timestamp_ms = now_ms;
  battery_state_.voltage_mv = update.state.voltage_mv;
  battery_state_.low_threshold_mv = update.state.low_threshold_mv;
  battery_state_.valid = update.state.valid;
  battery_state_.below_threshold = update.state.below_threshold;
  low_battery_.store(update.state.below_threshold, std::memory_order_release);
  static_cast<void>(battery_telemetry_.Publish(battery_state_));
  static_cast<void>(battery_display_mailbox_.Publish(
      {update.state.voltage_mv, update.state.valid}));
  if (update.request_alarm_pattern) {
    static_cast<void>(battery_alarm_mailbox_.Publish({now_ms}));
  }
}

void ControllerRuntime::ProcessBatteryThresholdService() {
  mentor_pi_mcu::app::microros::ServiceToken token{};
  BatteryThresholdRequest request{};
  if (!Take(&battery_threshold_slot_, &token, &request)) {
    return;
  }
  mentor_pi_mcu::app::microros::BatteryThresholdReply reply{};
  {
    CriticalGuard guard(this);
    if (!TokenIsCurrent(token)) {
      battery_threshold_slot_.state.store(SlotState::kCanceled,
                                          std::memory_order_release);
    } else {
      const mentor_pi::mcu::BatteryThresholdUpdate update =
          battery_monitor_.SetLowThreshold(request.threshold_mv);
      reply.result = update.result;
      reply.active_threshold_mv = update.active_threshold_mv;
    }
  }
  Complete(&battery_threshold_slot_, token, reply);
}

}  // namespace mentor_pi_mcu::app::controller
