// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::OkResult;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;

constexpr std::uint32_t kSafetyPeriodMs = 20U;
constexpr std::uint32_t kMotorWaitMs = 2U;
constexpr std::uint32_t kBusWaitMs = 10U;
constexpr std::uint32_t kPeripheralWaitMs = 1U;
constexpr std::uint32_t kSafetyStartupGraceMs = 250U;

float MotorAdmissionLimit(const mentor_pi::mcu::MotorController& controller) {
  return controller.closed_loop_enabled() ? controller.maximum_accepted_rps()
                                          : controller.profile().max_rps;
}

void SafetyTaskTrampoline(void* context) {
  static_cast<ControllerRuntime*>(context)->RunSafetySupervisorTask();
}

void MotorTaskTrampoline(void* context) {
  static_cast<ControllerRuntime*>(context)->RunMotorControlTask();
}

void BusTaskTrampoline(void* context) {
  static_cast<ControllerRuntime*>(context)->RunBusServoTask();
}

void SensorTaskTrampoline(void* context) {
  static_cast<ControllerRuntime*>(context)->RunSensorTask();
}

void PeripheralTaskTrampoline(void* context) {
  static_cast<ControllerRuntime*>(context)->RunPeripheralTask();
}

}  // namespace

ControllerRuntime::ControllerRuntime(
    const mentor_pi::mcu::MotorControlConfiguration& motor_configuration)
    : register_i2c_adapter_(this),
      raw_i2c_adapter_(this),
      bus_uart_adapter_(this),
      rgb_spi_adapter_(this),
      peripheral_adapter_(this),
      imu_driver_(register_i2c_adapter_),
      bus_driver_(bus_uart_adapter_),
      rgb_driver_(rgb_spi_adapter_),
      oled_driver_(raw_i2c_adapter_),
      gpio_driver_(peripheral_adapter_),
      motor_controller_(motor_configuration) {
  motor_max_rps_bits_.store(FloatBits(MotorAdmissionLimit(motor_controller_)),
                            std::memory_order_relaxed);
}

bool ControllerRuntime::Configure(
    const PlatformHooks& hooks,
    const mentor_pi::mcu::drivers::AxisTransform& imu_transform) {
  if (!PlatformHooksAreComplete(hooks) ||
      configured_.load(std::memory_order_acquire)) {
    return false;
  }
  hooks_ = hooks;
  imu_transform_ = imu_transform;
  const std::uint8_t captured_watchdog_task =
      hooks_.captured_watchdog_task(hooks_.context);
  last_watchdog_task_.store(
      captured_watchdog_task < static_cast<std::uint8_t>(ControllerTask::kCount)
          ? captured_watchdog_task
          : 255U,
      std::memory_order_release);
  configured_.store(true, std::memory_order_release);
  return true;
}

bool ControllerRuntime::InitializeSafeBoot() {
  if (!configured_.load(std::memory_order_acquire) ||
      initialized_.load(std::memory_order_acquire)) {
    return false;
  }

  hooks_.emergency_stop_motors(hooks_.context);
  Result result = hooks_.initialize_motor_outputs(hooks_.context);
  if (!result.ok()) {
    RecordLastError(result, mentor_pi_mcu::app::microros::ErrorSource::kMotors);
    return false;
  }
  hooks_.emergency_stop_motors(hooks_.context);

  result = gpio_driver_.InitializeSafe();
  if (!result.ok()) {
    RecordPeripheralResult(6U, result,
                           mentor_pi_mcu::app::microros::ErrorSource::kBuzzer);
    return false;
  }
  result = hooks_.initialize_pwm_servos(hooks_.context);
  if (!result.ok()) {
    RecordPeripheralResult(
        4U, result, mentor_pi_mcu::app::microros::ErrorSource::kPwmServos);
    return false;
  }
  const mentor_pi::mcu::PwmFrameUpdate initial =
      pwm_controller_.PrepareFollowingFrame();
  result = hooks_.set_pwm_servo_shadow(hooks_.context,
                                       initial.output_pulse_width_us());
  if (!result.ok()) {
    RecordPeripheralResult(
        4U, result, mentor_pi_mcu::app::microros::ErrorSource::kPwmServos);
    return false;
  }
  pending_pwm_frame_ = initial;
  pwm_shadow_waiting_for_commit_ = true;
  last_pwm_frame_sequence_ = hooks_.pwm_servo_frame_sequence(hooks_.context);
  rgb_pending_ = true;
  oled_pending_ = true;
  last_oled_attempt_ms_ = hooks_.monotonic_milliseconds(hooks_.context) - 250U;
  desired_session_active_.store(false, std::memory_order_release);
  startup_motor_inhibited_.store(true, std::memory_order_release);
  motor_controller_.SetSessionActive(false);
  initialized_.store(true, std::memory_order_release);
  return true;
}

ControllerTaskEntries ControllerRuntime::BuildTaskEntries(
    ControllerTaskEntry micro_ros_entry) {
  ControllerTaskEntries entries{};
  entries[static_cast<std::size_t>(ControllerTask::kSafetySupervisor)] = {
      &SafetyTaskTrampoline, this};
  entries[static_cast<std::size_t>(ControllerTask::kMotorControl)] = {
      &MotorTaskTrampoline, this};
  entries[static_cast<std::size_t>(ControllerTask::kMicroRos)] =
      micro_ros_entry;
  entries[static_cast<std::size_t>(ControllerTask::kBusServo)] = {
      &BusTaskTrampoline, this};
  entries[static_cast<std::size_t>(ControllerTask::kSensor)] = {
      &SensorTaskTrampoline, this};
  entries[static_cast<std::size_t>(ControllerTask::kPeripheral)] = {
      &PeripheralTaskTrampoline, this};
  return entries;
}

[[noreturn]] void ControllerRuntime::RunSafetySupervisorTask() {
  for (;;) {
    RunSafetySupervisorOnce();
    hooks_.wait_for_task(hooks_.context, ControllerTask::kSafetySupervisor,
                         kSafetyPeriodMs);
  }
}

[[noreturn]] void ControllerRuntime::RunMotorControlTask() {
  for (;;) {
    hooks_.wait_for_task(hooks_.context, ControllerTask::kMotorControl,
                         kMotorWaitMs);
    RunMotorControlOnce();
  }
}

[[noreturn]] void ControllerRuntime::RunBusServoTask() {
  for (;;) {
    RunBusServoOnce();
    hooks_.wait_for_task(hooks_.context, ControllerTask::kBusServo, kBusWaitMs);
  }
}

[[noreturn]] void ControllerRuntime::RunSensorTask() {
  for (;;) {
    RunSensorOnce();
    hooks_.wait_for_task(hooks_.context, ControllerTask::kSensor,
                         NextSensorWaitMilliseconds());
  }
}

std::uint32_t ControllerRuntime::NextSensorWaitMilliseconds() const {
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);
  std::uint32_t wait_ms = kSensorMaximumWaitMs;
  const auto shorten_to_deadline =
      [now_ms, &wait_ms](bool started, std::uint32_t deadline_ms) {
        if (!started) {
          return;
        }
        const std::uint32_t remaining_ms = deadline_ms - now_ms;
        if (remaining_ms == 0U || remaining_ms >= 0x80000000U) {
          wait_ms = 1U;
          return;
        }
        wait_ms = std::min(wait_ms, remaining_ms);
      };
  shorten_to_deadline(button_sampling_started_, next_button_sample_ms_);
  shorten_to_deadline(battery_sampling_started_, next_battery_sample_ms_);
  return wait_ms;
}

[[noreturn]] void ControllerRuntime::RunPeripheralTask() {
  for (;;) {
    RunPeripheralOnce();
    hooks_.wait_for_task(hooks_.context, ControllerTask::kPeripheral,
                         kPeripheralWaitMs);
  }
}

float ControllerRuntime::MotorMaximumRps() const {
  return BitsFloat(motor_max_rps_bits_.load(std::memory_order_acquire));
}

std::uint32_t ControllerRuntime::MonotonicMilliseconds() const {
  return hooks_.monotonic_milliseconds(hooks_.context);
}

std::uint32_t ControllerRuntime::MonotonicMicroseconds() const {
  return hooks_.monotonic_microseconds(hooks_.context);
}

void ControllerRuntime::WaitForMicroRos(std::uint32_t maximum_ms) const {
  hooks_.wait_for_task(hooks_.context, ControllerTask::kMicroRos, maximum_ms);
}

void ControllerRuntime::EmergencyStopMotors() const {
  hooks_.emergency_stop_motors(hooks_.context);
}

void ControllerRuntime::RevokeMotorAuthorityLocked(bool fatal_output_fault) {
  if (fatal_output_fault) {
    fatal_output_fault_.store(true, std::memory_order_release);
  }
  desired_session_active_.store(false, std::memory_order_release);
  hooks_.emergency_stop_motors(hooks_.context);
}

void ControllerRuntime::SetSessionActive(bool active,
                                         std::uint32_t generation) {
  CriticalGuard guard(this);
  if (active) {
    const bool currently_active =
        desired_session_active_.load(std::memory_order_relaxed);
    const std::uint32_t current_generation =
        session_generation_.load(std::memory_order_relaxed);
    const bool unrecoverable_fault =
        fatal_output_fault_.load(std::memory_order_relaxed) ||
        watchdog_withheld_.load(std::memory_order_relaxed);
    if (unrecoverable_fault ||
        (!currently_active &&
         !GenerationIsAfter(generation, current_generation))) {
      // Re-activation of an equal, stale, or invalid generation is never
      // authority. Recoverable session/transport loss needs a freshly created
      // generation; controller output/watchdog faults remain inhibited until
      // safe boot after reset.
      RevokeMotorAuthorityLocked(false);
      return;
    }
    if (currently_active) {
      // An exact duplicate is idempotent. A newer or older value is ignored:
      // it must not replace a live owner without an explicit inactive
      // transition.
      return;
    }
    // Publish the owner generation before granting authority. Readers that do
    // not need the controller critical section may observe the old inactive
    // state, but can never observe ACTIVE with the preceding generation.
    session_generation_.store(generation, std::memory_order_release);
    desired_session_active_.store(true, std::memory_order_release);
    return;
  }

  // Revoke authority before touching the outputs, while excluding every
  // MotorControlTask arm/apply critical section. A stale worker therefore
  // either finishes before this stop, or observes INACTIVE after it; it cannot
  // re-arm between the state transition and the physical stop.
  desired_session_active_.store(false, std::memory_order_release);
  const std::uint32_t current_generation =
      session_generation_.load(std::memory_order_relaxed);
  if (generation == current_generation ||
      GenerationIsAfter(generation, current_generation)) {
    session_generation_.store(generation, std::memory_order_release);
  }
  hooks_.emergency_stop_motors(hooks_.context);
}

void ControllerRuntime::InvalidateSessionWork(std::uint32_t generation) {
  {
    CriticalGuard guard(this);
    RevokeMotorAuthorityLocked(false);
    bus_stop_watermark_ = last_bus_command_generation_;
  }
  Cancel(&motor_model_slot_, generation);
  Cancel(&pwm_offsets_slot_, generation);
  Cancel(&battery_threshold_slot_, generation);
  Cancel(&bus_service_slot_, generation);
}

void ControllerRuntime::RunSafetySupervisorOnce() {
  if (!initialized()) {
    return;
  }
  const std::uint32_t started_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);
  constexpr std::array<std::uint32_t, kControllerTaskCount> kMaximumAgeMs{
      0U, 30U, 100U, 100U, 150U, 150U};

  bool healthy = !fatal_output_fault_.load(std::memory_order_acquire);
  std::uint8_t stale_task = 255U;
  bool task_not_started = false;
  for (std::size_t index = 1U; index < kControllerTaskCount; ++index) {
    if (!task_seen_[index].load(std::memory_order_acquire)) {
      task_not_started = true;
      stale_task = static_cast<std::uint8_t>(index);
      break;
    }
    const std::uint32_t heartbeat =
        task_heartbeat_ms_[index].load(std::memory_order_acquire);
    if (now_ms - heartbeat > kMaximumAgeMs[index]) {
      healthy = false;
      stale_task = static_cast<std::uint8_t>(index);
      break;
    }
  }

  if (task_not_started) {
    if (!safety_startup_grace_started_) {
      safety_startup_grace_started_ = true;
      safety_startup_deadline_ms_ = now_ms + kSafetyStartupGraceMs;
    }
    const bool grace_expired =
        static_cast<std::int32_t>(now_ms - safety_startup_deadline_ms_) >= 0;
    if (!grace_expired) {
      // Highest-priority SafetySupervisorTask necessarily runs before its
      // lower-priority peers on first scheduling. Keep outputs safe while the
      // peers establish their first progress marks, but keep the IWDG alive
      // for this one bounded startup interval.
      {
        CriticalGuard guard(this);
        hooks_.emergency_stop_motors(hooks_.context);
      }
      healthy = !fatal_output_fault_.load(std::memory_order_acquire);
      stale_task = 255U;
    } else {
      healthy = false;
    }
  }

  if (!task_not_started && healthy &&
      startup_motor_inhibited_.load(std::memory_order_acquire)) {
    // This one-way startup transition deliberately leaves the first session
    // generation active. MotorControlTask may consume its retained latest
    // command only after every peer has published an on-time heartbeat.
    CriticalGuard guard(this);
    startup_motor_inhibited_.store(false, std::memory_order_release);
  }

  if (!healthy) {
    {
      CriticalGuard guard(this);
      watchdog_withheld_.store(true, std::memory_order_release);
      RevokeMotorAuthorityLocked(false);
    }
    if (stale_task != 255U) {
      if (!watchdog_task_persist_requested_) {
        hooks_.persist_watchdog_task(hooks_.context,
                                     static_cast<ControllerTask>(stale_task));
        watchdog_task_persist_requested_ = true;
      }
      RecordLastError({ResultCode::kTimeout, stale_task},
                      mentor_pi_mcu::app::microros::ErrorSource::kExecutor);
    }
  } else if (!watchdog_withheld_.load(std::memory_order_acquire)) {
    const Result refresh = hooks_.refresh_watchdog(hooks_.context);
    if (!refresh.ok()) {
      {
        CriticalGuard guard(this);
        watchdog_withheld_.store(true, std::memory_order_release);
        RevokeMotorAuthorityLocked(false);
      }
      RecordLastError(refresh,
                      mentor_pi_mcu::app::microros::ErrorSource::kExecutor);
    }
  }
  RecordTaskProgress(ControllerTask::kSafetySupervisor, started_us,
                     kSafetyPeriodMs * 1000U);
}

void ControllerRuntime::AdvanceMicroRosHeartbeat() {
  const std::size_t index = static_cast<std::size_t>(ControllerTask::kMicroRos);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);
  task_heartbeat_ms_[index].store(now_ms, std::memory_order_release);
  task_seen_[index].store(true, std::memory_order_release);
}

void ControllerRuntime::RecordTaskProgress(ControllerTask task,
                                           std::uint32_t started_us,
                                           std::uint32_t expected_period_us) {
  const std::size_t index = static_cast<std::size_t>(task);
  const std::uint32_t finished_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);
  const std::uint32_t execution_us = finished_us - started_us;
  std::uint32_t maximum =
      task_max_execution_us_[index].load(std::memory_order_relaxed);
  while (execution_us > maximum &&
         !task_max_execution_us_[index].compare_exchange_weak(
             maximum, execution_us, std::memory_order_relaxed,
             std::memory_order_relaxed)) {
  }

  if (task_seen_[index].load(std::memory_order_acquire)) {
    const std::uint32_t previous_ms =
        task_heartbeat_ms_[index].load(std::memory_order_relaxed);
    const std::uint32_t elapsed_us = (now_ms - previous_ms) * 1000U;
    if (expected_period_us != 0U && elapsed_us > expected_period_us) {
      std::uint32_t current =
          task_missed_releases_[index].load(std::memory_order_relaxed);
      while (current != std::numeric_limits<std::uint32_t>::max() &&
             !task_missed_releases_[index].compare_exchange_weak(
                 current, current + 1U, std::memory_order_relaxed,
                 std::memory_order_relaxed)) {
      }
    }
  }
  task_heartbeat_ms_[index].store(now_ms, std::memory_order_release);
  task_seen_[index].store(true, std::memory_order_release);
}

void ControllerRuntime::RecordPeripheralResult(
    std::size_t peripheral_index, Result result,
    mentor_pi_mcu::app::microros::ErrorSource source) {
  if (result.ok() || result.code == ResultCode::kBusy) {
    return;
  }
  if (peripheral_index < peripheral_errors_.size()) {
    peripheral_errors_[peripheral_index].Increment();
    if (result.code == ResultCode::kTimeout) {
      peripheral_timeouts_[peripheral_index].Increment();
    }
  }
  RecordLastError(result, source);
}

void ControllerRuntime::RecordLastError(
    Result result, mentor_pi_mcu::app::microros::ErrorSource source) {
  if (result.ok() || result.code == ResultCode::kBusy) {
    return;
  }
  const std::uint32_t now_ms =
      configured_.load(std::memory_order_acquire)
          ? hooks_.monotonic_milliseconds(hooks_.context)
          : 0U;
  last_error_uptime_ms_.store(now_ms, std::memory_order_release);
  last_error_detail_.store(result.detail, std::memory_order_release);
  last_error_code_.store(static_cast<std::uint8_t>(result.code),
                         std::memory_order_release);
  last_error_source_.store(static_cast<std::uint8_t>(source),
                           std::memory_order_release);
}

bool ControllerRuntime::GenerationIsAfter(std::uint32_t candidate,
                                          std::uint32_t watermark) {
  if (candidate == 0U) {
    return false;
  }
  if (watermark == 0U) {
    return true;
  }
  return static_cast<std::int32_t>(candidate - watermark) > 0;
}

std::uint32_t ControllerRuntime::FloatBits(float value) {
  std::uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "float must be 32 bits");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float ControllerRuntime::BitsFloat(std::uint32_t value) {
  float result = 0.0F;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

mentor_pi::mcu::drivers::IoStatus ControllerRuntime::RegisterI2cAdapter::Read(
    std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
    std::size_t size, std::uint32_t deadline_us) {
  return runtime_->hooks_.register_i2c_read(runtime_->hooks_.context, address,
                                            reg, data, size, deadline_us);
}

mentor_pi::mcu::drivers::IoStatus ControllerRuntime::RegisterI2cAdapter::Write(
    std::uint8_t address, std::uint8_t reg, const std::uint8_t* data,
    std::size_t size, std::uint32_t deadline_us) {
  return runtime_->hooks_.register_i2c_write(runtime_->hooks_.context, address,
                                             reg, data, size, deadline_us);
}

mentor_pi::mcu::drivers::IoStatus ControllerRuntime::RawI2cAdapter::Write(
    std::uint8_t address, const std::uint8_t* data, std::size_t size,
    std::uint32_t deadline_ms) {
  return runtime_->hooks_.raw_i2c_write(runtime_->hooks_.context, address, data,
                                        size, deadline_ms);
}

mentor_pi::mcu::drivers::IoStatus
ControllerRuntime::BusUartAdapter::BeginExchange(const std::uint8_t* tx,
                                                 std::size_t tx_size,
                                                 std::size_t max_reply_size,
                                                 std::uint32_t deadline_ms) {
  return runtime_->hooks_.bus_uart_begin_exchange(
      runtime_->hooks_.context, tx, tx_size, max_reply_size, deadline_ms);
}

mentor_pi::mcu::drivers::IoStatus
ControllerRuntime::BusUartAdapter::PollExchange(std::uint32_t now_ms,
                                                std::uint8_t* reply,
                                                std::size_t capacity,
                                                std::size_t* reply_size) {
  return runtime_->hooks_.bus_uart_poll_exchange(
      runtime_->hooks_.context, now_ms, reply, capacity, reply_size);
}

void ControllerRuntime::BusUartAdapter::Cancel() {
  runtime_->hooks_.bus_uart_cancel(runtime_->hooks_.context);
}

mentor_pi::mcu::drivers::IoStatus
ControllerRuntime::RgbSpiAdapter::BeginTransmit(const std::uint8_t* data,
                                                std::size_t size,
                                                std::uint32_t deadline_us) {
  return runtime_->hooks_.rgb_spi_begin_transmit(runtime_->hooks_.context, data,
                                                 size, deadline_us);
}

mentor_pi::mcu::drivers::IoStatus
ControllerRuntime::RgbSpiAdapter::PollTransmit(std::uint32_t now_us) {
  return runtime_->hooks_.rgb_spi_poll_transmit(runtime_->hooks_.context,
                                                now_us);
}

void ControllerRuntime::RgbSpiAdapter::Cancel() {
  runtime_->hooks_.rgb_spi_cancel(runtime_->hooks_.context);
}

bool ControllerRuntime::PeripheralAdapter::ReadButtonPin(
    std::size_t button) const {
  return !runtime_->hooks_.read_button_pressed(runtime_->hooks_.context,
                                               button);
}

void ControllerRuntime::PeripheralAdapter::WriteLedPin(std::size_t led,
                                                       bool high) {
  constexpr std::array<bool, mentor_pi::mcu::kLedCount> kLedActiveHigh{
      false, false, true};
  if (led >= kLedActiveHigh.size()) {
    return;
  }
  // GpioPeripheralDriver's boundary is a raw electrical level, while the
  // target PlatformHooks boundary is semantic "on" so it can bind directly
  // to platform::SetLed. Convert back here and keep polarity in one owner.
  const bool on = high == kLedActiveHigh[led];
  runtime_->hooks_.set_led(runtime_->hooks_.context, led, on);
}

Result ControllerRuntime::PeripheralAdapter::SetBuzzerTone(
    std::uint16_t frequency_hz, bool enabled) {
  return runtime_->hooks_.set_buzzer(runtime_->hooks_.context, frequency_hz,
                                     enabled);
}

ControllerRuntime& ControllerInstance(
    const mentor_pi::mcu::MotorControlConfiguration& motor_configuration) {
  static ControllerRuntime runtime(motor_configuration);
  return runtime;
}

}  // namespace mentor_pi_mcu::app::controller
