// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi_mcu::app::microros::ErrorSource;

constexpr std::uint32_t kPeripheralMaximumPeriodUs = 150000U;
constexpr std::uint32_t kRgbDeadlineUs = 10000U;
// A complete 512-byte framebuffer takes about 50 ms on the retained 100 kHz
// bus. The target still caps every individual HAL transfer at 10 ms; this is
// the end-to-end deadline for the bounded multi-chunk flush.
constexpr std::uint32_t kOledDeadlineMs = 100U;
constexpr std::uint32_t kOledRetryMs = 250U;

}  // namespace

void ControllerRuntime::RunPeripheralOnce() {
  if (!initialized()) {
    return;
  }
  const std::uint32_t started_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);

  ProcessPwmFrame(now_ms);
  ProcessPwmCommands(now_ms);
  ProcessPwmOffsetService(now_ms);
  PreparePendingPwmFrame(now_ms);
  ProcessDiscreteOutputs(now_ms);
  ProcessRgb(now_ms, started_us);
  ProcessOled(now_ms);
  RecordTaskProgress(ControllerTask::kPeripheral, started_us,
                     kPeripheralMaximumPeriodUs);
}

void ControllerRuntime::SynchronizePwmSession(std::uint32_t now_ms) {
  // Caller holds the target critical section.  The frame ISR therefore cannot
  // swap a shadow while the committed snapshot is reconciled and replaced.
  const bool session_active =
      desired_session_active_.load(std::memory_order_relaxed);
  const std::uint32_t session_generation =
      session_generation_.load(std::memory_order_relaxed);
  if (pwm_owner_session_generation_ == 0U) {
    if (session_active) {
      pwm_owner_session_generation_ = session_generation;
    }
    return;
  }
  if (session_active && pwm_owner_session_generation_ == session_generation) {
    return;
  }

  const std::uint32_t sequence =
      hooks_.pwm_servo_frame_sequence(hooks_.context);
  if (sequence != last_pwm_frame_sequence_) {
    last_pwm_frame_sequence_ = sequence;
    if (pwm_shadow_waiting_for_commit_) {
      pwm_controller_.CommitFrame(pending_pwm_frame_);
      active_pwm_output_us_ = pwm_controller_.state().output_pulse_width_us;
      active_pwm_offset_us_ = pwm_controller_.state().offset_us;
    }
  }

  pwm_controller_.HoldCurrentOutputAndCancelPending(active_pwm_output_us_,
                                                    active_pwm_offset_us_);
  const mentor_pi::mcu::PwmFrameUpdate held_frame =
      pwm_controller_.PrepareFollowingFrame();
  const Result submitted = hooks_.set_pwm_servo_shadow(
      hooks_.context, held_frame.output_pulse_width_us());
  RecordPeripheralResult(4U, submitted, ErrorSource::kPwmServos);

  mentor_pi_mcu::app::microros::PwmServoTelemetry telemetry{};
  telemetry.timestamp_ms = now_ms;
  telemetry.target_pulse_width_us =
      pwm_controller_.state().target_pulse_width_us;
  telemetry.output_pulse_width_us = active_pwm_output_us_;
  telemetry.offset_us = active_pwm_offset_us_;
  telemetry.moving_mask = 0U;
  static_cast<void>(pwm_telemetry_.Publish(telemetry));
  if (submitted.ok()) {
    pending_pwm_frame_ = held_frame;
    pwm_shadow_waiting_for_commit_ = true;
  } else {
    pwm_shadow_waiting_for_commit_ = false;
  }

  if (pwm_offset_waiting_for_commit_) {
    // InvalidateSessionWork has already canceled the slot. Complete performs
    // the owner-side transition back to idle without exposing an old reply.
    Complete(&pwm_offsets_slot_, pending_pwm_offset_token_,
             pending_pwm_offset_reply_);
    pwm_offset_waiting_for_commit_ = false;
    pending_pwm_offset_token_ = {};
    pending_pwm_offset_reply_ = {};
  }
  pwm_owner_session_generation_ = session_active ? session_generation : 0U;
}

void ControllerRuntime::ProcessPwmCommands(std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  mentor_pi::mcu::PwmCommandSnapshot snapshot{};
  MailboxTag tag{};
  bool available = false;
  {
    CriticalGuard guard(this);
    available = pwm_mailbox_.ConsumeLatest(&snapshot);
    tag = pwm_mailbox_tag_;
  }
  if (!available) {
    return;
  }
  {
    CriticalGuard guard(this);
    const bool current =
        desired_session_active_.load(std::memory_order_relaxed) &&
        tag.session_generation ==
            session_generation_.load(std::memory_order_relaxed) &&
        tag.command_generation == snapshot.generation;
    if (!current) {
      return;
    }
    if (pwm_consumed_session_generation_ != tag.session_generation) {
      consumed_pwm_field_generation_.fill(0U);
      pwm_consumed_session_generation_ = tag.session_generation;
    }
    for (std::size_t servo = 0; servo < mentor_pi::mcu::kPwmServoCount;
         ++servo) {
      if (snapshot.field_generation[servo] == 0U ||
          snapshot.field_generation[servo] ==
              consumed_pwm_field_generation_[servo]) {
        continue;
      }
      consumed_pwm_field_generation_[servo] = snapshot.field_generation[servo];
      mentor_pi::mcu::PwmServoCommand command{};
      command.update_mask = static_cast<std::uint8_t>(1U << servo);
      command.duration_ms = snapshot.duration_ms[servo];
      command.pulse_width_us[servo] = snapshot.pulse_width_us[servo];
      const Result result = pwm_controller_.AcceptCommand(command);
      RecordPeripheralResult(4U, result, ErrorSource::kPwmServos);
    }
  }
}

void ControllerRuntime::ProcessPwmOffsetService(std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  if (pwm_offset_waiting_for_commit_) {
    return;
  }
  mentor_pi_mcu::app::microros::ServiceToken token{};
  mentor_pi::mcu::PwmServoOffsetCommand command{};
  if (!Take(&pwm_offsets_slot_, &token, &command)) {
    return;
  }
  mentor_pi_mcu::app::microros::PwmOffsetsReply reply{};
  bool wait_for_commit = false;
  {
    CriticalGuard guard(this);
    if (!TokenIsCurrent(token)) {
      pwm_offsets_slot_.state.store(SlotState::kCanceled,
                                    std::memory_order_release);
    } else {
      reply.result = pwm_controller_.StageOffsets(command);
      wait_for_commit =
          reply.result.ok() && pwm_controller_.offset_commit_pending();
      if (wait_for_commit) {
        pending_pwm_offset_token_ = token;
        pending_pwm_offset_reply_.result = reply.result;
        pending_pwm_offset_reply_.applied_mask = command.update_mask;
        pwm_offset_waiting_for_commit_ = true;
      }
    }
  }
  if (!wait_for_commit) {
    reply.applied_mask = reply.result.ok() ? command.update_mask : 0U;
    Complete(&pwm_offsets_slot_, token, reply);
  }
}

void ControllerRuntime::ProcessPwmFrame(std::uint32_t now_ms) {
  CriticalGuard guard(this);
  ProcessPwmFrameLocked(now_ms);
}

void ControllerRuntime::ProcessPwmFrameLocked(std::uint32_t now_ms) {
  SynchronizePwmSession(now_ms);
  const std::uint32_t sequence =
      hooks_.pwm_servo_frame_sequence(hooks_.context);
  if (sequence == last_pwm_frame_sequence_) {
    return;
  }
  last_pwm_frame_sequence_ = sequence;

  // The previously submitted shadow became active at this hardware boundary.
  if (pwm_shadow_waiting_for_commit_) {
    const std::uint8_t committed_offset_mask =
        pending_pwm_frame_.offset_commit_mask();
    pwm_controller_.CommitFrame(pending_pwm_frame_);
    const mentor_pi::mcu::PwmServoState& state = pwm_controller_.state();
    active_pwm_output_us_ = state.output_pulse_width_us;
    active_pwm_offset_us_ = state.offset_us;
    mentor_pi_mcu::app::microros::PwmServoTelemetry telemetry{};
    telemetry.timestamp_ms = now_ms;
    telemetry.target_pulse_width_us = state.target_pulse_width_us;
    telemetry.output_pulse_width_us = state.output_pulse_width_us;
    telemetry.offset_us = state.offset_us;
    telemetry.moving_mask = state.moving_mask;
    static_cast<void>(pwm_telemetry_.Publish(telemetry));
    pwm_shadow_waiting_for_commit_ = false;
    if (pwm_offset_waiting_for_commit_ && committed_offset_mask != 0U) {
      Complete(&pwm_offsets_slot_, pending_pwm_offset_token_,
               pending_pwm_offset_reply_);
      pwm_offset_waiting_for_commit_ = false;
      pending_pwm_offset_token_ = {};
      pending_pwm_offset_reply_ = {};
    }
  }

  // Calculate outside the ISR and submit a complete shadow for the following
  // common frame boundary.
  const mentor_pi::mcu::PwmFrameUpdate update =
      pwm_controller_.PrepareFollowingFrame();
  const Result submitted = hooks_.set_pwm_servo_shadow(
      hooks_.context, update.output_pulse_width_us());
  if (!submitted.ok()) {
    RecordPeripheralResult(4U, submitted, ErrorSource::kPwmServos);
    if (pwm_offset_waiting_for_commit_) {
      mentor_pi_mcu::app::microros::PwmOffsetsReply failed{};
      failed.result = submitted;
      failed.applied_mask = 0U;
      Complete(&pwm_offsets_slot_, pending_pwm_offset_token_, failed);
      pwm_offset_waiting_for_commit_ = false;
    }
    return;
  }
  pending_pwm_frame_ = update;
  pwm_shadow_waiting_for_commit_ = true;
}

void ControllerRuntime::PreparePendingPwmFrame(std::uint32_t now_ms) {
  CriticalGuard guard(this);
  // A boundary may have occurred between mailbox/service processing and this
  // critical section. Reconcile it first so every replacement starts from the
  // physically committed logical pulse, never from an uncommitted shadow.
  ProcessPwmFrameLocked(now_ms);
  if (!pwm_controller_.frame_prepare_pending()) {
    return;
  }

  const mentor_pi::mcu::PwmFrameUpdate base_frame =
      pwm_shadow_waiting_for_commit_ ? pending_pwm_frame_
                                     : pwm_controller_.PrepareFollowingFrame();
  const mentor_pi::mcu::PwmFrameUpdate update =
      pwm_controller_.PreparePendingFrame(base_frame);
  const Result submitted = hooks_.set_pwm_servo_shadow(
      hooks_.context, update.output_pulse_width_us());
  RecordPeripheralResult(4U, submitted, ErrorSource::kPwmServos);
  if (!submitted.ok()) {
    return;
  }
  pwm_controller_.ConfirmPendingFrameSubmitted(update);
  pending_pwm_frame_ = update;
  pwm_shadow_waiting_for_commit_ = true;
}

void ControllerRuntime::ProcessDiscreteOutputs(std::uint32_t now_ms) {
  mentor_pi::mcu::LedCommandSnapshot leds{};
  MailboxTag led_tag{};
  bool led_available = false;
  mentor_pi::mcu::BuzzerCommandSnapshot buzzer{};
  MailboxTag buzzer_tag{};
  bool buzzer_available = false;
  {
    CriticalGuard guard(this);
    led_available = led_mailbox_.ConsumeLatest(&leds);
    led_tag = led_mailbox_tag_;
    buzzer_available = buzzer_mailbox_.ConsumeLatest(&buzzer);
    buzzer_tag = buzzer_mailbox_tag_;
  }
  {
    CriticalGuard guard(this);
    const std::uint32_t current_session =
        session_generation_.load(std::memory_order_relaxed);
    if (led_available &&
        desired_session_active_.load(std::memory_order_relaxed) &&
        led_tag.session_generation == current_session &&
        led_tag.command_generation == leds.generation) {
      if (led_consumed_session_generation_ != led_tag.session_generation) {
        consumed_led_field_generation_.fill(0U);
        led_consumed_session_generation_ = led_tag.session_generation;
      }
      for (std::size_t led = 0; led < mentor_pi::mcu::kLedCount; ++led) {
        if (leds.field_generation[led] != 0U &&
            leds.field_generation[led] != consumed_led_field_generation_[led]) {
          const Result result =
              led_controller_.AcceptCommand(leds.commands[led], now_ms);
          consumed_led_field_generation_[led] = leds.field_generation[led];
          RecordPeripheralResult(5U, result, ErrorSource::kLeds);
        }
      }
    }
    if (buzzer_available &&
        desired_session_active_.load(std::memory_order_relaxed) &&
        buzzer_tag.session_generation == current_session &&
        buzzer_tag.command_generation == buzzer.generation) {
      const Result result =
          buzzer_controller_.AcceptHostCommand(buzzer.command, now_ms);
      RecordPeripheralResult(6U, result, ErrorSource::kBuzzer);
    }
  }

  BatteryAlarmRequest alarm{};
  if (battery_alarm_mailbox_.ConsumeLatest(&alarm)) {
    buzzer_controller_.TriggerBatteryAlarm(alarm.timestamp_ms);
  }
  auto led_output = led_controller_.Update(now_ms);
  led_output[mentor_pi::mcu::kHeartbeatLedIndex] =
      heartbeat_led_controller_.Update(
          successful_ros_heartbeats_.load(std::memory_order_relaxed));
  for (std::size_t led = 0; led < led_output.size(); ++led) {
    gpio_driver_.SetLed(led, led_output[led]);
  }
  const mentor_pi::mcu::BuzzerOutput buzzer_output =
      buzzer_controller_.Update(now_ms);
  const Result buzzer_result = gpio_driver_.SetBuzzer(
      buzzer_output.frequency_hz, buzzer_output.frequency_hz != 0U);
  RecordPeripheralResult(6U, buzzer_result, ErrorSource::kBuzzer);
}

void ControllerRuntime::ProcessRgb(std::uint32_t now_ms, std::uint32_t now_us) {
  Result polled{};
  {
    CriticalGuard guard(this);
    const bool was_busy = rgb_driver_.busy();
    const bool transfer_owner_is_stale =
        was_busy && active_rgb_session_generation_ != 0U &&
        (!desired_session_active_.load(std::memory_order_relaxed) ||
         active_rgb_session_generation_ !=
             session_generation_.load(std::memory_order_relaxed));
    polled = rgb_driver_.Poll(now_us);
    if (polled.code == ResultCode::kBusy && transfer_owner_is_stale) {
      rgb_driver_.Cancel();
      active_rgb_session_generation_ = 0U;
    } else if (polled.code != ResultCode::kBusy) {
      if (was_busy && polled.ok()) {
        active_rgb_state_ = inflight_rgb_state_;
      }
      active_rgb_session_generation_ = 0U;
    }
  }
  if (polled.code != ResultCode::kBusy) {
    RecordPeripheralResult(7U, polled, ErrorSource::kRgb);
  }

  mentor_pi::mcu::RgbCommandSnapshot snapshot{};
  MailboxTag tag{};
  bool available = false;
  {
    CriticalGuard guard(this);
    available = rgb_mailbox_.ConsumeLatest(&snapshot);
    tag = rgb_mailbox_tag_;
    if (available && desired_session_active_.load(std::memory_order_relaxed) &&
        tag.session_generation ==
            session_generation_.load(std::memory_order_relaxed) &&
        tag.command_generation == snapshot.generation) {
      if (rgb_consumed_session_generation_ != tag.session_generation) {
        consumed_rgb_field_generation_.fill(0U);
        rgb_consumed_session_generation_ = tag.session_generation;
        pending_rgb_state_ = active_rgb_state_;
      }
      bool changed = false;
      constexpr std::size_t kHostPixel = mentor_pi::mcu::kHostRgbPixelIndex;
      if (snapshot.field_generation[kHostPixel] != 0U &&
          snapshot.field_generation[kHostPixel] !=
              consumed_rgb_field_generation_[kHostPixel]) {
        pending_rgb_state_.red[kHostPixel] = snapshot.state.red[kHostPixel];
        pending_rgb_state_.green[kHostPixel] = snapshot.state.green[kHostPixel];
        pending_rgb_state_.blue[kHostPixel] = snapshot.state.blue[kHostPixel];
        consumed_rgb_field_generation_[kHostPixel] =
            snapshot.field_generation[kHostPixel];
        changed = true;
      }
      if (changed) {
        pending_rgb_session_generation_ = tag.session_generation;
        rgb_pending_ = true;
      }
    }
  }
  if (status_rgb_controller_.TransportSampleDue(now_ms)) {
    status_rgb_controller_.ObserveTransport(
        now_ms, hooks_.read_transport_activity(hooks_.context));
  }
  const StatusRgbColor status_color = status_rgb_controller_.Update(now_ms);

  Result started{ResultCode::kBusy, 0U};
  {
    CriticalGuard guard(this);
    const bool pending_is_current =
        pending_rgb_session_generation_ == 0U ||
        (desired_session_active_.load(std::memory_order_relaxed) &&
         pending_rgb_session_generation_ ==
             session_generation_.load(std::memory_order_relaxed));
    if (!pending_is_current) {
      pending_rgb_state_ = active_rgb_state_;
      pending_rgb_session_generation_ = 0U;
      rgb_pending_ = false;
    }

    constexpr std::size_t kStatusPixel = mentor_pi::mcu::kStatusRgbPixelIndex;
    if (pending_rgb_state_.red[kStatusPixel] != status_color.red ||
        pending_rgb_state_.green[kStatusPixel] != status_color.green ||
        pending_rgb_state_.blue[kStatusPixel] != status_color.blue) {
      pending_rgb_state_.red[kStatusPixel] = status_color.red;
      pending_rgb_state_.green[kStatusPixel] = status_color.green;
      pending_rgb_state_.blue[kStatusPixel] = status_color.blue;
      rgb_pending_ = true;
    }

    if (!rgb_driver_.busy() && rgb_pending_) {
      started = rgb_driver_.Begin(pending_rgb_state_, now_us + kRgbDeadlineUs);
      if (started.ok()) {
        inflight_rgb_state_ = pending_rgb_state_;
        rgb_pending_ = false;
        active_rgb_session_generation_ = pending_rgb_session_generation_;
        pending_rgb_session_generation_ = 0U;
      }
    }
  }
  if (!started.ok() && started.code != ResultCode::kBusy) {
    RecordPeripheralResult(7U, started, ErrorSource::kRgb);
  }
}

void ControllerRuntime::ProcessOled(std::uint32_t now_ms) {
  mentor_pi::mcu::OledCommandSnapshot snapshot{};
  MailboxTag tag{};
  bool command_available = false;
  {
    CriticalGuard guard(this);
    command_available = oled_mailbox_.ConsumeLatest(&snapshot);
    tag = oled_mailbox_tag_;
    if (command_available &&
        desired_session_active_.load(std::memory_order_relaxed) &&
        tag.session_generation ==
            session_generation_.load(std::memory_order_relaxed) &&
        tag.command_generation == snapshot.generation) {
      if (oled_consumed_session_generation_ != tag.session_generation) {
        consumed_oled_field_generation_.fill(0U);
        oled_consumed_session_generation_ = tag.session_generation;
        pending_oled_state_ = active_oled_state_;
      }
      bool changed = false;
      for (std::size_t line = 0U; line < mentor_pi::mcu::kOledHostLineCount;
           ++line) {
        if (snapshot.field_generation[line] == 0U ||
            snapshot.field_generation[line] ==
                consumed_oled_field_generation_[line]) {
          continue;
        }
        pending_oled_state_.lines[line] = snapshot.state.lines[line];
        consumed_oled_field_generation_[line] = snapshot.field_generation[line];
        changed = true;
      }
      if (changed) {
        pending_oled_session_generation_ = tag.session_generation;
        oled_pending_ = true;
      }
    }
  }
  BatteryDisplayState battery{};
  if (battery_display_mailbox_.ConsumeLatest(&battery)) {
    oled_battery_mv_ = battery.valid ? battery.voltage_mv : 0U;
    if (oled_battery_mv_ != rendered_oled_battery_mv_) {
      oled_pending_ = true;
    }
  }
  if (!oled_pending_ || now_ms - last_oled_attempt_ms_ < kOledRetryMs) {
    return;
  }
  {
    CriticalGuard guard(this);
    const bool pending_is_current =
        pending_oled_session_generation_ == 0U ||
        (desired_session_active_.load(std::memory_order_relaxed) &&
         pending_oled_session_generation_ ==
             session_generation_.load(std::memory_order_relaxed));
    if (!pending_is_current) {
      pending_oled_state_ = active_oled_state_;
      pending_oled_session_generation_ = 0U;
      oled_pending_ = oled_battery_mv_ != rendered_oled_battery_mv_;
    }
  }
  if (!oled_pending_) {
    return;
  }
  last_oled_attempt_ms_ = now_ms;
  const std::uint32_t deadline_ms = now_ms + kOledDeadlineMs;
  if (!oled_initialized_) {
    const Result initialized = oled_driver_.Initialize(deadline_ms);
    if (!initialized.ok()) {
      RecordPeripheralResult(2U, initialized, ErrorSource::kOled);
      return;
    }
    oled_initialized_ = true;
  }
  {
    CriticalGuard guard(this);
    const bool pending_is_current =
        pending_oled_session_generation_ == 0U ||
        (desired_session_active_.load(std::memory_order_relaxed) &&
         pending_oled_session_generation_ ==
             session_generation_.load(std::memory_order_relaxed));
    if (!pending_is_current) {
      pending_oled_state_ = active_oled_state_;
      pending_oled_session_generation_ = 0U;
      oled_pending_ = oled_battery_mv_ != rendered_oled_battery_mv_;
      return;
    }
  }
  const Result rendered =
      oled_driver_.Render(pending_oled_state_, oled_battery_mv_, deadline_ms);
  if (!rendered.ok()) {
    RecordPeripheralResult(2U, rendered, ErrorSource::kOled);
    return;
  }
  active_oled_state_ = pending_oled_state_;
  pending_oled_session_generation_ = 0U;
  rendered_oled_battery_mv_ = oled_battery_mv_;
  oled_pending_ = false;
}

}  // namespace mentor_pi_mcu::app::controller
