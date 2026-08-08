// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::CommandAdmission;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi_mcu::app::microros::ServiceToken;

CommandAdmission InactiveAdmission() {
  return {{ResultCode::kBusy, 0U}, false, 0U};
}

bool RequestsMotorMotion(const mentor_pi::mcu::MotorCommand& command) {
  for (std::size_t motor = 0U; motor < mentor_pi::mcu::kMotorCount; ++motor) {
    const auto bit = static_cast<std::uint8_t>(1U << motor);
    if ((command.update_mask & bit) != 0U &&
        command.target_rps[motor] != 0.0F) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool ControllerRuntime::TokenIsCurrent(ServiceToken token) const {
  return token.session_generation != 0U && token.request_generation != 0U &&
         desired_session_active_.load(std::memory_order_acquire) &&
         token.session_generation ==
             session_generation_.load(std::memory_order_acquire);
}

CommandAdmission ControllerRuntime::PublishMotorCommand(
    const mentor_pi::mcu::MotorCommand& command, std::uint32_t accepted_at_us) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    motor_controller_.RecordRejectedCommand(command.update_mask);
    return InactiveAdmission();
  }
  // The build-time lock applies only to an otherwise valid command. Preserve
  // precise mask, finite-value, and model-range validation results.
  const Result validation =
      mentor_pi::mcu::ValidateMotorCommand(command, MotorMaximumRps());
  if (!validation.ok()) {
    motor_controller_.RecordRejectedCommand(command.update_mask);
    return {validation, false, 0U};
  }
  if (RequestsMotorMotion(command) &&
      !motor_controller_.nonzero_motion_enabled()) {
    motor_controller_.RecordRejectedCommand(command.update_mask);
    return {{ResultCode::kUnsupported, 0U}, false, 0U};
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    motor_controller_.RecordRejectedCommand(command.update_mask);
    return InactiveAdmission();
  }
  // Every merged-mailbox worker consumes under this same controller critical
  // section. ResetMergedFields therefore has strict discard-before-publish
  // ordering while remaining a producer-side mailbox operation.
  if (motor_mailbox_session_generation_ != observed_session) {
    motor_mailbox_.ResetMergedFields();
    motor_mailbox_tag_ = {};
    motor_mailbox_session_generation_ = observed_session;
  }
  CommandAdmission admission =
      motor_mailbox_.Publish(command, MotorMaximumRps(), accepted_at_us);
  if (!admission.result.ok()) {
    motor_controller_.RecordRejectedCommand(command.update_mask);
  }
  if (admission.result.ok()) {
    motor_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishPwmServoCommand(
    const mentor_pi::mcu::PwmServoCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  if (pwm_mailbox_session_generation_ != observed_session) {
    pwm_mailbox_.ResetMergedFields();
    pwm_mailbox_tag_ = {};
    pwm_mailbox_session_generation_ = observed_session;
  }
  CommandAdmission admission = pwm_mailbox_.Publish(command);
  if (admission.result.ok()) {
    pwm_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishBusServoCommand(
    const mentor_pi::mcu::BusServoCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  CommandAdmission admission = bus_mailbox_.Publish(command);
  if (admission.result.ok()) {
    bus_mailbox_tag_ = {observed_session, admission.generation};
    last_bus_command_generation_ = admission.generation;
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishLedCommand(
    const mentor_pi::mcu::LedCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  if (led_mailbox_session_generation_ != observed_session) {
    led_mailbox_.ResetMergedFields();
    led_mailbox_tag_ = {};
    led_mailbox_session_generation_ = observed_session;
  }
  CommandAdmission admission = led_mailbox_.Publish(command);
  if (admission.result.ok()) {
    led_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishBuzzerCommand(
    const mentor_pi::mcu::BuzzerCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  CommandAdmission admission = buzzer_mailbox_.Publish(command);
  if (admission.result.ok()) {
    buzzer_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishRgbCommand(
    const mentor_pi::mcu::RgbCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  if (rgb_mailbox_session_generation_ != observed_session) {
    rgb_mailbox_.ResetMergedFields();
    rgb_mailbox_tag_ = {};
    rgb_mailbox_session_generation_ = observed_session;
  }
  CommandAdmission admission = rgb_mailbox_.Publish(command);
  if (admission.result.ok()) {
    rgb_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

CommandAdmission ControllerRuntime::PublishOledCommand(
    const mentor_pi::mcu::OledCommand& command) {
  const std::uint32_t observed_session =
      session_generation_.load(std::memory_order_acquire);
  if (observed_session == 0U ||
      !desired_session_active_.load(std::memory_order_acquire) ||
      session_generation_.load(std::memory_order_acquire) != observed_session) {
    return InactiveAdmission();
  }
  CriticalGuard guard(this);
  if (!desired_session_active_.load(std::memory_order_relaxed) ||
      session_generation_.load(std::memory_order_relaxed) != observed_session) {
    return InactiveAdmission();
  }
  if (oled_mailbox_session_generation_ != observed_session) {
    oled_mailbox_.ResetMergedFields();
    oled_mailbox_tag_ = {};
    oled_mailbox_session_generation_ = observed_session;
  }
  CommandAdmission admission = oled_mailbox_.Publish(command);
  if (admission.result.ok()) {
    oled_mailbox_tag_ = {observed_session, admission.generation};
  }
  return admission;
}

bool ControllerRuntime::ReadMotorTelemetry(
    mentor_pi_mcu::app::microros::MotorTelemetry* output) {
  return motor_telemetry_.ConsumeLatest(output);
}

bool ControllerRuntime::ReadPwmServoTelemetry(
    mentor_pi_mcu::app::microros::PwmServoTelemetry* output) {
  return pwm_telemetry_.ConsumeLatest(output);
}

bool ControllerRuntime::ReadImuTelemetry(
    mentor_pi_mcu::app::microros::ImuTelemetry* output) {
  return imu_telemetry_.ConsumeLatest(output);
}

bool ControllerRuntime::ReadBatteryTelemetry(
    mentor_pi_mcu::app::microros::BatteryTelemetry* output) {
  return battery_telemetry_.ConsumeLatest(output);
}

bool ControllerRuntime::PopButtonEvent(mentor_pi::mcu::ButtonEvent* output) {
  return button_controller_.PopEvent(output);
}

bool ControllerRuntime::DispatchMotorModel(ServiceToken token,
                                           mentor_pi::mcu::MotorModel model) {
  return Dispatch(&motor_model_slot_, token, MotorModelRequest{model});
}

bool ControllerRuntime::PollMotorModel(
    ServiceToken token, mentor_pi_mcu::app::microros::MotorModelReply* output) {
  return Poll(&motor_model_slot_, token, output);
}

bool ControllerRuntime::DispatchMotorPid(
    ServiceToken token, const mentor_pi::mcu::SetMotorPidCommand& command) {
  return Dispatch(&motor_pid_slot_, token, command);
}

bool ControllerRuntime::PollMotorPid(
    ServiceToken token, mentor_pi_mcu::app::microros::MotorPidReply* output) {
  return Poll(&motor_pid_slot_, token, output);
}

bool ControllerRuntime::CancelMotorPid(ServiceToken token) {
  CriticalGuard guard(this);
  if (motor_pid_slot_.token.session_generation != token.session_generation ||
      motor_pid_slot_.token.request_generation != token.request_generation) {
    return true;
  }
  SlotState state = motor_pid_slot_.state.load(std::memory_order_acquire);
  while (true) {
    switch (state) {
      case SlotState::kIdle:
      case SlotState::kCanceled:
        return true;
      case SlotState::kComplete:
        return false;
      case SlotState::kWriting:
        return false;
      case SlotState::kReady:
      case SlotState::kProcessing: {
        const SlotState replacement = state == SlotState::kReady
                                          ? SlotState::kIdle
                                          : SlotState::kCanceled;
        if (motor_pid_slot_.state.compare_exchange_weak(
                state, replacement, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
          return true;
        }
        break;
      }
    }
  }
}

bool ControllerRuntime::DispatchPwmOffsets(
    ServiceToken token, const mentor_pi::mcu::PwmServoOffsetCommand& command) {
  return Dispatch(&pwm_offsets_slot_, token, command);
}

bool ControllerRuntime::PollPwmOffsets(
    ServiceToken token, mentor_pi_mcu::app::microros::PwmOffsetsReply* output) {
  return Poll(&pwm_offsets_slot_, token, output);
}

bool ControllerRuntime::DispatchBatteryThreshold(ServiceToken token,
                                                 std::uint16_t threshold_mv) {
  return Dispatch(&battery_threshold_slot_, token,
                  BatteryThresholdRequest{threshold_mv});
}

bool ControllerRuntime::PollBatteryThreshold(
    ServiceToken token,
    mentor_pi_mcu::app::microros::BatteryThresholdReply* output) {
  return Poll(&battery_threshold_slot_, token, output);
}

bool ControllerRuntime::DispatchBusGetState(
    ServiceToken token,
    const mentor_pi::mcu::GetBusServoStateCommand& command) {
  BusServiceRequest request{};
  request.kind = mentor_pi_mcu::app::microros::BusServiceKind::kGetState;
  request.get_state = command;
  return Dispatch(&bus_service_slot_, token, request);
}

bool ControllerRuntime::PollBusGetState(
    ServiceToken token,
    mentor_pi_mcu::app::microros::GetBusServoStateReply* output) {
  BusServiceReply reply{};
  if (!Poll(&bus_service_slot_, token, &reply) ||
      reply.kind != mentor_pi_mcu::app::microros::BusServiceKind::kGetState) {
    return false;
  }
  *output = reply.get_state;
  return true;
}

bool ControllerRuntime::DispatchBusConfigure(
    ServiceToken token,
    const mentor_pi::mcu::ConfigureBusServoCommand& command) {
  BusServiceRequest request{};
  request.kind = mentor_pi_mcu::app::microros::BusServiceKind::kConfigure;
  request.configure = command;
  return Dispatch(&bus_service_slot_, token, request);
}

bool ControllerRuntime::PollBusConfigure(
    ServiceToken token,
    mentor_pi_mcu::app::microros::ConfigureBusServoReply* output) {
  BusServiceReply reply{};
  if (!Poll(&bus_service_slot_, token, &reply) ||
      reply.kind != mentor_pi_mcu::app::microros::BusServiceKind::kConfigure) {
    return false;
  }
  *output = reply.configure;
  return true;
}

bool ControllerRuntime::DispatchBusStop(
    ServiceToken token, const mentor_pi::mcu::StopBusServosCommand& command) {
  BusServiceRequest request{};
  request.kind = mentor_pi_mcu::app::microros::BusServiceKind::kStop;
  request.stop = command;
  const bool dispatched = Dispatch(&bus_service_slot_, token, request);
  if (dispatched) {
    CriticalGuard guard(this);
    bus_stop_watermark_ = last_bus_command_generation_;
  }
  return dispatched;
}

bool ControllerRuntime::PollBusStop(
    ServiceToken token,
    mentor_pi_mcu::app::microros::StopBusServosReply* output) {
  BusServiceReply reply{};
  if (!Poll(&bus_service_slot_, token, &reply) ||
      reply.kind != mentor_pi_mcu::app::microros::BusServiceKind::kStop) {
    return false;
  }
  *output = reply.stop;
  return true;
}

}  // namespace mentor_pi_mcu::app::controller
