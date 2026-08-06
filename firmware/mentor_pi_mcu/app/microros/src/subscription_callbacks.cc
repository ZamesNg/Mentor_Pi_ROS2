// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "mentor_pi_mcu/domain/validation.h"
#include "runtime_internal.h"

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::size_t ToIndex(SubscriptionIndex index) {
  return static_cast<std::size_t>(index);
}

mentor_pi::mcu::Result InactiveResult() {
  return {mentor_pi::mcu::ResultCode::kBusy, 0U};
}

}  // namespace

void MicroRosRuntime::OnMotorCommand(
    const mentor_pi_interfaces__msg__MotorCommand& message) {
  mentor_pi::mcu::MotorCommand command{};
  command.update_mask = message.update_mask;
  std::copy_n(message.target_rps, command.target_rps.size(),
              command.target_rps.begin());

  mentor_pi::mcu::Result result = mentor_pi::mcu::ValidateMotorCommand(
      command, hooks_.motor_max_rps(hooks_.context));
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_motor_command(hooks_.context, command, NowUs());
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kMotors), result,
                      admission.overwrote_unread, ErrorSource::kMotors);
}

void MicroRosRuntime::OnPwmServoCommand(
    const mentor_pi_interfaces__msg__PwmServoCommand& message) {
  mentor_pi::mcu::PwmServoCommand command{};
  command.update_mask = message.update_mask;
  command.duration_ms = message.duration_ms;
  std::copy_n(message.pulse_width_us, command.pulse_width_us.size(),
              command.pulse_width_us.begin());

  mentor_pi::mcu::Result result =
      mentor_pi::mcu::ValidatePwmServoCommand(command);
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_pwm_servo_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kPwmServos), result,
                      admission.overwrote_unread, ErrorSource::kPwmServos);
}

void MicroRosRuntime::OnBusServoCommand(
    const mentor_pi_interfaces__msg__BusServoCommand& message) {
  mentor_pi::mcu::BusServoCommand command{};
  command.count = message.count;
  command.duration_ms = message.duration_ms;
  std::copy_n(message.servo_id, command.servo_id.size(),
              command.servo_id.begin());
  std::copy_n(message.position, command.position.size(),
              command.position.begin());

  mentor_pi::mcu::Result result =
      mentor_pi::mcu::ValidateBusServoCommand(command);
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_bus_servo_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kBusServos), result,
                      admission.overwrote_unread, ErrorSource::kBusServos);
}

void MicroRosRuntime::OnLedCommand(
    const mentor_pi_interfaces__msg__LedCommand& message) {
  const mentor_pi::mcu::LedCommand command{message.led_id, message.on_time_ms,
                                           message.off_time_ms, message.repeat};
  mentor_pi::mcu::Result result = mentor_pi::mcu::ValidateLedCommand(command);
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_led_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kLeds), result,
                      admission.overwrote_unread, ErrorSource::kLeds);
}

void MicroRosRuntime::OnBuzzerCommand(
    const mentor_pi_interfaces__msg__BuzzerCommand& message) {
  const mentor_pi::mcu::BuzzerCommand command{
      message.frequency_hz, message.on_time_ms, message.off_time_ms,
      message.repeat};
  mentor_pi::mcu::Result result =
      mentor_pi::mcu::ValidateBuzzerCommand(command);
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_buzzer_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kBuzzer), result,
                      admission.overwrote_unread, ErrorSource::kBuzzer);
}

void MicroRosRuntime::OnRgbCommand(
    const mentor_pi_interfaces__msg__RgbCommand& message) {
  mentor_pi::mcu::RgbCommand command{};
  command.update_mask = message.update_mask;
  std::copy_n(message.red, command.red.size(), command.red.begin());
  std::copy_n(message.green, command.green.size(), command.green.begin());
  std::copy_n(message.blue, command.blue.size(), command.blue.begin());

  mentor_pi::mcu::Result result = mentor_pi::mcu::ValidateRgbCommand(command);
  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_rgb_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kRgb), result,
                      admission.overwrote_unread, ErrorSource::kRgb);
}

void MicroRosRuntime::OnOledCommand(
    const mentor_pi_interfaces__msg__OledCommand& message) {
  mentor_pi::mcu::OledCommand command{};
  command.update_mask = message.update_mask;
  const std::array<const rosidl_runtime_c__String*,
                   mentor_pi::mcu::kOledHostLineCount>
      source_lines{&message.line_1, &message.line_2};
  mentor_pi::mcu::Result result{};
  for (std::size_t line = 0U; line < mentor_pi::mcu::kOledHostLineCount;
       ++line) {
    const auto selected = static_cast<std::uint8_t>(1U << line);
    if ((command.update_mask & selected) == 0U) {
      continue;
    }
    const rosidl_runtime_c__String& source = *source_lines[line];
    if (source.size > mentor_pi::mcu::kOledLineCapacity ||
        (source.size != 0U && source.data == nullptr)) {
      result = {mentor_pi::mcu::ResultCode::kInvalidArgument,
                static_cast<std::uint16_t>(line + 1U)};
      break;
    }
    auto& destination = command.lines[line];
    if (source.size != 0U) {
      std::memcpy(destination.bytes.data(), source.data, source.size);
    }
    destination.size = static_cast<std::uint8_t>(source.size);
    destination.bytes[source.size] = '\0';
  }
  if (result.ok()) {
    result = mentor_pi::mcu::ValidateOledCommand(command);
  }

  mentor_pi::mcu::CommandAdmission admission{};
  if (result.ok() && lifecycle_.state() != SessionState::kActive) {
    result = InactiveResult();
  }
  if (result.ok()) {
    admission = hooks_.publish_oled_command(hooks_.context, command);
    result = admission.result;
  }
  RecordCommandResult(ToIndex(SubscriptionIndex::kOled), result,
                      admission.overwrote_unread, ErrorSource::kOled);
}

void MotorSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnMotorCommand(
        *static_cast<const mentor_pi_interfaces__msg__MotorCommand*>(message));
  }
}

void PwmServoSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnPwmServoCommand(
        *static_cast<const mentor_pi_interfaces__msg__PwmServoCommand*>(
            message));
  }
}

void BusServoSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnBusServoCommand(
        *static_cast<const mentor_pi_interfaces__msg__BusServoCommand*>(
            message));
  }
}

void LedSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnLedCommand(
        *static_cast<const mentor_pi_interfaces__msg__LedCommand*>(message));
  }
}

void BuzzerSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnBuzzerCommand(
        *static_cast<const mentor_pi_interfaces__msg__BuzzerCommand*>(message));
  }
}

void RgbSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnRgbCommand(
        *static_cast<const mentor_pi_interfaces__msg__RgbCommand*>(message));
  }
}

void OledSubscriptionCallback(const void* message) {
  if (message != nullptr) {
    RuntimeInstance().OnOledCommand(
        *static_cast<const mentor_pi_interfaces__msg__OledCommand*>(message));
  }
}

}  // namespace mentor_pi_mcu::app::microros
