// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/battery_monitor.h"
#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/motor_controller.h"
#include "mentor_pi_mcu/domain/pattern_controller.h"
#include "mentor_pi_mcu/domain/pwm_servo_controller.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

using fuzz::FuzzInput;
using fuzz::Require;

bool Equal(Result left, Result right) {
  return left.code == right.code && left.detail == right.detail;
}

bool Equal(const MotorChannelState& left, const MotorChannelState& right) {
  return left.target_rps == right.target_rps &&
         left.measured_rps == right.measured_rps &&
         left.encoder_count == right.encoder_count &&
         left.output_permille == right.output_permille &&
         left.armed == right.armed &&
         left.watchdog_stopped == right.watchdog_stopped;
}

bool Equal(const PwmServoState& left, const PwmServoState& right) {
  return left.target_pulse_width_us == right.target_pulse_width_us &&
         left.output_pulse_width_us == right.output_pulse_width_us &&
         left.offset_us == right.offset_us &&
         left.moving_mask == right.moving_mask;
}

Result ExpectedDefaultMotorResult(const MotorCommand& command) {
  return ValidateMotorCommand(command, kMotorImplementationMaximumRps);
}

MotorCommand ReadMotor(FuzzInput* input) {
  MotorCommand command{};
  command.update_mask = input->ReadU8();
  for (float& target : command.target_rps) {
    target = input->ReadBiasedFloat();
  }
  return command;
}

void CheckMotorState(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const MotorCommand command = ReadMotor(&input);
  const std::uint32_t now_us = input.ReadU32();
  const MotorControlConfiguration configuration =
      DefaultAdrcMotorControlConfiguration();
  MotorController controller(configuration);
  controller.SetSessionActive(true);
  const auto before = controller.channels();
  const Result expected = ExpectedDefaultMotorResult(command);
  const Result result = controller.AcceptCommand(command, now_us);
  Require(Equal(result, expected));

  if (!result.ok()) {
    for (std::size_t index = 0U; index < kMotorCount; ++index) {
      Require(Equal(controller.channels()[index], before[index]));
    }
    return;
  }
  for (std::size_t index = 0U; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      Require(Equal(controller.channels()[index], before[index]));
      continue;
    }
    const MotorChannelState& channel = controller.channels()[index];
    Require(channel.target_rps == command.target_rps[index]);
    Require(channel.armed == (command.target_rps[index] != 0.0F));
    Require(channel.output_permille == 0);
  }

  controller.EvaluateLeases(now_us + kMotorLeaseExpiryUs);
  for (std::size_t index = 0U; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        command.target_rps[index] != 0.0F) {
      Require(!controller.channels()[index].armed);
      Require(controller.channels()[index].target_rps == 0.0F);
      Require(controller.lease_expiry_count(index) == 1U);
    }
  }
}

void CheckMotorModelService(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const auto requested_model = static_cast<MotorModel>(input.ReadU8());
  MotorController controller;
  const MotorModelChange change = controller.SetModel(requested_model);
  if (IsValidMotorModel(requested_model)) {
    Require(change.result.ok());
    Require(change.active_profile.model == requested_model);
    Require(controller.profile().model == requested_model);
  } else {
    Require(change.result.code == ResultCode::kInvalidArgument);
    Require(change.active_profile.model == MotorModel::kJga27);
    Require(controller.profile().model == MotorModel::kJga27);
  }

  const MotorControlConfiguration configuration =
      DefaultAdrcMotorControlConfiguration();
  MotorController moving(configuration);
  moving.SetSessionActive(true);
  MotorCommand start{};
  start.update_mask = 0x01U;
  start.target_rps[0] = 0.25F;
  Require(moving.AcceptCommand(start, 1U).ok());
  Require(moving.SetModel(MotorModel::kJga27).result.ok());
  const MotorModelChange busy = moving.SetModel(MotorModel::kJgb520);
  Require(busy.result.code == ResultCode::kBusy);
  Require(moving.profile().model == MotorModel::kJga27);
  start.target_rps[0] = 0.0F;
  Require(moving.AcceptCommand(start, 2U).ok());
  Require(moving.SetModel(MotorModel::kJgb520).result.ok());
}

PwmServoCommand ReadPwm(FuzzInput* input) {
  PwmServoCommand command{};
  command.update_mask = input->ReadU8();
  command.duration_ms = input->ReadU16();
  for (std::uint16_t& pulse : command.pulse_width_us) {
    pulse = input->ReadU16();
  }
  return command;
}

PwmServoOffsetCommand ReadOffsets(FuzzInput* input) {
  PwmServoOffsetCommand command{};
  command.update_mask = input->ReadU8();
  for (std::int16_t& offset : command.offset_us) {
    offset = input->ReadI16();
  }
  return command;
}

void CheckPwmState(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const PwmServoCommand command = ReadPwm(&input);
  const Result validation = ValidatePwmServoCommand(command);
  PwmServoController controller;
  const PwmServoState before = controller.state();
  const Result result = controller.AcceptCommand(command);
  Require(Equal(result, validation));
  if (!result.ok()) {
    Require(Equal(controller.state(), before));
    Require(!controller.frame_prepare_pending());
  } else {
    for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
      const auto bit = static_cast<std::uint8_t>(1U << index);
      const std::uint16_t expected = (command.update_mask & bit) != 0U
                                         ? command.pulse_width_us[index]
                                         : before.target_pulse_width_us[index];
      Require(controller.state().target_pulse_width_us[index] == expected);
    }
    PwmFrameUpdate frame =
        controller.PreparePendingFrame(controller.PrepareFollowingFrame());
    controller.ConfirmPendingFrameSubmitted(frame);
    controller.CommitFrame(frame);
    const std::uint8_t following_frames =
        static_cast<std::uint8_t>(input.ReadU8() % 9U);
    for (std::uint8_t index = 0U; index < following_frames; ++index) {
      frame = controller.PrepareFollowingFrame();
      controller.CommitFrame(frame);
    }
    for (std::uint16_t pulse : controller.state().output_pulse_width_us) {
      Require(pulse >= kPwmMinimumPulseUs && pulse <= kPwmMaximumPulseUs);
    }
  }

  const PwmServoOffsetCommand offsets = ReadOffsets(&input);
  const Result offset_validation = ValidatePwmServoOffsets(offsets);
  PwmServoController offset_controller;
  const PwmServoState offset_before = offset_controller.state();
  const Result offset_result = offset_controller.StageOffsets(offsets);
  Require(Equal(offset_result, offset_validation));
  if (!offset_result.ok()) {
    Require(Equal(offset_controller.state(), offset_before));
    Require(!offset_controller.offset_commit_pending());
  } else {
    std::uint8_t changed_mask = 0U;
    for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
      const auto bit = static_cast<std::uint8_t>(1U << index);
      if ((offsets.update_mask & bit) != 0U && offsets.offset_us[index] != 0) {
        changed_mask = static_cast<std::uint8_t>(changed_mask | bit);
      }
    }
    Require(offset_controller.offset_commit_pending() == (changed_mask != 0U));
    if (changed_mask != 0U) {
      PwmFrameUpdate frame = offset_controller.PreparePendingFrame(
          offset_controller.PrepareFollowingFrame());
      Require(frame.offset_commit_mask() == offsets.update_mask);
      offset_controller.ConfirmPendingFrameSubmitted(frame);
      offset_controller.CommitFrame(frame);
      Require(!offset_controller.offset_commit_pending());
      for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
        const auto bit = static_cast<std::uint8_t>(1U << index);
        const std::int16_t expected = (offsets.update_mask & bit) != 0U
                                          ? offsets.offset_us[index]
                                          : static_cast<std::int16_t>(0);
        Require(offset_controller.state().offset_us[index] == expected);
      }
    }
  }

  PwmServoController busy_controller;
  PwmServoOffsetCommand first{};
  first.update_mask = 0x01U;
  first.offset_us[0] = 50;
  Require(busy_controller.StageOffsets(first).ok());
  const Result busy_result = busy_controller.StageOffsets(offsets);
  Require(Equal(busy_result, offset_validation.ok()
                                 ? Result{ResultCode::kBusy, 0U}
                                 : offset_validation));
}

void CheckPatternState(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const std::uint32_t now_ms = input.ReadU32();
  const LedCommand led{input.ReadU8(), input.ReadU16(), input.ReadU16(),
                       input.ReadU16()};
  LedController led_subject;
  LedController led_control;
  for (std::uint8_t id = kFirstHostLedId; id <= kLastHostLedId; ++id) {
    const LedCommand baseline{id, 1U, 0U, 0U};
    Require(led_subject.AcceptCommand(baseline, 0U).ok());
    Require(led_control.AcceptCommand(baseline, 0U).ok());
  }
  const Result led_validation = ValidateLedCommand(led);
  Require(Equal(led_subject.AcceptCommand(led, now_ms), led_validation));
  if (!led_validation.ok()) {
    Require(led_subject.Update(now_ms) == led_control.Update(now_ms));
  }

  const BuzzerCommand buzzer{input.ReadU16(), input.ReadU16(), input.ReadU16(),
                             input.ReadU16()};
  BuzzerController buzzer_subject;
  BuzzerController buzzer_control;
  const BuzzerCommand baseline{1000U, 10U, 10U, 0U};
  Require(buzzer_subject.AcceptHostCommand(baseline, 0U).ok());
  Require(buzzer_control.AcceptHostCommand(baseline, 0U).ok());
  const Result buzzer_validation = ValidateBuzzerCommand(buzzer);
  Require(Equal(buzzer_subject.AcceptHostCommand(buzzer, now_ms),
                buzzer_validation));
  if (!buzzer_validation.ok()) {
    const BuzzerOutput subject = buzzer_subject.Update(now_ms);
    const BuzzerOutput control = buzzer_control.Update(now_ms);
    Require(subject.frequency_hz == control.frequency_hz);
    Require(subject.battery_alarm_active == control.battery_alarm_active);
  }
}

void CheckBatteryService(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const std::uint16_t threshold_mv = input.ReadU16();
  BatteryMonitor monitor;
  const BatteryState before = monitor.state();
  const Result validation = ValidateBatteryThreshold(threshold_mv);
  const BatteryThresholdUpdate update = monitor.SetLowThreshold(threshold_mv);
  Require(Equal(update.result, validation));
  Require(update.active_threshold_mv ==
          (validation.ok() ? threshold_mv : before.low_threshold_mv));
  Require(monitor.state().low_threshold_mv == update.active_threshold_mv);
  if (!validation.ok()) {
    Require(monitor.state().voltage_mv == before.voltage_mv);
    Require(monitor.state().valid == before.valid);
    Require(monitor.state().below_threshold == before.below_threshold);
  }
}

StopBusServosCommand ReadStop(FuzzInput* input) {
  StopBusServosCommand command{};
  command.count = input->ReadU8();
  for (std::uint8_t& id : command.servo_id) {
    id = input->ReadU8();
  }
  return command;
}

void CheckBusStopService(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const StopBusServosCommand stop = ReadStop(&input);
  const Result validation = ValidateStopBusServosCommand(stop);
  BusServoScheduler scheduler;
  BusServoCommand move{};
  move.count = 2U;
  move.servo_id[0] = 1U;
  move.servo_id[1] = 2U;
  move.position[0] = 400U;
  move.position[1] = 600U;
  move.duration_ms = 20U;
  Require(scheduler.SubmitMove(move).result.ok());
  const Result result = scheduler.AcceptStop(stop);
  Require(Equal(result, validation));
  if (!result.ok()) {
    const ScheduledBusFrame frame = scheduler.BeginFrame();
    Require(frame.result.ok());
    Require(frame.kind == ScheduledBusFrameKind::kMove);
    const ParsedBusServoFrame parsed =
        BusServoCodec::ParseFrame(frame.frame.bytes.data(), frame.frame.size);
    Require(parsed.result.ok());
    Require(parsed.servo_id == 1U);
    return;
  }

  for (std::size_t index = 0U; index < stop.count; ++index) {
    const ScheduledBusFrame frame = scheduler.BeginFrame();
    Require(frame.result.ok());
    Require(frame.kind == ScheduledBusFrameKind::kStop);
    const ParsedBusServoFrame parsed =
        BusServoCodec::ParseFrame(frame.frame.bytes.data(), frame.frame.size);
    Require(parsed.result.ok());
    Require(parsed.servo_id == stop.servo_id[index]);
    Require(parsed.opcode == BusServoOpcode::kMoveStop);
    scheduler.CompleteFrame(true);
  }
  Require(!scheduler.has_work());
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::CheckMotorState(data, size);
  mentor_pi::mcu::CheckMotorModelService(data, size);
  mentor_pi::mcu::CheckPwmState(data, size);
  mentor_pi::mcu::CheckPatternState(data, size);
  mentor_pi::mcu::CheckBatteryService(data, size);
  mentor_pi::mcu::CheckBusStopService(data, size);
  return 0;
}
