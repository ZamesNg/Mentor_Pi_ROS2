// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstddef>
#include <cstdint>
#include <limits>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/command_mailboxes.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

using fuzz::FuzzInput;
using fuzz::Require;

std::uint32_t NextGeneration(std::uint32_t generation) {
  return generation == std::numeric_limits<std::uint32_t>::max()
             ? 1U
             : generation + 1U;
}

bool Equal(Result left, Result right) {
  return left.code == right.code && left.detail == right.detail;
}

bool Equal(const BusServoCommand& left, const BusServoCommand& right) {
  return left.count == right.count && left.servo_id == right.servo_id &&
         left.position == right.position &&
         left.duration_ms == right.duration_ms;
}

bool Equal(const LedCommand& left, const LedCommand& right) {
  return left.led_id == right.led_id && left.on_time_ms == right.on_time_ms &&
         left.off_time_ms == right.off_time_ms && left.repeat == right.repeat;
}

bool Equal(const BuzzerCommand& left, const BuzzerCommand& right) {
  return left.frequency_hz == right.frequency_hz &&
         left.on_time_ms == right.on_time_ms &&
         left.off_time_ms == right.off_time_ms && left.repeat == right.repeat;
}

bool Equal(const BoundedText& left, const BoundedText& right) {
  return left.size == right.size && left.bytes == right.bytes;
}

BoundedText ReadText(FuzzInput* input) {
  BoundedText text{};
  text.size = input->ReadU8();
  for (char& byte : text.bytes) {
    byte = static_cast<char>(input->ReadU8());
  }
  return text;
}

void CheckAdmission(const CommandAdmission& admission, Result expected,
                    bool overwrote, std::uint32_t generation) {
  Require(Equal(admission.result, expected));
  Require(admission.overwrote_unread == overwrote);
  Require(admission.generation == generation);
}

void CheckMotorMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  MotorCommand command{};
  command.update_mask = input.ReadU8();
  for (float& target : command.target_rps) {
    target = input.ReadBiasedFloat();
  }
  const float maximum_rps = input.ReadBiasedFloat();
  const std::uint32_t accepted_at_us = input.ReadU32();
  const Result validation = ValidateMotorCommand(command, maximum_rps);

  MotorCommandMailbox mailbox;
  const MotorCommand baseline{0x0fU, {0.1F, -0.1F, 0.2F, -0.2F}};
  CheckAdmission(mailbox.Publish(baseline, 6.0F, 11U), OkResult(), false, 1U);
  const CommandAdmission admission =
      mailbox.Publish(command, maximum_rps, accepted_at_us);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);
  Require(mailbox.overwrite_count() == (validation.ok() ? 1U : 0U));
  Require(!mailbox.ConsumeLatest(nullptr));

  MotorCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.generation == (validation.ok() ? 2U : 1U));
  for (std::size_t index = 0U; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    const bool replaced = validation.ok() && (command.update_mask & bit) != 0U;
    Require(
        snapshot.target_rps[index] ==
        (replaced ? command.target_rps[index] : baseline.target_rps[index]));
    Require(snapshot.accepted_at_us[index] ==
            (replaced ? accepted_at_us : 11U));
    Require(snapshot.field_generation[index] == (replaced ? 2U : 1U));
  }
  Require(!mailbox.ConsumeLatest(&snapshot));

  mailbox.ResetMergedFields();
  MotorCommand fresh{};
  fresh.update_mask = 0x04U;
  fresh.target_rps[2] = 0.25F;
  const std::uint32_t fresh_generation =
      NextGeneration(validation.ok() ? 2U : 1U);
  CheckAdmission(mailbox.Publish(fresh, 6.0F, 77U), OkResult(), false,
                 fresh_generation);
  Require(mailbox.ConsumeLatest(&snapshot));
  for (std::size_t index = 0U; index < kMotorCount; ++index) {
    const bool selected = index == 2U;
    Require(snapshot.target_rps[index] == (selected ? 0.25F : 0.0F));
    Require(snapshot.accepted_at_us[index] == (selected ? 77U : 0U));
    Require(snapshot.field_generation[index] ==
            (selected ? fresh_generation : 0U));
  }
}

void CheckPwmMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  PwmServoCommand command{};
  command.update_mask = input.ReadU8();
  command.duration_ms = input.ReadU16();
  for (std::uint16_t& pulse : command.pulse_width_us) {
    pulse = input.ReadU16();
  }
  const Result validation = ValidatePwmServoCommand(command);

  PwmCommandMailbox mailbox;
  const PwmServoCommand baseline{0x0fU, 20U, {600U, 700U, 800U, 900U}};
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);
  Require(mailbox.overwrite_count() == (validation.ok() ? 1U : 0U));

  PwmCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    const bool replaced = validation.ok() && (command.update_mask & bit) != 0U;
    Require(snapshot.pulse_width_us[index] ==
            (replaced ? command.pulse_width_us[index]
                      : baseline.pulse_width_us[index]));
    Require(snapshot.duration_ms[index] ==
            (replaced ? command.duration_ms : baseline.duration_ms));
    Require(snapshot.field_generation[index] == (replaced ? 2U : 1U));
  }

  mailbox.ResetMergedFields();
  const PwmServoCommand fresh{0x08U, 40U, {0U, 0U, 0U, 1600U}};
  const std::uint32_t fresh_generation =
      NextGeneration(validation.ok() ? 2U : 1U);
  CheckAdmission(mailbox.Publish(fresh), OkResult(), false, fresh_generation);
  Require(mailbox.ConsumeLatest(&snapshot));
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    const bool selected = index == 3U;
    Require(snapshot.pulse_width_us[index] == (selected ? 1600U : 1500U));
    Require(snapshot.duration_ms[index] == (selected ? 40U : 0U));
    Require(snapshot.field_generation[index] ==
            (selected ? fresh_generation : 0U));
  }
}

BusServoCommand ReadMove(FuzzInput* input) {
  BusServoCommand command{};
  command.count = input->ReadU8();
  for (std::uint8_t& id : command.servo_id) {
    id = input->ReadU8();
  }
  for (std::uint16_t& position : command.position) {
    position = input->ReadU16();
  }
  command.duration_ms = input->ReadU16();
  return command;
}

void CheckBusMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const BusServoCommand command = ReadMove(&input);
  const Result validation = ValidateBusServoCommand(command);

  BusMotionMailbox mailbox;
  BusServoCommand baseline{};
  baseline.count = 1U;
  baseline.servo_id[0] = 1U;
  baseline.position[0] = 500U;
  baseline.duration_ms = 20U;
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);
  Require(mailbox.overwrite_count() == (validation.ok() ? 1U : 0U));

  BusMotionSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.generation == (validation.ok() ? 2U : 1U));
  Require(Equal(snapshot.command, validation.ok() ? command : baseline));
}

void CheckLedMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const LedCommand command{input.ReadU8(), input.ReadU16(), input.ReadU16(),
                           input.ReadU16()};
  const Result validation = ValidateLedCommand(command);

  LedCommandMailbox mailbox;
  const LedCommand baseline{1U, 10U, 20U, 30U};
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);

  LedCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(Equal(snapshot.commands[0],
                validation.ok() && command.led_id == 1U ? command : baseline));
  for (std::size_t index = 1U; index < kLedCount; ++index) {
    const bool replaced =
        validation.ok() &&
        command.led_id == static_cast<std::uint8_t>(index + 1U);
    Require(Equal(snapshot.commands[index],
                  replaced ? command
                           : LedCommand{static_cast<std::uint8_t>(index + 1U),
                                        0U, 0U, 0U}));
  }

  mailbox.ResetMergedFields();
  const LedCommand fresh{2U, 1U, 2U, 3U};
  const std::uint32_t fresh_generation =
      NextGeneration(validation.ok() ? 2U : 1U);
  CheckAdmission(mailbox.Publish(fresh), OkResult(), false, fresh_generation);
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.field_generation[0] == 0U);
  Require(snapshot.field_generation[1] == fresh_generation);
  Require(snapshot.field_generation[2] == 0U);
  Require(Equal(snapshot.commands[1], fresh));
}

void CheckBuzzerMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const BuzzerCommand command{input.ReadU16(), input.ReadU16(), input.ReadU16(),
                              input.ReadU16()};
  const Result validation = ValidateBuzzerCommand(command);

  BuzzerCommandMailbox mailbox;
  const BuzzerCommand baseline{1000U, 10U, 20U, 30U};
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);

  BuzzerCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.generation == (validation.ok() ? 2U : 1U));
  Require(Equal(snapshot.command, validation.ok() ? command : baseline));
}

void CheckRgbMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  RgbCommand command{};
  command.update_mask = input.ReadU8();
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    command.red[pixel] = input.ReadU8();
    command.green[pixel] = input.ReadU8();
    command.blue[pixel] = input.ReadU8();
  }
  const Result validation = ValidateRgbCommand(command);

  RgbCommandMailbox mailbox;
  const RgbCommand baseline{0x03U, {1U, 2U}, {3U, 4U}, {5U, 6U}};
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);

  RgbCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    const auto bit = static_cast<std::uint8_t>(1U << pixel);
    const bool replaced = validation.ok() && (command.update_mask & bit) != 0U;
    Require(snapshot.state.red[pixel] ==
            (replaced ? command.red[pixel] : baseline.red[pixel]));
    Require(snapshot.state.green[pixel] ==
            (replaced ? command.green[pixel] : baseline.green[pixel]));
    Require(snapshot.state.blue[pixel] ==
            (replaced ? command.blue[pixel] : baseline.blue[pixel]));
  }

  mailbox.ResetMergedFields();
  const RgbCommand fresh{0x02U, {0U, 7U}, {0U, 8U}, {0U, 9U}};
  const std::uint32_t fresh_generation =
      NextGeneration(validation.ok() ? 2U : 1U);
  CheckAdmission(mailbox.Publish(fresh), OkResult(), false, fresh_generation);
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.field_generation[0] == 0U);
  Require(snapshot.field_generation[1] == fresh_generation);
  Require(snapshot.state.red[0] == 0U && snapshot.state.red[1] == 7U);
}

void CheckOledMailbox(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  OledCommand command{};
  command.update_mask = input.ReadU8();
  for (BoundedText& line : command.lines) {
    line = ReadText(&input);
  }
  const Result validation = ValidateOledCommand(command);

  OledCommand baseline{};
  baseline.update_mask = 0x03U;
  baseline.lines[0].bytes[0] = 'A';
  baseline.lines[0].size = 1U;
  baseline.lines[1].bytes[0] = 'B';
  baseline.lines[1].size = 1U;
  OledCommandMailbox mailbox;
  CheckAdmission(mailbox.Publish(baseline), OkResult(), false, 1U);
  const CommandAdmission admission = mailbox.Publish(command);
  CheckAdmission(admission, validation, validation.ok(),
                 validation.ok() ? 2U : 1U);

  OledCommandSnapshot snapshot{};
  Require(mailbox.ConsumeLatest(&snapshot));
  for (std::size_t line = 0U; line < kOledHostLineCount; ++line) {
    const auto bit = static_cast<std::uint8_t>(1U << line);
    const bool replaced = validation.ok() && (command.update_mask & bit) != 0U;
    Require(Equal(snapshot.state.lines[line],
                  replaced ? command.lines[line] : baseline.lines[line]));
  }

  mailbox.ResetMergedFields();
  OledCommand fresh{};
  fresh.update_mask = 0x02U;
  fresh.lines[1].bytes[0] = 'Z';
  fresh.lines[1].size = 1U;
  const std::uint32_t fresh_generation =
      NextGeneration(validation.ok() ? 2U : 1U);
  CheckAdmission(mailbox.Publish(fresh), OkResult(), false, fresh_generation);
  Require(mailbox.ConsumeLatest(&snapshot));
  Require(snapshot.field_generation[0] == 0U);
  Require(snapshot.field_generation[1] == fresh_generation);
  Require(snapshot.state.lines[0].size == 0U);
  Require(Equal(snapshot.state.lines[1], fresh.lines[1]));
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::CheckMotorMailbox(data, size);
  mentor_pi::mcu::CheckPwmMailbox(data, size);
  mentor_pi::mcu::CheckBusMailbox(data, size);
  mentor_pi::mcu::CheckLedMailbox(data, size);
  mentor_pi::mcu::CheckBuzzerMailbox(data, size);
  mentor_pi::mcu::CheckRgbMailbox(data, size);
  mentor_pi::mcu::CheckOledMailbox(data, size);
  return 0;
}
