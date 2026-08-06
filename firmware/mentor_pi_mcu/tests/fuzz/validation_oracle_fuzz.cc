// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

using fuzz::FuzzInput;
using fuzz::Require;

Result ExpectedMask(std::uint16_t mask, std::uint16_t allowed) {
  const std::uint16_t invalid =
      static_cast<std::uint16_t>(mask & static_cast<std::uint16_t>(~allowed));
  return mask == 0U || invalid != 0U
             ? Result{ResultCode::kInvalidArgument, invalid}
             : OkResult();
}

bool IsServoId(std::uint8_t id) { return id >= 1U && id <= 253U; }

template <typename Command>
Result ExpectedIdList(const Command& command) {
  if (command.count == 0U || command.count > kBusServoBatchCapacity) {
    return {ResultCode::kInvalidArgument, command.count};
  }
  for (std::size_t index = 0U; index < command.count; ++index) {
    if (!IsServoId(command.servo_id[index])) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
    for (std::size_t prior = 0U; prior < index; ++prior) {
      if (command.servo_id[prior] == command.servo_id[index]) {
        return {ResultCode::kInvalidArgument,
                static_cast<std::uint16_t>(index + 1U)};
      }
    }
  }
  return OkResult();
}

Result ExpectedMotor(const MotorCommand& command, float maximum_rps) {
  const Result mask = ExpectedMask(command.update_mask, kAllMotorMask);
  if (!mask.ok()) {
    return mask;
  }
  if (!std::isfinite(maximum_rps) || maximum_rps <= 0.0F) {
    return {ResultCode::kInvalidArgument, 0U};
  }
  for (std::size_t index = 0U; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    const float target = command.target_rps[index];
    if ((command.update_mask & bit) != 0U &&
        (!std::isfinite(target) || std::fabs(target) > maximum_rps)) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ExpectedPwm(const PwmServoCommand& command) {
  const Result mask = ExpectedMask(command.update_mask, kAllPwmServoMask);
  if (!mask.ok()) {
    return mask;
  }
  if (command.duration_ms < 20U || command.duration_ms > 30000U) {
    return {ResultCode::kOutOfRange, 0U};
  }
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        (command.pulse_width_us[index] < 500U ||
         command.pulse_width_us[index] > 2500U)) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ExpectedOffsets(const PwmServoOffsetCommand& command) {
  const Result mask = ExpectedMask(command.update_mask, kAllPwmServoMask);
  if (!mask.ok()) {
    return mask;
  }
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        (command.offset_us[index] < -100 || command.offset_us[index] > 100)) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ExpectedMove(const BusServoCommand& command) {
  const Result ids = ExpectedIdList(command);
  if (!ids.ok()) {
    return ids;
  }
  if (command.duration_ms > 30000U) {
    return {ResultCode::kOutOfRange, 0U};
  }
  for (std::size_t index = 0U; index < command.count; ++index) {
    if (command.position[index] > 1000U) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ExpectedConfigure(const ConfigureBusServoCommand& command) {
  if (!IsServoId(command.servo_id)) {
    return {ResultCode::kOutOfRange, 0U};
  }
  const Result mask =
      ExpectedMask(command.update_mask, ConfigureBusServoCommand::kAllUpdates);
  if (!mask.ok()) {
    return mask;
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetId) != 0U &&
      !IsServoId(command.new_id)) {
    return {ResultCode::kOutOfRange, 1U};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetOffset) != 0U &&
      (command.offset < -125 || command.offset > 125)) {
    return {ResultCode::kOutOfRange, 2U};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetPositionLimits) !=
          0U &&
      (command.position_min > 1000U || command.position_max > 1000U ||
       command.position_min > command.position_max)) {
    return {command.position_min > command.position_max
                ? ResultCode::kInvalidArgument
                : ResultCode::kOutOfRange,
            4U};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetVoltageLimits) !=
          0U &&
      (command.voltage_min_mv < 4500U || command.voltage_min_mv > 14000U ||
       command.voltage_max_mv < 4500U || command.voltage_max_mv > 14000U ||
       command.voltage_min_mv > command.voltage_max_mv)) {
    return {command.voltage_min_mv > command.voltage_max_mv
                ? ResultCode::kInvalidArgument
                : ResultCode::kOutOfRange,
            5U};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetTemperatureLimit) !=
          0U &&
      command.temperature_limit_c > 100U) {
    return {ResultCode::kOutOfRange, 6U};
  }
  return OkResult();
}

Result ExpectedGetState(const GetBusServoStateCommand& command) {
  const Result fields =
      ExpectedMask(command.fields, GetBusServoStateCommand::kAllFields);
  if (!fields.ok()) {
    return fields;
  }
  if (command.servo_id == 254U) {
    return command.fields == GetBusServoStateCommand::kFieldId
               ? OkResult()
               : Result{ResultCode::kInvalidArgument, command.fields};
  }
  return IsServoId(command.servo_id) ? OkResult()
                                     : Result{ResultCode::kOutOfRange, 0U};
}

bool Equal(Result left, Result right) {
  return left.code == right.code && left.detail == right.detail;
}

BoundedText ReadText(FuzzInput* input) {
  BoundedText text{};
  text.size = input->ReadU8();
  for (char& byte : text.bytes) {
    byte = static_cast<char>(input->ReadU8());
  }
  return text;
}

void CheckMotor(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  MotorCommand command{};
  command.update_mask = input.ReadU8();
  for (float& target : command.target_rps) {
    target = input.ReadBiasedFloat();
  }
  const float maximum_rps = input.ReadBiasedFloat();
  Require(Equal(ValidateMotorCommand(command, maximum_rps),
                ExpectedMotor(command, maximum_rps)));
}

void CheckPwm(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  PwmServoCommand command{};
  command.update_mask = input.ReadU8();
  command.duration_ms = input.ReadU16();
  for (std::uint16_t& pulse : command.pulse_width_us) {
    pulse = input.ReadU16();
  }
  Require(Equal(ValidatePwmServoCommand(command), ExpectedPwm(command)));

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = input.ReadU8();
  for (std::int16_t& offset : offsets.offset_us) {
    offset = input.ReadI16();
  }
  Require(Equal(ValidatePwmServoOffsets(offsets), ExpectedOffsets(offsets)));
}

void CheckBus(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  BusServoCommand move{};
  move.count = input.ReadU8();
  for (std::uint8_t& id : move.servo_id) {
    id = input.ReadU8();
  }
  for (std::uint16_t& position : move.position) {
    position = input.ReadU16();
  }
  move.duration_ms = input.ReadU16();
  Require(Equal(ValidateBusServoCommand(move), ExpectedMove(move)));

  StopBusServosCommand stop{};
  stop.count = input.ReadU8();
  for (std::uint8_t& id : stop.servo_id) {
    id = input.ReadU8();
  }
  Require(Equal(ValidateStopBusServosCommand(stop), ExpectedIdList(stop)));

  ConfigureBusServoCommand configure{};
  configure.servo_id = input.ReadU8();
  configure.update_mask = input.ReadU16();
  configure.new_id = input.ReadU8();
  configure.offset = input.ReadI8();
  configure.position_min = input.ReadU16();
  configure.position_max = input.ReadU16();
  configure.voltage_min_mv = input.ReadU16();
  configure.voltage_max_mv = input.ReadU16();
  configure.temperature_limit_c = input.ReadU8();
  configure.torque_enabled = input.ReadU8() != 0U;
  Require(Equal(ValidateConfigureBusServoCommand(configure),
                ExpectedConfigure(configure)));

  const GetBusServoStateCommand get{input.ReadU8(), input.ReadU16()};
  Require(Equal(ValidateGetBusServoStateCommand(get), ExpectedGetState(get)));
}

void CheckPeripherals(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  const LedCommand led{input.ReadU8(), input.ReadU16(), input.ReadU16(),
                       input.ReadU16()};
  const Result expected_led = led.led_id >= 1U && led.led_id <= kLedCount
                                  ? OkResult()
                                  : Result{ResultCode::kOutOfRange, led.led_id};
  Require(Equal(ValidateLedCommand(led), expected_led));

  const BuzzerCommand buzzer{input.ReadU16(), input.ReadU16(), input.ReadU16(),
                             input.ReadU16()};
  const bool forced_off = buzzer.frequency_hz == 0U || buzzer.on_time_ms == 0U;
  const Result expected_buzzer = forced_off || (buzzer.frequency_hz >= 10U &&
                                                buzzer.frequency_hz <= 20000U)
                                     ? OkResult()
                                     : Result{ResultCode::kOutOfRange, 0U};
  Require(Equal(ValidateBuzzerCommand(buzzer), expected_buzzer));

  RgbCommand rgb{};
  rgb.update_mask = input.ReadU8();
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    rgb.red[pixel] = input.ReadU8();
    rgb.green[pixel] = input.ReadU8();
    rgb.blue[pixel] = input.ReadU8();
  }
  Require(Equal(ValidateRgbCommand(rgb),
                ExpectedMask(rgb.update_mask, kAllRgbPixelMask)));

  OledCommand oled{};
  oled.update_mask = input.ReadU8();
  for (BoundedText& line : oled.lines) {
    line = ReadText(&input);
  }
  Result expected_oled = ExpectedMask(oled.update_mask, kAllOledLineMask);
  if (expected_oled.ok()) {
    for (std::size_t line = 0U; line < kOledHostLineCount; ++line) {
      const auto bit = static_cast<std::uint8_t>(1U << line);
      if ((oled.update_mask & bit) == 0U) {
        continue;
      }
      const BoundedText& text = oled.lines[line];
      bool valid = text.size <= kOledLineCapacity;
      if (valid) {
        valid = text.bytes[text.size] == '\0';
      }
      for (std::size_t index = 0U; valid && index < text.size; ++index) {
        const auto byte = static_cast<unsigned char>(text.bytes[index]);
        valid = byte >= 0x20U && byte <= 0x7eU;
      }
      if (!valid) {
        expected_oled = {ResultCode::kInvalidArgument,
                         static_cast<std::uint16_t>(line + 1U)};
        break;
      }
    }
  }
  Require(Equal(ValidateOledCommand(oled), expected_oled));

  const std::uint16_t threshold_mv = input.ReadU16();
  const Result expected_threshold =
      threshold_mv >= 5000U && threshold_mv <= 20000U
          ? OkResult()
          : Result{ResultCode::kOutOfRange, 0U};
  Require(Equal(ValidateBatteryThreshold(threshold_mv), expected_threshold));

  const std::uint8_t model = input.ReadU8();
  Require(IsValidMotorModel(static_cast<MotorModel>(model)) == (model <= 3U));
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::CheckMotor(data, size);
  mentor_pi::mcu::CheckPwm(data, size);
  mentor_pi::mcu::CheckBus(data, size);
  mentor_pi::mcu::CheckPeripherals(data, size);
  return 0;
}
