// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/state_merger.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

class ByteReader {
 public:
  ByteReader(const std::uint8_t* data, std::size_t size)
      : data_(data), size_(size) {}

  std::uint8_t ReadU8() {
    if (data_ == nullptr || offset_ >= size_) {
      return 0U;
    }
    return data_[offset_++];
  }

  std::uint16_t ReadU16() {
    const std::uint16_t low = ReadU8();
    const std::uint16_t high = ReadU8();
    return static_cast<std::uint16_t>(low | (high << 8U));
  }

  std::int16_t ReadI16() {
    const std::uint16_t bits = ReadU16();
    std::int16_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::int8_t ReadI8() {
    const std::uint8_t bits = ReadU8();
    std::int8_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::uint32_t ReadU32() {
    const std::uint32_t byte0 = ReadU8();
    const std::uint32_t byte1 = ReadU8();
    const std::uint32_t byte2 = ReadU8();
    const std::uint32_t byte3 = ReadU8();
    return byte0 | (byte1 << 8U) | (byte2 << 16U) | (byte3 << 24U);
  }

  float ReadFloat() {
    const std::uint32_t bits = ReadU32();
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  void Reset() { offset_ = 0U; }

 private:
  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t offset_{0U};
};

[[noreturn]] void FailInvariant() { std::abort(); }

void Require(bool condition) {
  if (!condition) {
    FailInvariant();
  }
}

bool Equal(const RgbState& left, const RgbState& right) {
  return left.red == right.red && left.green == right.green &&
         left.blue == right.blue;
}

bool Equal(const BoundedText& left, const BoundedText& right) {
  return left.size == right.size && left.bytes == right.bytes;
}

bool Equal(const OledState& left, const OledState& right) {
  for (std::size_t line = 0U; line < kOledHostLineCount; ++line) {
    if (!Equal(left.lines[line], right.lines[line])) {
      return false;
    }
  }
  return true;
}

BoundedText ReadText(ByteReader* reader) {
  BoundedText text{};
  text.size = reader->ReadU8();
  for (char& byte : text.bytes) {
    byte = static_cast<char>(reader->ReadU8());
  }
  return text;
}

void ExerciseValidation(ByteReader* reader) {
  MotorCommand motor{};
  motor.update_mask = reader->ReadU8();
  for (float& target : motor.target_rps) {
    target = reader->ReadFloat();
  }
  const float maximum_rps = reader->ReadFloat();
  static_cast<void>(ValidateMotorCommand(motor, maximum_rps));

  reader->Reset();
  PwmServoCommand pwm{};
  pwm.update_mask = reader->ReadU8();
  pwm.duration_ms = reader->ReadU16();
  for (std::uint16_t& pulse : pwm.pulse_width_us) {
    pulse = reader->ReadU16();
  }
  static_cast<void>(ValidatePwmServoCommand(pwm));

  reader->Reset();
  PwmServoOffsetCommand offsets{};
  offsets.update_mask = reader->ReadU8();
  for (std::int16_t& offset : offsets.offset_us) {
    offset = reader->ReadI16();
  }
  static_cast<void>(ValidatePwmServoOffsets(offsets));

  reader->Reset();
  BusServoCommand bus{};
  bus.count = reader->ReadU8();
  for (std::uint8_t& id : bus.servo_id) {
    id = reader->ReadU8();
  }
  for (std::uint16_t& position : bus.position) {
    position = reader->ReadU16();
  }
  bus.duration_ms = reader->ReadU16();
  static_cast<void>(ValidateBusServoCommand(bus));

  reader->Reset();
  StopBusServosCommand stop{};
  stop.count = reader->ReadU8();
  for (std::uint8_t& id : stop.servo_id) {
    id = reader->ReadU8();
  }
  static_cast<void>(ValidateStopBusServosCommand(stop));

  reader->Reset();
  ConfigureBusServoCommand configure{};
  configure.servo_id = reader->ReadU8();
  configure.update_mask = reader->ReadU16();
  configure.new_id = reader->ReadU8();
  configure.offset = reader->ReadI8();
  configure.position_min = reader->ReadU16();
  configure.position_max = reader->ReadU16();
  configure.voltage_min_mv = reader->ReadU16();
  configure.voltage_max_mv = reader->ReadU16();
  configure.temperature_limit_c = reader->ReadU8();
  configure.torque_enabled = (reader->ReadU8() & 1U) != 0U;
  static_cast<void>(ValidateConfigureBusServoCommand(configure));

  reader->Reset();
  GetBusServoStateCommand get{};
  get.servo_id = reader->ReadU8();
  get.fields = reader->ReadU16();
  static_cast<void>(ValidateGetBusServoStateCommand(get));

  reader->Reset();
  const LedCommand led{reader->ReadU8(), reader->ReadU16(), reader->ReadU16(),
                       reader->ReadU16()};
  static_cast<void>(ValidateLedCommand(led));
  reader->Reset();
  const BuzzerCommand buzzer{reader->ReadU16(), reader->ReadU16(),
                             reader->ReadU16(), reader->ReadU16()};
  static_cast<void>(ValidateBuzzerCommand(buzzer));

  reader->Reset();
  static_cast<void>(ValidateBatteryThreshold(reader->ReadU16()));
  reader->Reset();
  static_cast<void>(
      IsValidMotorModel(static_cast<MotorModel>(reader->ReadU8())));
}

void ExerciseRgbMerge(ByteReader* reader) {
  RgbState initial{};
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    initial.red[pixel] = reader->ReadU8();
    initial.green[pixel] = reader->ReadU8();
    initial.blue[pixel] = reader->ReadU8();
  }
  RgbCommand command{};
  command.update_mask = reader->ReadU8();
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    command.red[pixel] = reader->ReadU8();
    command.green[pixel] = reader->ReadU8();
    command.blue[pixel] = reader->ReadU8();
  }

  RgbState merged = initial;
  const Result validation = ValidateRgbCommand(command);
  const Result result = MergeRgbCommand(command, &merged);
  Require(result.code == validation.code);
  Require(result.detail == validation.detail);
  if (!result.ok()) {
    Require(Equal(merged, initial));
  } else {
    for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
      const auto bit = static_cast<std::uint8_t>(1U << pixel);
      if ((command.update_mask & bit) != 0U) {
        Require(merged.red[pixel] == command.red[pixel]);
        Require(merged.green[pixel] == command.green[pixel]);
        Require(merged.blue[pixel] == command.blue[pixel]);
      } else {
        Require(merged.red[pixel] == initial.red[pixel]);
        Require(merged.green[pixel] == initial.green[pixel]);
        Require(merged.blue[pixel] == initial.blue[pixel]);
      }
    }
  }
  Require(MergeRgbCommand(command, nullptr).code ==
          ResultCode::kInvalidArgument);
}

void ExerciseOledMerge(ByteReader* reader) {
  OledState initial{};
  for (BoundedText& line : initial.lines) {
    line = ReadText(reader);
  }
  OledCommand command{};
  command.update_mask = reader->ReadU8();
  for (BoundedText& line : command.lines) {
    line = ReadText(reader);
  }

  OledState merged = initial;
  const Result validation = ValidateOledCommand(command);
  const Result result = MergeOledCommand(command, &merged);
  Require(result.code == validation.code);
  Require(result.detail == validation.detail);
  if (!result.ok()) {
    Require(Equal(merged, initial));
  } else {
    for (std::size_t line = 0U; line < kOledHostLineCount; ++line) {
      const auto bit = static_cast<std::uint8_t>(1U << line);
      Require(Equal(merged.lines[line], (command.update_mask & bit) != 0U
                                            ? command.lines[line]
                                            : initial.lines[line]));
    }
  }
  Require(MergeOledCommand(command, nullptr).code ==
          ResultCode::kInvalidArgument);
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::ByteReader validation_reader(data, size);
  mentor_pi::mcu::ExerciseValidation(&validation_reader);
  mentor_pi::mcu::ByteReader rgb_reader(data, size);
  mentor_pi::mcu::ExerciseRgbMerge(&rgb_reader);
  mentor_pi::mcu::ByteReader oled_reader(data, size);
  mentor_pi::mcu::ExerciseOledMerge(&oled_reader);
  return 0;
}
