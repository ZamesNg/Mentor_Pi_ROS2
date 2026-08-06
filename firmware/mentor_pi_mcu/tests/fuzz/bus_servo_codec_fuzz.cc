// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "mentor_pi_mcu/domain/bus_servo.h"

namespace mentor_pi::mcu {
namespace {

std::uint8_t ByteAt(const std::uint8_t* data, std::size_t size,
                    std::size_t index) {
  return data != nullptr && index < size ? data[index] : 0U;
}

[[noreturn]] void FailInvariant() { std::abort(); }

void Require(bool condition) {
  if (!condition) {
    FailInvariant();
  }
}

void CheckRoundTrip(std::uint8_t servo_id, BusServoOpcode opcode,
                    const std::uint8_t* arguments, std::size_t argument_count) {
  BusServoFrame frame{};
  const Result built = BusServoCodec::BuildFrame(servo_id, opcode, arguments,
                                                 argument_count, &frame);
  Require(built.ok());
  const ParsedBusServoFrame parsed =
      BusServoCodec::ParseFrame(frame.bytes.data(), frame.size);
  Require(parsed.result.ok());
  Require(parsed.servo_id == servo_id);
  Require(parsed.opcode == opcode);
  Require(parsed.argument_count == argument_count);
  Require(std::memcmp(parsed.arguments.data(), arguments, argument_count) == 0);
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  using mentor_pi::mcu::BusServoCodec;
  using mentor_pi::mcu::BusServoFrame;
  using mentor_pi::mcu::BusServoOpcode;
  using mentor_pi::mcu::kBusServoMaximumArguments;
  using mentor_pi::mcu::ParsedBusServoFrame;
  using mentor_pi::mcu::Require;
  using mentor_pi::mcu::Result;

  const ParsedBusServoFrame arbitrary = BusServoCodec::ParseFrame(data, size);
  static_cast<void>(BusServoCodec::Checksum(data, size));
  if (arbitrary.result.ok()) {
    BusServoFrame rebuilt{};
    const Result result = BusServoCodec::BuildFrame(
        arbitrary.servo_id, arbitrary.opcode, arbitrary.arguments.data(),
        arbitrary.argument_count, &rebuilt);
    Require(result.ok());
    Require(rebuilt.size == size);
    Require(std::memcmp(rebuilt.bytes.data(), data, size) == 0);
  }

  std::array<std::uint8_t, kBusServoMaximumArguments> arguments{};
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    arguments[index] = mentor_pi::mcu::ByteAt(data, size, index + 3U);
  }
  const std::uint8_t servo_id = static_cast<std::uint8_t>(
      (mentor_pi::mcu::ByteAt(data, size, 0U) % 254U) + 1U);
  const auto opcode =
      static_cast<BusServoOpcode>(mentor_pi::mcu::ByteAt(data, size, 1U));
  const std::size_t argument_count =
      mentor_pi::mcu::ByteAt(data, size, 2U) % (kBusServoMaximumArguments + 1U);
  mentor_pi::mcu::CheckRoundTrip(servo_id, opcode, arguments.data(),
                                 argument_count);

  BusServoFrame malformed_output{};
  const std::size_t malformed_count =
      mentor_pi::mcu::ByteAt(data, size, 2U) % (kBusServoMaximumArguments + 2U);
  const std::uint8_t raw_id = mentor_pi::mcu::ByteAt(data, size, 0U);
  const bool null_arguments =
      (mentor_pi::mcu::ByteAt(data, size, 1U) & 1U) != 0U;
  const bool null_output = (mentor_pi::mcu::ByteAt(data, size, 1U) & 2U) != 0U;
  static_cast<void>(BusServoCodec::BuildFrame(
      raw_id, opcode, null_arguments ? nullptr : arguments.data(),
      malformed_count, null_output ? nullptr : &malformed_output));
  return 0;
}
