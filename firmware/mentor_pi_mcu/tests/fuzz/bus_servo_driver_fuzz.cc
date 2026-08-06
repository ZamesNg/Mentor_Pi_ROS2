// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"
#include "mentor_pi_mcu/drivers/bus_servo_uart.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi::mcu::drivers {
namespace {

using mentor_pi::mcu::fuzz::FuzzInput;
using mentor_pi::mcu::fuzz::Require;

struct FakeUartConfiguration {
  std::uint8_t begin_status{0U};
  std::uint8_t poll_status{0U};
  std::uint8_t reply_mode{0U};
  std::uint8_t reported_id{1U};
  std::uint8_t reply_size{0U};
  std::array<std::uint8_t, kBusServoMaximumFrameBytes> reply_bytes{};
};

class FakeHalfDuplexUart : public HalfDuplexUart {
 public:
  explicit FakeHalfDuplexUart(FakeUartConfiguration configuration)
      : configuration_(configuration) {}

  IoStatus BeginExchange(const std::uint8_t* tx, std::size_t tx_size,
                         std::size_t max_reply_size,
                         std::uint32_t deadline_ms) override {
    Require(tx != nullptr);
    Require(tx_size >= 6U && tx_size <= kBusServoMaximumFrameBytes);
    Require(max_reply_size <= kBusServoMaximumFrameBytes);
    Require(frame_count_ < frames_.size());
    BusServoFrame& recorded = frames_[frame_count_];
    std::memcpy(recorded.bytes.data(), tx, tx_size);
    recorded.size = static_cast<std::uint8_t>(tx_size);
    expected_reply_size_[frame_count_] = max_reply_size;
    ++frame_count_;
    current_frame_ = recorded;
    current_reply_size_ = max_reply_size;
    deadline_ms_ = deadline_ms;
    const auto status = static_cast<IoStatus>(configuration_.begin_status);
    active_ = status == IoStatus::kOk;
    return status;
  }

  IoStatus PollExchange(std::uint32_t now_ms, std::uint8_t* reply,
                        std::size_t capacity,
                        std::size_t* reply_size) override {
    Require(active_);
    Require(reply != nullptr);
    Require(reply_size != nullptr);
    Require(capacity <= kBusServoMaximumFrameBytes);
    last_poll_ms_ = now_ms;
    ++poll_count_;
    const auto status = static_cast<IoStatus>(configuration_.poll_status);
    if (status == IoStatus::kBusy) {
      return status;
    }
    active_ = false;
    if (status != IoStatus::kOk) {
      *reply_size = 0U;
      return status;
    }
    if (current_reply_size_ == 0U) {
      *reply_size = 0U;
      return IoStatus::kOk;
    }
    if ((configuration_.reply_mode & 1U) == 0U) {
      BuildValidReply(reply, capacity, reply_size);
    } else {
      const std::size_t copied = std::min(
          capacity, static_cast<std::size_t>(configuration_.reply_size));
      std::memcpy(reply, configuration_.reply_bytes.data(), copied);
      *reply_size = copied;
    }
    return IoStatus::kOk;
  }

  void Cancel() override {
    active_ = false;
    ++cancel_count_;
  }

  void CheckRecordedFrames() const {
    for (std::size_t index = 0U; index < frame_count_; ++index) {
      const ParsedBusServoFrame parsed = BusServoCodec::ParseFrame(
          frames_[index].bytes.data(), frames_[index].size);
      Require(parsed.result.ok());
      Require(expected_reply_size_[index] == 0U ||
              expected_reply_size_[index] >= 6U);
      Require(expected_reply_size_[index] <= kBusServoMaximumFrameBytes);
    }
  }

  std::size_t frame_count() const { return frame_count_; }
  std::size_t poll_count() const { return poll_count_; }
  std::size_t cancel_count() const { return cancel_count_; }
  std::uint32_t deadline_ms() const { return deadline_ms_; }
  std::uint32_t last_poll_ms() const { return last_poll_ms_; }

 private:
  void BuildValidReply(std::uint8_t* reply, std::size_t capacity,
                       std::size_t* reply_size) const {
    const ParsedBusServoFrame request = BusServoCodec::ParseFrame(
        current_frame_.bytes.data(), current_frame_.size);
    Require(request.result.ok());
    Require(current_reply_size_ >= 6U);
    const std::size_t argument_count = current_reply_size_ - 6U;
    Require(argument_count <= kBusServoMaximumArguments);
    std::array<std::uint8_t, kBusServoMaximumArguments> arguments{};
    for (std::size_t index = 0U; index < argument_count; ++index) {
      arguments[index] = static_cast<std::uint8_t>(
          configuration_.reply_bytes[index] + static_cast<std::uint8_t>(index));
    }
    const std::uint8_t response_id =
        request.servo_id == 254U
            ? static_cast<std::uint8_t>(
                  (static_cast<std::uint16_t>(configuration_.reported_id) %
                   253U) +
                  1U)
            : request.servo_id;
    if (request.opcode == BusServoOpcode::kIdRead && argument_count > 0U) {
      arguments[0] = response_id;
    }
    BusServoFrame response{};
    Require(BusServoCodec::BuildFrame(response_id, request.opcode,
                                      arguments.data(), argument_count,
                                      &response)
                .ok());
    Require(response.size == current_reply_size_);
    Require(response.size <= capacity);
    std::memcpy(reply, response.bytes.data(), response.size);
    *reply_size = response.size;
  }

  static constexpr std::size_t kMaximumRecordedFrames = 64U;
  FakeUartConfiguration configuration_{};
  std::array<BusServoFrame, kMaximumRecordedFrames> frames_{};
  std::array<std::size_t, kMaximumRecordedFrames> expected_reply_size_{};
  BusServoFrame current_frame_{};
  std::size_t current_reply_size_{0U};
  std::size_t frame_count_{0U};
  std::size_t poll_count_{0U};
  std::size_t cancel_count_{0U};
  std::uint32_t deadline_ms_{0U};
  std::uint32_t last_poll_ms_{0U};
  bool active_{false};
};

FakeUartConfiguration ReadUartConfiguration(const std::uint8_t* data,
                                            std::size_t size) {
  FuzzInput input(data, size);
  FakeUartConfiguration configuration{};
  configuration.begin_status = input.ReadU8();
  configuration.poll_status = input.ReadU8();
  configuration.reply_mode = input.ReadU8();
  configuration.reported_id = input.ReadU8();
  configuration.reply_size = static_cast<std::uint8_t>(
      input.ReadU8() % (kBusServoMaximumFrameBytes + 1U));
  for (std::uint8_t& byte : configuration.reply_bytes) {
    byte = input.ReadU8();
  }
  return configuration;
}

std::uint32_t PollTime(std::uint32_t start_ms, std::uint8_t mode,
                       std::size_t iteration) {
  switch (mode & 3U) {
    case 0U:
      return start_ms + static_cast<std::uint32_t>(iteration);
    case 1U:
      return start_ms + 199U;
    case 2U:
      return start_ms + 200U;
    case 3U:
      return start_ms - 1U + static_cast<std::uint32_t>(iteration);
    default:
      return start_ms;
  }
}

BusServoCommand ReadMove(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  BusServoCommand command{};
  command.count = input.ReadU8();
  for (std::uint8_t& id : command.servo_id) {
    id = input.ReadU8();
  }
  for (std::uint16_t& position : command.position) {
    position = input.ReadU16();
  }
  command.duration_ms = input.ReadU16();
  return command;
}

StopBusServosCommand ReadStop(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  StopBusServosCommand command{};
  command.count = input.ReadU8();
  for (std::uint8_t& id : command.servo_id) {
    id = input.ReadU8();
  }
  return command;
}

GetBusServoStateCommand ReadGetState(const std::uint8_t* data,
                                     std::size_t size) {
  FuzzInput input(data, size);
  return {input.ReadU8(), input.ReadU16()};
}

ConfigureBusServoCommand ReadConfigure(const std::uint8_t* data,
                                       std::size_t size) {
  FuzzInput input(data, size);
  ConfigureBusServoCommand command{};
  command.servo_id = input.ReadU8();
  command.update_mask = input.ReadU16();
  command.new_id = input.ReadU8();
  command.offset = input.ReadI8();
  command.position_min = input.ReadU16();
  command.position_max = input.ReadU16();
  command.voltage_min_mv = input.ReadU16();
  command.voltage_max_mv = input.ReadU16();
  command.temperature_limit_c = input.ReadU8();
  command.torque_enabled = input.ReadU8() != 0U;
  return command;
}

Result Start(BusServoUartDriver* driver, const BusServoCommand& command,
             std::uint32_t now_ms) {
  return driver->StartMove(command, now_ms);
}

Result Start(BusServoUartDriver* driver, const StopBusServosCommand& command,
             std::uint32_t now_ms) {
  return driver->StartStop(command, now_ms);
}

Result Start(BusServoUartDriver* driver, const GetBusServoStateCommand& command,
             std::uint32_t now_ms) {
  return driver->StartQuery(command, now_ms);
}

Result Start(BusServoUartDriver* driver,
             const ConfigureBusServoCommand& command, std::uint32_t now_ms) {
  return driver->StartConfigure(command, now_ms);
}

Result Validate(const BusServoCommand& command) {
  return ValidateBusServoCommand(command);
}

Result Validate(const StopBusServosCommand& command) {
  return ValidateStopBusServosCommand(command);
}

Result Validate(const GetBusServoStateCommand& command) {
  return ValidateGetBusServoStateCommand(command);
}

Result Validate(const ConfigureBusServoCommand& command) {
  return ValidateConfigureBusServoCommand(command);
}

bool Equal(Result left, Result right) {
  return left.code == right.code && left.detail == right.detail;
}

template <typename Command>
void ExerciseOperation(const Command& command,
                       const FakeUartConfiguration& uart_configuration,
                       std::uint32_t start_ms, std::uint8_t time_mode) {
  FakeHalfDuplexUart uart(uart_configuration);
  BusServoUartDriver driver(uart);
  const Result validation = Validate(command);
  const Result start = Start(&driver, command, start_ms);
  if (!validation.ok()) {
    Require(Equal(start, validation));
    Require(uart.frame_count() == 0U);
    Require(uart.poll_count() == 0U);
    Require(!driver.busy());
    return;
  }

  Require(uart.frame_count() >= 1U);
  uart.CheckRecordedFrames();
  if (driver.busy()) {
    const std::size_t frames_before_busy_retry = uart.frame_count();
    const Result collision = Start(&driver, command, start_ms);
    Require(collision.code == ResultCode::kBusy);
    Require(uart.frame_count() == frames_before_busy_retry);
  }
  for (std::size_t iteration = 0U; iteration < 32U && driver.busy();
       ++iteration) {
    static_cast<void>(driver.Poll(PollTime(start_ms, time_mode, iteration)));
  }
  uart.CheckRecordedFrames();
  driver.Cancel();
  Require(!driver.busy());
  Require(uart.frame_count() <= 33U);
  Require(uart.poll_count() <= 32U);
  Require(uart.cancel_count() <= 2U);
  static_cast<void>(uart.deadline_ms());
  static_cast<void>(uart.last_poll_ms());
}

void ExerciseArbitraryCommands(const std::uint8_t* data, std::size_t size,
                               const FakeUartConfiguration& configuration,
                               std::uint32_t start_ms, std::uint8_t time_mode) {
  ExerciseOperation(ReadMove(data, size), configuration, start_ms, time_mode);
  ExerciseOperation(ReadStop(data, size), configuration, start_ms, time_mode);
  ExerciseOperation(ReadGetState(data, size), configuration, start_ms,
                    time_mode);
  ExerciseOperation(ReadConfigure(data, size), configuration, start_ms,
                    time_mode);
}

void ExerciseMaximumValidCommands(const FakeUartConfiguration& configuration,
                                  std::uint32_t start_ms,
                                  std::uint8_t time_mode) {
  BusServoCommand move{};
  StopBusServosCommand stop{};
  move.count = static_cast<std::uint8_t>(kBusServoBatchCapacity);
  stop.count = static_cast<std::uint8_t>(kBusServoBatchCapacity);
  for (std::size_t index = 0U; index < kBusServoBatchCapacity; ++index) {
    move.servo_id[index] = static_cast<std::uint8_t>(index + 1U);
    move.position[index] = static_cast<std::uint16_t>(index * 50U);
    stop.servo_id[index] = static_cast<std::uint8_t>(index + 1U);
  }
  move.duration_ms = 30000U;
  ExerciseOperation(move, configuration, start_ms, time_mode);
  ExerciseOperation(stop, configuration, start_ms, time_mode);

  const GetBusServoStateCommand get{1U, GetBusServoStateCommand::kAllFields};
  ExerciseOperation(get, configuration, start_ms, time_mode);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.update_mask = ConfigureBusServoCommand::kAllUpdates;
  configure.new_id = 253U;
  configure.offset = 125;
  configure.position_min = 0U;
  configure.position_max = 1000U;
  configure.voltage_min_mv = 4500U;
  configure.voltage_max_mv = 14000U;
  configure.temperature_limit_c = 100U;
  configure.torque_enabled = true;
  ExerciseOperation(configure, configuration, start_ms, time_mode);
}

}  // namespace
}  // namespace mentor_pi::mcu::drivers

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::fuzz::FuzzInput input(data, size);
  const auto configuration =
      mentor_pi::mcu::drivers::ReadUartConfiguration(data, size);
  const std::uint32_t start_ms = input.ReadU32();
  const std::uint8_t time_mode = input.ReadU8();
  mentor_pi::mcu::drivers::ExerciseArbitraryCommands(data, size, configuration,
                                                     start_ms, time_mode);
  mentor_pi::mcu::drivers::ExerciseMaximumValidCommands(configuration, start_ms,
                                                        time_mode);
  return 0;
}
