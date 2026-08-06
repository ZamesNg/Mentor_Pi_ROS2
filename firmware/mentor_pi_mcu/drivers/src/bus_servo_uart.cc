#include "mentor_pi_mcu/drivers/bus_servo_uart.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu::drivers {
namespace {

std::array<std::uint8_t, 2> EncodeU16(std::uint16_t value) {
  return {static_cast<std::uint8_t>(value & 0xffU),
          static_cast<std::uint8_t>(value >> 8U)};
}

std::uint16_t DecodeU16(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::int16_t DecodeI16(const std::uint8_t* bytes) {
  const std::uint16_t value = DecodeU16(bytes);
  if (value <= 0x7fffU) {
    return static_cast<std::int16_t>(value);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(value) - 0x10000);
}

std::int8_t DecodeI8(std::uint8_t value) {
  if (value <= 0x7fU) {
    return static_cast<std::int8_t>(value);
  }
  return static_cast<std::int8_t>(static_cast<std::int16_t>(value) - 0x100);
}

bool DeadlineReached(std::uint32_t now, std::uint32_t deadline) {
  return now - deadline < 0x80000000U;
}

Result IoResult(IoStatus status) {
  switch (status) {
    case IoStatus::kOk:
      return OkResult();
    case IoStatus::kBusy:
      return {ResultCode::kBusy, 0};
    case IoStatus::kTimeout:
      return {ResultCode::kTimeout, 0};
    case IoStatus::kIoError:
      return {ResultCode::kIoError, 0};
  }
  return {ResultCode::kIoError, 0};
}

}  // namespace

Result BusServoUartDriver::StartMove(const BusServoCommand& command,
                                     std::uint32_t now_ms) {
  const Result validation = ValidateBusServoCommand(command);
  if (!validation.ok()) {
    return validation;
  }
  Result result = BeginOperation(Operation::kMove, now_ms);
  if (!result.ok()) {
    return result;
  }
  for (std::size_t index = 0; index < command.count; ++index) {
    const auto position = EncodeU16(command.position[index]);
    const auto duration = EncodeU16(command.duration_ms);
    const std::array<std::uint8_t, 4> arguments{position[0], position[1],
                                                duration[0], duration[1]};
    result = AddStep(command.servo_id[index], BusServoOpcode::kMoveTimeWrite,
                     arguments.data(), arguments.size(), false, 0U,
                     static_cast<std::uint16_t>(1U << index));
    if (!result.ok()) {
      Finish(result);
      return result;
    }
  }
  result = StartCurrentExchange();
  if (result.code == ResultCode::kBusy) {
    return OkResult();
  }
  if (!result.ok()) {
    Finish(result);
  }
  return result;
}

Result BusServoUartDriver::StartStop(const StopBusServosCommand& command,
                                     std::uint32_t now_ms) {
  const Result validation = ValidateStopBusServosCommand(command);
  if (!validation.ok()) {
    return validation;
  }
  Result result = BeginOperation(Operation::kStop, now_ms);
  if (!result.ok()) {
    return result;
  }
  for (std::size_t index = 0; index < command.count; ++index) {
    result =
        AddStep(command.servo_id[index], BusServoOpcode::kMoveStop, nullptr, 0U,
                false, 0U, static_cast<std::uint16_t>(1U << index));
    if (!result.ok()) {
      Finish(result);
      return result;
    }
  }
  result = StartCurrentExchange();
  if (result.code == ResultCode::kBusy) {
    return OkResult();
  }
  if (!result.ok()) {
    Finish(result);
  }
  return result;
}

Result BusServoUartDriver::StartQuery(const GetBusServoStateCommand& command,
                                      std::uint32_t now_ms) {
  const Result validation = ValidateGetBusServoStateCommand(command);
  if (!validation.ok()) {
    return validation;
  }
  Result result = BeginOperation(Operation::kQuery, now_ms);
  if (!result.ok()) {
    return result;
  }
  state_.requested_id = command.servo_id;
  struct QueryDefinition {
    std::uint16_t field;
    BusServoOpcode opcode;
    std::uint8_t arguments;
  };
  constexpr std::array<QueryDefinition, 9> kQueries{{
      {BusServoState::kFieldId, BusServoOpcode::kIdRead, 1U},
      {BusServoState::kFieldPosition, BusServoOpcode::kPositionRead, 2U},
      {BusServoState::kFieldOffset, BusServoOpcode::kOffsetRead, 1U},
      {BusServoState::kFieldVoltage, BusServoOpcode::kVoltageRead, 2U},
      {BusServoState::kFieldTemperature, BusServoOpcode::kTemperatureRead, 1U},
      {BusServoState::kFieldPositionLimits, BusServoOpcode::kPositionLimitsRead,
       4U},
      {BusServoState::kFieldVoltageLimits, BusServoOpcode::kVoltageLimitsRead,
       4U},
      {BusServoState::kFieldTemperatureLimit,
       BusServoOpcode::kTemperatureLimitRead, 1U},
      {BusServoState::kFieldTorque, BusServoOpcode::kTorqueRead, 1U},
  }};
  for (const QueryDefinition& query : kQueries) {
    if ((command.fields & query.field) == 0U) {
      continue;
    }
    result = AddStep(command.servo_id, query.opcode, nullptr, 0U, true,
                     query.arguments, query.field);
    if (!result.ok()) {
      Finish(result);
      return result;
    }
  }
  result = StartCurrentExchange();
  if (result.code == ResultCode::kBusy) {
    return OkResult();
  }
  if (!result.ok()) {
    Finish(result);
  }
  return result;
}

Result BusServoUartDriver::StartConfigure(
    const ConfigureBusServoCommand& command, std::uint32_t now_ms) {
  const Result validation = ValidateConfigureBusServoCommand(command);
  if (!validation.ok()) {
    return validation;
  }
  Result result = BeginOperation(Operation::kConfigure, now_ms);
  if (!result.ok()) {
    return result;
  }
  const auto add_write = [this, &command](std::uint16_t field,
                                          BusServoOpcode opcode,
                                          const std::uint8_t* arguments,
                                          std::size_t argument_count) {
    if ((command.update_mask & field) == 0U) {
      return OkResult();
    }
    return AddStep(command.servo_id, opcode, arguments, argument_count, false,
                   0U, field);
  };

  const std::uint8_t offset = static_cast<std::uint8_t>(command.offset);
  result = add_write(ConfigureBusServoCommand::kSetOffset,
                     BusServoOpcode::kOffsetAdjust, &offset, 1U);
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSaveOffset,
                       BusServoOpcode::kOffsetSave, nullptr, 0U);
  }
  const auto position_min = EncodeU16(command.position_min);
  const auto position_max = EncodeU16(command.position_max);
  const std::array<std::uint8_t, 4> position_limits{
      position_min[0], position_min[1], position_max[0], position_max[1]};
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSetPositionLimits,
                       BusServoOpcode::kPositionLimitsWrite,
                       position_limits.data(), position_limits.size());
  }
  const auto voltage_min = EncodeU16(command.voltage_min_mv);
  const auto voltage_max = EncodeU16(command.voltage_max_mv);
  const std::array<std::uint8_t, 4> voltage_limits{
      voltage_min[0], voltage_min[1], voltage_max[0], voltage_max[1]};
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSetVoltageLimits,
                       BusServoOpcode::kVoltageLimitsWrite,
                       voltage_limits.data(), voltage_limits.size());
  }
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSetTemperatureLimit,
                       BusServoOpcode::kTemperatureLimitWrite,
                       &command.temperature_limit_c, 1U);
  }
  const std::uint8_t torque = command.torque_enabled ? 1U : 0U;
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSetTorque,
                       BusServoOpcode::kTorqueWrite, &torque, 1U);
  }
  // ID change is intentionally last because subsequent frames must use the
  // original address.
  if (result.ok()) {
    result = add_write(ConfigureBusServoCommand::kSetId,
                       BusServoOpcode::kIdWrite, &command.new_id, 1U);
  }
  if (!result.ok()) {
    Finish(result);
    return result;
  }
  result = StartCurrentExchange();
  if (result.code == ResultCode::kBusy) {
    return OkResult();
  }
  if (!result.ok()) {
    Finish(result);
  }
  return result;
}

BusServoPollResult BusServoUartDriver::Poll(std::uint32_t now_ms) {
  if (operation_ == Operation::kIdle) {
    return {terminal_result_, true, completed_mask_, state_};
  }
  if (DeadlineReached(now_ms, deadline_ms_)) {
    uart_.Cancel();
    Finish({ResultCode::kTimeout, step_index_});
    return {terminal_result_, true, completed_mask_, state_};
  }
  if (!exchange_started_) {
    const Result start = StartCurrentExchange();
    if (start.code == ResultCode::kBusy) {
      return {start, false, completed_mask_, state_};
    }
    if (!start.ok()) {
      Finish(start);
      return {terminal_result_, true, completed_mask_, state_};
    }
  }

  std::size_t reply_size = 0U;
  const IoStatus status =
      uart_.PollExchange(now_ms, reply_.data(), reply_.size(), &reply_size);
  if (status == IoStatus::kBusy) {
    return {{ResultCode::kBusy, 0}, false, completed_mask_, state_};
  }
  if (status != IoStatus::kOk) {
    Finish(IoResult(status));
    return {terminal_result_, true, completed_mask_, state_};
  }

  const Step& step = steps_[step_index_];
  if (step.expects_response) {
    const Result consumed = ConsumeReply(reply_.data(), reply_size);
    if (!consumed.ok()) {
      Finish(consumed);
      return {terminal_result_, true, completed_mask_, state_};
    }
  }
  completed_mask_ = static_cast<std::uint16_t>(completed_mask_ | step.field);
  ++step_index_;
  exchange_started_ = false;
  if (step_index_ >= step_count_) {
    Finish(OkResult());
    return {terminal_result_, true, completed_mask_, state_};
  }
  return {{ResultCode::kBusy, 0}, false, completed_mask_, state_};
}

void BusServoUartDriver::Cancel() {
  if (exchange_started_) {
    uart_.Cancel();
  }
  steps_ = {};
  state_ = {};
  terminal_result_ = {ResultCode::kIoError, 0};
  operation_ = Operation::kIdle;
  deadline_ms_ = 0U;
  completed_mask_ = 0U;
  step_count_ = 0U;
  step_index_ = 0U;
  exchange_started_ = false;
}

Result BusServoUartDriver::BeginOperation(Operation operation,
                                          std::uint32_t now_ms) {
  if (busy()) {
    return {ResultCode::kBusy, 0};
  }
  steps_ = {};
  state_ = {};
  terminal_result_ = {ResultCode::kBusy, 0};
  operation_ = operation;
  deadline_ms_ = now_ms + kOperationTimeoutMs;
  completed_mask_ = 0U;
  step_count_ = 0U;
  step_index_ = 0U;
  exchange_started_ = false;
  return OkResult();
}

Result BusServoUartDriver::AddStep(std::uint8_t id, BusServoOpcode opcode,
                                   const std::uint8_t* arguments,
                                   std::size_t argument_count,
                                   bool expects_response,
                                   std::uint8_t response_arguments,
                                   std::uint16_t field) {
  if (step_count_ >= steps_.size()) {
    return {ResultCode::kOutOfRange, step_count_};
  }
  Step step{};
  const Result result = BusServoCodec::BuildFrame(id, opcode, arguments,
                                                  argument_count, &step.frame);
  if (!result.ok()) {
    return result;
  }
  step.response_opcode = opcode;
  step.field = field;
  step.response_arguments = response_arguments;
  step.expects_response = expects_response;
  steps_[step_count_] = step;
  ++step_count_;
  return OkResult();
}

Result BusServoUartDriver::StartCurrentExchange() {
  if (step_index_ >= step_count_) {
    return {ResultCode::kInvalidArgument, step_index_};
  }
  const Step& step = steps_[step_index_];
  const std::size_t expected_reply_size =
      step.expects_response
          ? static_cast<std::size_t>(step.response_arguments) + 6U
          : 0U;
  const IoStatus status =
      uart_.BeginExchange(step.frame.bytes.data(), step.frame.size,
                          expected_reply_size, deadline_ms_);
  if (status == IoStatus::kOk) {
    exchange_started_ = true;
  }
  return IoResult(status);
}

Result BusServoUartDriver::ConsumeReply(const std::uint8_t* reply,
                                        std::size_t size) {
  const ParsedBusServoFrame parsed = BusServoCodec::ParseFrame(reply, size);
  if (!parsed.result.ok()) {
    return parsed.result;
  }
  const Step& step = steps_[step_index_];
  const std::uint8_t requested_id = step.frame.bytes[2];
  const bool broadcast_id_read =
      requested_id == 254U && step.field == BusServoState::kFieldId;
  if ((!broadcast_id_read && parsed.servo_id != requested_id) ||
      parsed.opcode != step.response_opcode ||
      parsed.argument_count != step.response_arguments) {
    return {ResultCode::kIoError, static_cast<std::uint16_t>(step_index_ + 1U)};
  }
  const auto& arguments = parsed.arguments;
  state_.reported_id = parsed.servo_id;
  switch (step.field) {
    case BusServoState::kFieldId:
      state_.reported_id = arguments[0];
      break;
    case BusServoState::kFieldPosition:
      state_.position = DecodeI16(arguments.data());
      break;
    case BusServoState::kFieldOffset:
      state_.offset = DecodeI8(arguments[0]);
      break;
    case BusServoState::kFieldVoltage:
      state_.voltage_mv = DecodeU16(arguments.data());
      break;
    case BusServoState::kFieldTemperature:
      state_.temperature_c = arguments[0];
      break;
    case BusServoState::kFieldPositionLimits:
      state_.position_min = DecodeU16(arguments.data());
      state_.position_max = DecodeU16(&arguments[2]);
      break;
    case BusServoState::kFieldVoltageLimits:
      state_.voltage_min_mv = DecodeU16(arguments.data());
      state_.voltage_max_mv = DecodeU16(&arguments[2]);
      break;
    case BusServoState::kFieldTemperatureLimit:
      state_.temperature_limit_c = arguments[0];
      break;
    case BusServoState::kFieldTorque:
      state_.torque_enabled = arguments[0] != 0U;
      break;
    default:
      return {ResultCode::kInvalidArgument, step.field};
  }
  state_.valid_fields =
      static_cast<std::uint16_t>(state_.valid_fields | step.field);
  return OkResult();
}

void BusServoUartDriver::Finish(Result result) {
  if (!result.ok() && result.code != ResultCode::kBusy &&
      completed_mask_ != 0U) {
    // The exact completed/applied mask remains available in the poll result.
    // Detail retains the causal result code for diagnostics.
    terminal_result_ = {ResultCode::kPartial,
                        static_cast<std::uint16_t>(result.code)};
  } else {
    terminal_result_ = result;
  }
  operation_ = Operation::kIdle;
  exchange_started_ = false;
}

}  // namespace mentor_pi::mcu::drivers
