#include "mentor_pi_mcu/domain/bus_servo.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {

Result BusServoCodec::BuildFrame(std::uint8_t servo_id, BusServoOpcode opcode,
                                 const std::uint8_t* arguments,
                                 std::size_t argument_count,
                                 BusServoFrame* frame) {
  if (frame == nullptr || (arguments == nullptr && argument_count != 0U)) {
    return {ResultCode::kInvalidArgument, 0};
  }
  if (servo_id == 0U || servo_id > 254U) {
    return {ResultCode::kOutOfRange, 0};
  }
  if (argument_count > kBusServoMaximumArguments) {
    return {ResultCode::kOutOfRange,
            static_cast<std::uint16_t>(argument_count)};
  }

  BusServoFrame built{};
  built.bytes[0] = kBusServoFrameHeader;
  built.bytes[1] = kBusServoFrameHeader;
  built.bytes[2] = servo_id;
  built.bytes[3] = static_cast<std::uint8_t>(argument_count + 3U);
  built.bytes[4] = static_cast<std::uint8_t>(opcode);
  for (std::size_t index = 0; index < argument_count; ++index) {
    built.bytes[5U + index] = arguments[index];
  }
  built.bytes[5U + argument_count] =
      Checksum(&built.bytes[2], argument_count + 3U);
  built.size = static_cast<std::uint8_t>(argument_count + 6U);
  *frame = built;
  return OkResult();
}

ParsedBusServoFrame BusServoCodec::ParseFrame(const std::uint8_t* bytes,
                                              std::size_t size) {
  ParsedBusServoFrame parsed{};
  if (bytes == nullptr || size < 6U) {
    parsed.result = {ResultCode::kInvalidArgument, 0};
    return parsed;
  }
  if (bytes[0] != kBusServoFrameHeader || bytes[1] != kBusServoFrameHeader) {
    parsed.result = {ResultCode::kInvalidArgument, 1};
    return parsed;
  }
  const std::uint8_t length = bytes[3];
  if (length < 3U || length > kBusServoMaximumArguments + 3U ||
      size != static_cast<std::size_t>(length) + 3U) {
    parsed.result = {ResultCode::kInvalidArgument, 2};
    return parsed;
  }
  if (bytes[2] == 0U || bytes[2] > 254U) {
    parsed.result = {ResultCode::kOutOfRange, 0};
    return parsed;
  }
  const std::size_t argument_count = static_cast<std::size_t>(length - 3U);
  const std::uint8_t expected_checksum =
      Checksum(&bytes[2], argument_count + 3U);
  if (bytes[size - 1U] != expected_checksum) {
    parsed.result = {ResultCode::kIoError, 1};
    return parsed;
  }

  parsed.servo_id = bytes[2];
  parsed.opcode = static_cast<BusServoOpcode>(bytes[4]);
  parsed.argument_count = static_cast<std::uint8_t>(argument_count);
  for (std::size_t index = 0; index < argument_count; ++index) {
    parsed.arguments[index] = bytes[5U + index];
  }
  parsed.result = OkResult();
  return parsed;
}

std::uint8_t BusServoCodec::Checksum(const std::uint8_t* id_through_arguments,
                                     std::size_t size) {
  if (id_through_arguments == nullptr) {
    return 0U;
  }
  std::uint16_t sum = 0;
  for (std::size_t index = 0; index < size; ++index) {
    sum = static_cast<std::uint16_t>(sum + id_through_arguments[index]);
  }
  return static_cast<std::uint8_t>(~sum);
}

BusMoveAdmission BusServoScheduler::SubmitMove(const BusServoCommand& command) {
  const Result result = ValidateBusServoCommand(command);
  if (!result.ok()) {
    return {result, generation_, false};
  }
  generation_ = NextGeneration(generation_);
  const bool overwrote = pending_move_.valid;
  if (overwrote) {
    move_overwrite_count_.Increment();
  }
  pending_move_ = {command, generation_, 0, true};
  return {OkResult(), generation_, overwrote};
}

Result BusServoScheduler::AcceptStop(const StopBusServosCommand& command) {
  const Result result = ValidateStopBusServosCommand(command);
  if (!result.ok()) {
    return result;
  }
  if (stop_pending_) {
    return {ResultCode::kBusy, 0};
  }
  stop_command_ = command;
  stop_next_index_ = 0;
  stop_pending_ = true;
  stop_watermark_ = generation_;
  // The UART owner serializes admission, so every move present at this point
  // is necessarily at or below the newly captured watermark. Avoid numeric
  // ordering here because generations intentionally wrap.
  pending_move_.valid = false;
  if (active_move_.valid) {
    if (frame_in_progress_) {
      cancel_active_after_frame_ = true;
    } else {
      active_move_.valid = false;
    }
  }
  return OkResult();
}

ScheduledBusFrame BusServoScheduler::BeginFrame() {
  if (frame_in_progress_) {
    return {{ResultCode::kBusy, 0}};
  }
  if (stop_pending_) {
    return BuildStopFrame();
  }
  if (!active_move_.valid && pending_move_.valid) {
    active_move_ = pending_move_;
    pending_move_.valid = false;
  }
  if (active_move_.valid) {
    return BuildMoveFrame();
  }
  return {{ResultCode::kBusy, 0}};
}

void BusServoScheduler::CompleteFrame(bool transmitted) {
  if (!frame_in_progress_) {
    return;
  }
  frame_in_progress_ = false;
  if (in_progress_kind_ == ScheduledBusFrameKind::kStop) {
    if (!transmitted) {
      stop_pending_ = false;
    } else {
      ++stop_next_index_;
      if (stop_next_index_ >= stop_command_.count) {
        stop_pending_ = false;
      }
    }
  } else if (in_progress_kind_ == ScheduledBusFrameKind::kMove) {
    if (!transmitted || cancel_active_after_frame_) {
      active_move_.valid = false;
    } else {
      ++active_move_.next_index;
      if (active_move_.next_index >= active_move_.command.count) {
        active_move_.valid = false;
      }
    }
  }
  cancel_active_after_frame_ = false;
  in_progress_kind_ = ScheduledBusFrameKind::kNone;
}

void BusServoScheduler::CancelAll() {
  pending_move_.valid = false;
  active_move_.valid = false;
  stop_pending_ = false;
  frame_in_progress_ = false;
  cancel_active_after_frame_ = false;
  in_progress_kind_ = ScheduledBusFrameKind::kNone;
}

bool BusServoScheduler::has_work() const {
  return frame_in_progress_ || stop_pending_ || active_move_.valid ||
         pending_move_.valid;
}

std::uint32_t BusServoScheduler::NextGeneration(std::uint32_t generation) {
  return generation == std::numeric_limits<std::uint32_t>::max()
             ? 1U
             : generation + 1U;
}

ScheduledBusFrame BusServoScheduler::BuildMoveFrame() {
  const std::size_t index = active_move_.next_index;
  const std::uint16_t position = active_move_.command.position[index];
  const std::uint16_t duration = active_move_.command.duration_ms;
  const std::array<std::uint8_t, 4> arguments{
      static_cast<std::uint8_t>(position & 0xffU),
      static_cast<std::uint8_t>(position >> 8U),
      static_cast<std::uint8_t>(duration & 0xffU),
      static_cast<std::uint8_t>(duration >> 8U)};
  ScheduledBusFrame scheduled{};
  scheduled.result = BusServoCodec::BuildFrame(
      active_move_.command.servo_id[index], BusServoOpcode::kMoveTimeWrite,
      arguments.data(), arguments.size(), &scheduled.frame);
  if (!scheduled.result.ok()) {
    active_move_.valid = false;
    return scheduled;
  }
  scheduled.kind = ScheduledBusFrameKind::kMove;
  scheduled.generation = active_move_.generation;
  scheduled.batch_index = active_move_.next_index;
  frame_in_progress_ = true;
  in_progress_kind_ = ScheduledBusFrameKind::kMove;
  return scheduled;
}

ScheduledBusFrame BusServoScheduler::BuildStopFrame() {
  ScheduledBusFrame scheduled{};
  scheduled.result = BusServoCodec::BuildFrame(
      stop_command_.servo_id[stop_next_index_], BusServoOpcode::kMoveStop,
      nullptr, 0, &scheduled.frame);
  if (!scheduled.result.ok()) {
    stop_pending_ = false;
    return scheduled;
  }
  scheduled.kind = ScheduledBusFrameKind::kStop;
  scheduled.generation = stop_watermark_;
  scheduled.batch_index = stop_next_index_;
  frame_in_progress_ = true;
  in_progress_kind_ = ScheduledBusFrameKind::kStop;
  return scheduled;
}

}  // namespace mentor_pi::mcu
