// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi_mcu::app::microros::BusServiceKind;
using mentor_pi_mcu::app::microros::ErrorSource;

constexpr std::uint32_t kBusWorkerHeartbeatUs = 20000U;

std::uint8_t CountSetBits(std::uint16_t value) {
  std::uint8_t count = 0U;
  while (value != 0U) {
    count = static_cast<std::uint8_t>(count + (value & 1U));
    value = static_cast<std::uint16_t>(value >> 1U);
  }
  return count;
}

mentor_pi_mcu::app::microros::BusServoState ConvertState(
    const mentor_pi::mcu::drivers::BusServoState& input) {
  mentor_pi_mcu::app::microros::BusServoState output{};
  output.valid_fields = input.valid_fields;
  output.requested_id = input.requested_id;
  output.reported_id = input.reported_id;
  output.position = input.position;
  output.offset = input.offset;
  output.voltage_mv = input.voltage_mv;
  output.temperature_c = input.temperature_c;
  output.position_min = input.position_min;
  output.position_max = input.position_max;
  output.voltage_min_mv = input.voltage_min_mv;
  output.voltage_max_mv = input.voltage_max_mv;
  output.temperature_limit_c = input.temperature_limit_c;
  output.torque_enabled = input.torque_enabled;
  return output;
}

}  // namespace

void ControllerRuntime::RunBusServoOnce() {
  if (!initialized()) {
    return;
  }
  const std::uint32_t started_us =
      hooks_.monotonic_microseconds(hooks_.context);
  const std::uint32_t now_ms = hooks_.monotonic_milliseconds(hooks_.context);

  if (bus_driver_.busy()) {
    ProcessBusDriver(now_ms);
  }
  StartNextBusWork(now_ms);
  bus_busy_.store(bus_driver_.busy() || bus_service_pending_,
                  std::memory_order_release);
  RecordTaskProgress(ControllerTask::kBusServo, started_us,
                     kBusWorkerHeartbeatUs);
}

void ControllerRuntime::ProcessBusDriver(std::uint32_t now_ms) {
  mentor_pi_mcu::app::microros::ServiceToken canceled_token{};
  BusServiceReply canceled_reply{};
  bool canceled_service = false;
  mentor_pi::mcu::drivers::BusServoPollResult result{};
  {
    CriticalGuard guard(this);
    const bool move_is_stale =
        bus_move_active_ &&
        (!desired_session_active_.load(std::memory_order_relaxed) ||
         bus_move_session_generation_ !=
             session_generation_.load(std::memory_order_relaxed));
    const bool service_is_stale =
        active_bus_operation_ != BusServiceKind::kNone &&
        !TokenIsCurrent(active_bus_service_token_);
    if (move_is_stale || service_is_stale) {
      bus_driver_.Cancel();
      bus_move_active_ = false;
      bus_move_session_generation_ = 0U;
      previous_bus_completed_mask_ = 0U;
      if (service_is_stale) {
        canceled_token = active_bus_service_token_;
        canceled_reply.kind = active_bus_operation_;
        active_bus_operation_ = BusServiceKind::kNone;
        active_bus_service_token_ = {};
        bus_service_pending_ = false;
        canceled_service = true;
      }
    } else {
      result = bus_driver_.Poll(now_ms);
    }
  }
  if (canceled_service) {
    Complete(&bus_service_slot_, canceled_token, canceled_reply);
  }
  if (canceled_service || (!bus_driver_.busy() && !result.complete)) {
    return;
  }
  if (!result.complete) {
    if (bus_move_active_ && bus_service_pending_ &&
        pending_bus_request_.kind == BusServiceKind::kStop &&
        result.completed_mask != previous_bus_completed_mask_) {
      // Poll just completed one full UART frame and deliberately does not
      // start the next until the following poll. Canceling here therefore
      // abandons only the unsent remainder; it never truncates a frame.
      bus_driver_.Cancel();
      bus_move_active_ = false;
      bus_move_session_generation_ = 0U;
      previous_bus_completed_mask_ = 0U;
    } else {
      previous_bus_completed_mask_ = result.completed_mask;
    }
    return;
  }
  FinishBusOperation(result);
}

void ControllerRuntime::FinishBusOperation(
    const mentor_pi::mcu::drivers::BusServoPollResult& result) {
  RecordPeripheralResult(0U, result.result, ErrorSource::kBusServos);
  if (bus_move_active_) {
    bus_move_active_ = false;
    bus_move_session_generation_ = 0U;
    previous_bus_completed_mask_ = 0U;
    return;
  }
  if (active_bus_operation_ == BusServiceKind::kNone) {
    return;
  }

  BusServiceReply reply{};
  reply.kind = active_bus_operation_;
  switch (active_bus_operation_) {
    case BusServiceKind::kGetState:
      reply.get_state.result = result.result;
      reply.get_state.state = ConvertState(result.state);
      break;
    case BusServiceKind::kConfigure:
      reply.configure.result = result.result;
      reply.configure.applied_mask = result.completed_mask;
      reply.configure.effective_id = pending_bus_request_.configure.servo_id;
      if ((result.completed_mask &
           mentor_pi::mcu::ConfigureBusServoCommand::kSetId) != 0U) {
        reply.configure.effective_id = pending_bus_request_.configure.new_id;
      }
      break;
    case BusServiceKind::kStop:
      reply.stop.result = result.result;
      reply.stop.commands_transmitted = CountSetBits(result.completed_mask);
      break;
    case BusServiceKind::kNone:
      break;
  }
  Complete(&bus_service_slot_, active_bus_service_token_, reply);
  active_bus_operation_ = BusServiceKind::kNone;
  active_bus_service_token_ = {};
  bus_service_pending_ = false;
  previous_bus_completed_mask_ = 0U;
}

void ControllerRuntime::StartNextBusWork(std::uint32_t now_ms) {
  if (!bus_service_pending_ && active_bus_operation_ == BusServiceKind::kNone) {
    BusServiceRequest request{};
    mentor_pi_mcu::app::microros::ServiceToken token{};
    if (Take(&bus_service_slot_, &token, &request)) {
      pending_bus_request_ = request;
      pending_bus_token_ = token;
      bus_service_pending_ = true;
    }
  }

  if (bus_driver_.busy()) {
    return;
  }

  if (bus_service_pending_) {
    if (!TokenIsCurrent(pending_bus_token_)) {
      BusServiceReply canceled{};
      canceled.kind = pending_bus_request_.kind;
      Complete(&bus_service_slot_, pending_bus_token_, canceled);
      bus_service_pending_ = false;
      return;
    }
    Result started{ResultCode::kUnsupported, 0U};
    {
      // Start* only builds fixed frames and admits one nonblocking exchange.
      // Keep teardown from invalidating a persistent configuration between
      // the final token check and UART admission.
      CriticalGuard guard(this);
      if (!TokenIsCurrent(pending_bus_token_)) {
        BusServiceReply canceled{};
        canceled.kind = pending_bus_request_.kind;
        Complete(&bus_service_slot_, pending_bus_token_, canceled);
        bus_service_pending_ = false;
        return;
      }
      switch (pending_bus_request_.kind) {
        case BusServiceKind::kGetState:
          started =
              bus_driver_.StartQuery(pending_bus_request_.get_state, now_ms);
          break;
        case BusServiceKind::kConfigure:
          started = bus_driver_.StartConfigure(pending_bus_request_.configure,
                                               now_ms);
          break;
        case BusServiceKind::kStop:
          started = bus_driver_.StartStop(pending_bus_request_.stop, now_ms);
          break;
        case BusServiceKind::kNone:
          started = {ResultCode::kInvalidArgument, 0U};
          break;
      }
      if (started.ok()) {
        active_bus_operation_ = pending_bus_request_.kind;
        active_bus_service_token_ = pending_bus_token_;
        previous_bus_completed_mask_ = 0U;
      }
    }
    if (!started.ok()) {
      mentor_pi::mcu::drivers::BusServoPollResult terminal{};
      terminal.complete = true;
      terminal.result = started;
      active_bus_operation_ = pending_bus_request_.kind;
      active_bus_service_token_ = pending_bus_token_;
      FinishBusOperation(terminal);
      return;
    }
    return;
  }

  mentor_pi::mcu::BusMotionSnapshot motion{};
  MailboxTag tag{};
  bool available = false;
  {
    CriticalGuard guard(this);
    available = bus_mailbox_.ConsumeLatest(&motion);
    tag = bus_mailbox_tag_;
  }
  if (!available) {
    return;
  }
  Result started{};
  {
    CriticalGuard guard(this);
    if (!desired_session_active_.load(std::memory_order_relaxed) ||
        tag.session_generation !=
            session_generation_.load(std::memory_order_relaxed) ||
        tag.command_generation != motion.generation ||
        !GenerationIsAfter(motion.generation, bus_stop_watermark_)) {
      return;
    }
    started = bus_driver_.StartMove(motion.command, now_ms);
    if (started.ok()) {
      bus_move_active_ = true;
      bus_move_session_generation_ = tag.session_generation;
      previous_bus_completed_mask_ = 0U;
    }
  }
  if (!started.ok()) {
    RecordPeripheralResult(0U, started, ErrorSource::kBusServos);
    return;
  }
}

}  // namespace mentor_pi_mcu::app::controller
