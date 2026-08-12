// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "runtime_internal.h"

extern "C" {
#include "rcl/error_handling.h"
#include "rmw_microros/rmw_microros.h"
}

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::uint32_t kLocalServiceDeadlineMs = 50U;
constexpr std::uint32_t kBusServiceDeadlineMs = 200U;

enum class ServiceSlotIndex : std::uint8_t {
  kMotorModel = 0U,
  kMotorAdrc,
  kPwmOffsets,
  kBatteryThreshold,
  kBus,
};

enum class ServiceRequestGroupIndex : std::uint8_t {
  kMotorModel = 0U,
  kMotorAdrc,
  kPwmOffsets,
  kBatteryThreshold,
  kBus,
};

enum class BusRequestIndex : std::uint8_t {
  kStop = 0U,
  kGetState,
  kConfigure,
};

static_assert(static_cast<std::size_t>(ServiceSlotIndex::kBus) + 1U ==
              kServiceSlotCount);
static_assert(static_cast<std::size_t>(ServiceRequestGroupIndex::kBus) + 1U ==
              kServiceRequestGroupCount);
static_assert(static_cast<std::size_t>(BusRequestIndex::kConfigure) + 1U ==
              kBusServiceEndpointCount);

constexpr std::size_t ToIndex(ServiceIndex index) {
  return static_cast<std::size_t>(index);
}

template <typename Integer>
void SaturatingIncrement(Integer* value) {
  if (*value != std::numeric_limits<Integer>::max()) {
    ++(*value);
  }
}

template <typename Response>
void SetWireResult(Response* response, mentor_pi::mcu::Result result) {
  response->result.code = static_cast<std::uint8_t>(result.code);
  response->result.detail = result.detail;
}

mentor_pi::mcu::Result BusyResult() {
  return {mentor_pi::mcu::ResultCode::kBusy, 0U};
}

mentor_pi::mcu::Result TimeoutResult() {
  return {mentor_pi::mcu::ResultCode::kTimeout, 0U};
}

mentor_pi::mcu::Result RclError(rcl_ret_t result) {
  return {mentor_pi::mcu::ResultCode::kIoError,
          static_cast<std::uint16_t>(result)};
}

}  // namespace

rcl_ret_t MicroRosRuntime::TakeServiceRequest(std::size_t service_index,
                                              rmw_request_id_t* request_id,
                                              void* request) {
  return static_cast<rcl_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kRequestTake, kNonblockingCallDeadlineMs,
      [this, service_index, request_id, request]() {
        return static_cast<std::int32_t>(
            rcl_take_request(&services_[service_index], request_id, request));
      }));
}

void MicroRosRuntime::PumpServices(std::uint32_t now_ms) {
  if (PollOneServiceCompletion(now_ms) ||
      lifecycle_.state() != SessionState::kActive) {
    return;
  }
  static_cast<void>(TakeOneServiceRequest(now_ms));
}

bool MicroRosRuntime::TakeOneServiceRequest(std::uint32_t now_ms) {
  for (std::size_t offset = 0U; offset < kServiceRequestGroupCount; ++offset) {
    const std::size_t group = service_request_cursor_.Peek(offset);
    bool taken = false;
    switch (static_cast<ServiceRequestGroupIndex>(group)) {
      case ServiceRequestGroupIndex::kMotorModel:
        taken = TakeMotorModelRequest(now_ms);
        break;
      case ServiceRequestGroupIndex::kMotorAdrc:
        taken = TakeMotorAdrcRequest(now_ms);
        break;
      case ServiceRequestGroupIndex::kPwmOffsets:
        taken = TakePwmOffsetsRequest(now_ms);
        break;
      case ServiceRequestGroupIndex::kBatteryThreshold:
        taken = TakeBatteryThresholdRequest(now_ms);
        break;
      case ServiceRequestGroupIndex::kBus:
        taken = TakeOneBusRequest(now_ms);
        break;
    }
    if (taken || lifecycle_.state() != SessionState::kActive) {
      service_request_cursor_.AdvancePast(group);
      return true;
    }
  }
  return false;
}

bool MicroRosRuntime::TakeOneBusRequest(std::uint32_t now_ms) {
  if (bus_service_slot_.occupied) {
    for (std::size_t offset = 0U; offset < kBusServiceEndpointCount; ++offset) {
      const std::size_t endpoint = busy_bus_request_cursor_.Peek(offset);
      bool taken = false;
      switch (static_cast<BusRequestIndex>(endpoint)) {
        case BusRequestIndex::kStop:
          taken = TakeBusStopRequest(now_ms);
          break;
        case BusRequestIndex::kGetState:
          taken = TakeBusGetRequest(now_ms);
          break;
        case BusRequestIndex::kConfigure:
          taken = TakeBusConfigureRequest(now_ms);
          break;
      }
      if (taken || lifecycle_.state() != SessionState::kActive) {
        busy_bus_request_cursor_.AdvancePast(endpoint);
        return true;
      }
    }
    return false;
  }

  // Stop retains priority whenever the shared bus-service slot is free.
  if (TakeBusStopRequest(now_ms) ||
      lifecycle_.state() != SessionState::kActive) {
    return true;
  }
  const bool get_first = alternate_bus_get_first_;
  if ((get_first ? TakeBusGetRequest(now_ms)
                 : TakeBusConfigureRequest(now_ms)) ||
      lifecycle_.state() != SessionState::kActive) {
    alternate_bus_get_first_ = !alternate_bus_get_first_;
    return true;
  }
  if ((get_first ? TakeBusConfigureRequest(now_ms)
                 : TakeBusGetRequest(now_ms)) ||
      lifecycle_.state() != SessionState::kActive) {
    alternate_bus_get_first_ = !alternate_bus_get_first_;
    return true;
  }
  return false;
}

bool MicroRosRuntime::TakeMotorModelRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kMotorModel), &request_id,
                         &service_messages_.motor_model_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);

  auto& response = service_messages_.motor_model_response;
  response = {};
  if (motor_model_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorModel),
                                          &request_id, &response, result));
    return true;
  }

  const auto model = static_cast<mentor_pi::mcu::MotorModel>(
      service_messages_.motor_model_request.model);
  if (!mentor_pi::mcu::IsValidMotorModel(model)) {
    const mentor_pi::mcu::Result result{
        mentor_pi::mcu::ResultCode::kInvalidArgument,
        service_messages_.motor_model_request.model};
    SetWireResult(&response, result);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorModel),
                                          &request_id, &response, result));
    return true;
  }

  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_motor_model(hooks_.context, token, model)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorModel),
                                          &request_id, &response, result));
    return true;
  }
  motor_model_slot_.request_id = request_id;
  motor_model_slot_.token = token;
  motor_model_slot_.deadline_ms = now_ms + kLocalServiceDeadlineMs;
  motor_model_slot_.occupied = true;
  return true;
}

bool MicroRosRuntime::TakeMotorAdrcRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kMotorAdrc), &request_id,
                         &service_messages_.motor_adrc_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);

  auto& response = service_messages_.motor_adrc_response;
  response = {};
  if (motor_adrc_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorAdrc),
                                          &request_id, &response, result));
    return true;
  }

  const auto& request = service_messages_.motor_adrc_request;
  mentor_pi::mcu::SetMotorAdrcCommand command{};
  command.update_mask = request.update_mask;
  std::copy_n(request.input_gain_rps_per_second_per_permille,
              command.input_gain_rps_per_second_per_permille.size(),
              command.input_gain_rps_per_second_per_permille.begin());
  std::copy_n(request.controller_bandwidth_rad_s,
              command.controller_bandwidth_rad_s.size(),
              command.controller_bandwidth_rad_s.begin());
  std::copy_n(request.observer_bandwidth_rad_s,
              command.observer_bandwidth_rad_s.size(),
              command.observer_bandwidth_rad_s.begin());
  std::copy_n(request.velocity_filter_new_weight,
              command.velocity_filter_new_weight.size(),
              command.velocity_filter_new_weight.begin());
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidateSetMotorAdrcCommand(command);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorAdrc),
                                          &request_id, &response, validation));
    return true;
  }

  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_motor_adrc(hooks_.context, token, command)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kMotorAdrc),
                                          &request_id, &response, result));
    return true;
  }
  motor_adrc_slot_.request_id = request_id;
  motor_adrc_slot_.token = token;
  motor_adrc_slot_.deadline_ms = now_ms + kLocalServiceDeadlineMs;
  motor_adrc_slot_.occupied = true;
  return true;
}

bool MicroRosRuntime::TakePwmOffsetsRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kPwmOffsets), &request_id,
                         &service_messages_.pwm_offsets_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);

  auto& response = service_messages_.pwm_offsets_response;
  response = {};
  if (pwm_offsets_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kPwmOffsets),
                                          &request_id, &response, result));
    return true;
  }

  mentor_pi::mcu::PwmServoOffsetCommand command{};
  command.update_mask = service_messages_.pwm_offsets_request.update_mask;
  std::copy_n(service_messages_.pwm_offsets_request.offset_us,
              command.offset_us.size(), command.offset_us.begin());
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidatePwmServoOffsets(command);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kPwmOffsets),
                                          &request_id, &response, validation));
    return true;
  }

  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_pwm_offsets(hooks_.context, token, command)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kPwmOffsets),
                                          &request_id, &response, result));
    return true;
  }
  pwm_offsets_slot_.request_id = request_id;
  pwm_offsets_slot_.token = token;
  pwm_offsets_slot_.deadline_ms = now_ms + kLocalServiceDeadlineMs;
  pwm_offsets_slot_.occupied = true;
  return true;
}

bool MicroRosRuntime::TakeBatteryThresholdRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kBatteryThreshold), &request_id,
                         &service_messages_.battery_threshold_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);

  auto& response = service_messages_.battery_threshold_response;
  response = {};
  if (battery_threshold_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(
        SendServiceResponse(ToIndex(ServiceIndex::kBatteryThreshold),
                            &request_id, &response, result));
    return true;
  }

  const std::uint16_t threshold_mv =
      service_messages_.battery_threshold_request.threshold_mv;
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidateBatteryThreshold(threshold_mv);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(
        SendServiceResponse(ToIndex(ServiceIndex::kBatteryThreshold),
                            &request_id, &response, validation));
    return true;
  }

  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_battery_threshold(hooks_.context, token, threshold_mv)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(
        SendServiceResponse(ToIndex(ServiceIndex::kBatteryThreshold),
                            &request_id, &response, result));
    return true;
  }
  battery_threshold_slot_.request_id = request_id;
  battery_threshold_slot_.token = token;
  battery_threshold_slot_.deadline_ms = now_ms + kLocalServiceDeadlineMs;
  battery_threshold_slot_.occupied = true;
  return true;
}

bool MicroRosRuntime::TakeBusStopRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kBusStop), &request_id,
                         &service_messages_.bus_stop_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);
  auto& response = service_messages_.bus_stop_response;
  response = {};
  if (bus_service_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusStop),
                                          &request_id, &response, result));
    return true;
  }

  mentor_pi::mcu::StopBusServosCommand command{};
  command.count = service_messages_.bus_stop_request.count;
  std::copy_n(service_messages_.bus_stop_request.servo_id,
              command.servo_id.size(), command.servo_id.begin());
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidateStopBusServosCommand(command);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusStop),
                                          &request_id, &response, validation));
    return true;
  }
  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_bus_stop(hooks_.context, token, command)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusStop),
                                          &request_id, &response, result));
    return true;
  }
  bus_service_slot_.request_id = request_id;
  bus_service_slot_.token = token;
  bus_service_slot_.deadline_ms = now_ms + kBusServiceDeadlineMs;
  bus_service_slot_.occupied = true;
  bus_service_slot_.kind = BusServiceKind::kStop;
  return true;
}

bool MicroRosRuntime::TakeBusGetRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kBusGetState), &request_id,
                         &service_messages_.bus_get_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);
  auto& response = service_messages_.bus_get_response;
  response = {};
  if (bus_service_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusGetState),
                                          &request_id, &response, result));
    return true;
  }

  const mentor_pi::mcu::GetBusServoStateCommand command{
      service_messages_.bus_get_request.servo_id,
      service_messages_.bus_get_request.fields};
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidateGetBusServoStateCommand(command);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusGetState),
                                          &request_id, &response, validation));
    return true;
  }
  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_bus_get_state(hooks_.context, token, command)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusGetState),
                                          &request_id, &response, result));
    return true;
  }
  bus_service_slot_.request_id = request_id;
  bus_service_slot_.token = token;
  bus_service_slot_.deadline_ms = now_ms + kBusServiceDeadlineMs;
  bus_service_slot_.occupied = true;
  bus_service_slot_.kind = BusServiceKind::kGetState;
  return true;
}

bool MicroRosRuntime::TakeBusConfigureRequest(std::uint32_t now_ms) {
  rmw_request_id_t request_id{};
  const rcl_ret_t take =
      TakeServiceRequest(ToIndex(ServiceIndex::kBusConfigure), &request_id,
                         &service_messages_.bus_configure_request);
  if (take == RCL_RET_SERVICE_TAKE_FAILED) {
    return false;
  }
  if (take != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(take));
    return true;
  }
  SaturatingIncrement(&counters_.service_requests);
  auto& response = service_messages_.bus_configure_response;
  response = {};
  if (bus_service_slot_.occupied) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusConfigure),
                                          &request_id, &response, result));
    return true;
  }

  const auto& request = service_messages_.bus_configure_request;
  const mentor_pi::mcu::ConfigureBusServoCommand command{
      request.servo_id,
      request.update_mask,
      request.new_id,
      request.offset,
      request.position_min,
      request.position_max,
      request.voltage_min_mv,
      request.voltage_max_mv,
      request.temperature_limit_c,
      request.torque_enabled};
  const mentor_pi::mcu::Result validation =
      mentor_pi::mcu::ValidateConfigureBusServoCommand(command);
  if (!validation.ok()) {
    SetWireResult(&response, validation);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusConfigure),
                                          &request_id, &response, validation));
    return true;
  }
  const ServiceToken token = NewServiceToken();
  if (!hooks_.dispatch_bus_configure(hooks_.context, token, command)) {
    const auto result = BusyResult();
    SetWireResult(&response, result);
    SaturatingIncrement(&counters_.service_busy_rejections);
    static_cast<void>(SendServiceResponse(ToIndex(ServiceIndex::kBusConfigure),
                                          &request_id, &response, result));
    return true;
  }
  bus_service_slot_.request_id = request_id;
  bus_service_slot_.token = token;
  bus_service_slot_.deadline_ms = now_ms + kBusServiceDeadlineMs;
  bus_service_slot_.occupied = true;
  bus_service_slot_.kind = BusServiceKind::kConfigure;
  return true;
}

bool MicroRosRuntime::PollOneServiceCompletion(std::uint32_t now_ms) {
  for (std::size_t offset = 0U; offset < kServiceSlotCount; ++offset) {
    const std::size_t slot = service_completion_cursor_.Peek(offset);
    bool occupied = false;
    switch (static_cast<ServiceSlotIndex>(slot)) {
      case ServiceSlotIndex::kMotorModel:
        occupied = motor_model_slot_.occupied;
        break;
      case ServiceSlotIndex::kMotorAdrc:
        occupied = motor_adrc_slot_.occupied;
        break;
      case ServiceSlotIndex::kPwmOffsets:
        occupied = pwm_offsets_slot_.occupied;
        break;
      case ServiceSlotIndex::kBatteryThreshold:
        occupied = battery_threshold_slot_.occupied;
        break;
      case ServiceSlotIndex::kBus:
        occupied = bus_service_slot_.occupied;
        break;
    }
    if (!occupied) {
      continue;
    }

    service_completion_cursor_.AdvancePast(slot);
    switch (static_cast<ServiceSlotIndex>(slot)) {
      case ServiceSlotIndex::kMotorModel:
        return PollMotorModelCompletion(now_ms);
      case ServiceSlotIndex::kMotorAdrc:
        return PollMotorAdrcCompletion(now_ms);
      case ServiceSlotIndex::kPwmOffsets:
        return PollPwmOffsetsCompletion(now_ms);
      case ServiceSlotIndex::kBatteryThreshold:
        return PollBatteryThresholdCompletion(now_ms);
      case ServiceSlotIndex::kBus:
        return PollBusCompletion(now_ms);
    }
  }
  return false;
}

bool MicroRosRuntime::PollMotorModelCompletion(std::uint32_t now_ms) {
  if (!motor_model_slot_.occupied) {
    return false;
  }
  MotorModelReply reply{};
  const bool complete =
      hooks_.poll_motor_model(hooks_.context, motor_model_slot_.token, &reply);
  if (motor_model_slot_.response_sent) {
    if (complete) {
      SaturatingIncrement(&counters_.late_response_drops);
      motor_model_slot_ = {};
    }
    return false;
  }
  if (!complete && !DeadlineReached(now_ms, motor_model_slot_.deadline_ms)) {
    return false;
  }
  if (!complete) {
    reply.result = TimeoutResult();
    SaturatingIncrement(&counters_.service_timeouts);
  }
  auto& response = service_messages_.motor_model_response;
  response = {};
  SetWireResult(&response, reply.result);
  response.active_model = static_cast<std::uint8_t>(reply.active_model);
  response.ticks_per_revolution = reply.ticks_per_revolution;
  response.max_rps = reply.max_rps;
  const bool sent = SendServiceResponse(ToIndex(ServiceIndex::kMotorModel),
                                        &motor_model_slot_.request_id,
                                        &response, reply.result);
  if (complete || !sent) {
    motor_model_slot_ = {};
  } else {
    motor_model_slot_.response_sent = true;
  }
  return true;
}

bool MicroRosRuntime::PollMotorAdrcCompletion(std::uint32_t now_ms) {
  if (!motor_adrc_slot_.occupied) {
    return false;
  }
  MotorAdrcReply reply{};
  bool complete =
      hooks_.poll_motor_adrc(hooks_.context, motor_adrc_slot_.token, &reply);
  if (motor_adrc_slot_.response_sent) {
    if (complete) {
      SaturatingIncrement(&counters_.late_response_drops);
      motor_adrc_slot_ = {};
    }
    return false;
  }
  const bool deadline_reached =
      DeadlineReached(now_ms, motor_adrc_slot_.deadline_ms);
  if (!complete && !deadline_reached) {
    return false;
  }
  bool canceled = false;
  if (!complete) {
    canceled = hooks_.cancel_motor_adrc(hooks_.context, motor_adrc_slot_.token);
    if (!canceled) {
      complete = hooks_.poll_motor_adrc(hooks_.context, motor_adrc_slot_.token,
                                        &reply);
    }
    if (!complete) {
      reply.result = TimeoutResult();
      SaturatingIncrement(&counters_.service_timeouts);
    }
  }
  auto& response = service_messages_.motor_adrc_response;
  response = {};
  SetWireResult(&response, reply.result);
  response.applied_mask = reply.applied_mask;
  const bool sent = SendServiceResponse(ToIndex(ServiceIndex::kMotorAdrc),
                                        &motor_adrc_slot_.request_id, &response,
                                        reply.result);
  if (complete || canceled || !sent) {
    motor_adrc_slot_ = {};
  } else {
    motor_adrc_slot_.response_sent = true;
  }
  return true;
}

bool MicroRosRuntime::PollPwmOffsetsCompletion(std::uint32_t now_ms) {
  if (!pwm_offsets_slot_.occupied) {
    return false;
  }
  PwmOffsetsReply reply{};
  const bool complete =
      hooks_.poll_pwm_offsets(hooks_.context, pwm_offsets_slot_.token, &reply);
  if (pwm_offsets_slot_.response_sent) {
    if (complete) {
      SaturatingIncrement(&counters_.late_response_drops);
      pwm_offsets_slot_ = {};
    }
    return false;
  }
  if (!complete && !DeadlineReached(now_ms, pwm_offsets_slot_.deadline_ms)) {
    return false;
  }
  if (!complete) {
    reply.result = TimeoutResult();
    SaturatingIncrement(&counters_.service_timeouts);
  }
  auto& response = service_messages_.pwm_offsets_response;
  response = {};
  SetWireResult(&response, reply.result);
  response.applied_mask = reply.applied_mask;
  const bool sent = SendServiceResponse(ToIndex(ServiceIndex::kPwmOffsets),
                                        &pwm_offsets_slot_.request_id,
                                        &response, reply.result);
  if (complete || !sent) {
    pwm_offsets_slot_ = {};
  } else {
    pwm_offsets_slot_.response_sent = true;
  }
  return true;
}

bool MicroRosRuntime::PollBatteryThresholdCompletion(std::uint32_t now_ms) {
  if (!battery_threshold_slot_.occupied) {
    return false;
  }
  BatteryThresholdReply reply{};
  const bool complete = hooks_.poll_battery_threshold(
      hooks_.context, battery_threshold_slot_.token, &reply);
  if (battery_threshold_slot_.response_sent) {
    if (complete) {
      SaturatingIncrement(&counters_.late_response_drops);
      battery_threshold_slot_ = {};
    }
    return false;
  }
  if (!complete &&
      !DeadlineReached(now_ms, battery_threshold_slot_.deadline_ms)) {
    return false;
  }
  if (!complete) {
    reply.result = TimeoutResult();
    SaturatingIncrement(&counters_.service_timeouts);
  }
  auto& response = service_messages_.battery_threshold_response;
  response = {};
  SetWireResult(&response, reply.result);
  response.active_threshold_mv = reply.active_threshold_mv;
  const bool sent = SendServiceResponse(
      ToIndex(ServiceIndex::kBatteryThreshold),
      &battery_threshold_slot_.request_id, &response, reply.result);
  if (complete || !sent) {
    battery_threshold_slot_ = {};
  } else {
    battery_threshold_slot_.response_sent = true;
  }
  return true;
}

bool MicroRosRuntime::PollBusCompletion(std::uint32_t now_ms) {
  if (!bus_service_slot_.occupied) {
    return false;
  }

  bool complete = false;
  mentor_pi::mcu::Result result{};
  void* response = nullptr;
  std::size_t service_index = 0U;
  switch (bus_service_slot_.kind) {
    case BusServiceKind::kGetState: {
      GetBusServoStateReply reply{};
      complete = hooks_.poll_bus_get_state(hooks_.context,
                                           bus_service_slot_.token, &reply);
      if (!complete &&
          !DeadlineReached(now_ms, bus_service_slot_.deadline_ms)) {
        return false;
      }
      if (!complete) {
        reply.result = TimeoutResult();
      }
      auto& wire = service_messages_.bus_get_response;
      wire = {};
      SetWireResult(&wire, reply.result);
      wire.state.valid_fields = reply.state.valid_fields;
      wire.state.requested_id = reply.state.requested_id;
      wire.state.reported_id = reply.state.reported_id;
      wire.state.position = reply.state.position;
      wire.state.offset = reply.state.offset;
      wire.state.voltage_mv = reply.state.voltage_mv;
      wire.state.temperature_c = reply.state.temperature_c;
      wire.state.position_min = reply.state.position_min;
      wire.state.position_max = reply.state.position_max;
      wire.state.voltage_min_mv = reply.state.voltage_min_mv;
      wire.state.voltage_max_mv = reply.state.voltage_max_mv;
      wire.state.temperature_limit_c = reply.state.temperature_limit_c;
      wire.state.torque_enabled = reply.state.torque_enabled;
      result = reply.result;
      response = &wire;
      service_index = ToIndex(ServiceIndex::kBusGetState);
      break;
    }
    case BusServiceKind::kConfigure: {
      ConfigureBusServoReply reply{};
      complete = hooks_.poll_bus_configure(hooks_.context,
                                           bus_service_slot_.token, &reply);
      if (!complete &&
          !DeadlineReached(now_ms, bus_service_slot_.deadline_ms)) {
        return false;
      }
      if (!complete) {
        reply.result = TimeoutResult();
      }
      auto& wire = service_messages_.bus_configure_response;
      wire = {};
      SetWireResult(&wire, reply.result);
      wire.applied_mask = reply.applied_mask;
      wire.effective_id = reply.effective_id;
      result = reply.result;
      response = &wire;
      service_index = ToIndex(ServiceIndex::kBusConfigure);
      break;
    }
    case BusServiceKind::kStop: {
      StopBusServosReply reply{};
      complete =
          hooks_.poll_bus_stop(hooks_.context, bus_service_slot_.token, &reply);
      if (!complete &&
          !DeadlineReached(now_ms, bus_service_slot_.deadline_ms)) {
        return false;
      }
      if (!complete) {
        reply.result = TimeoutResult();
      }
      auto& wire = service_messages_.bus_stop_response;
      wire = {};
      SetWireResult(&wire, reply.result);
      wire.commands_transmitted = reply.commands_transmitted;
      result = reply.result;
      response = &wire;
      service_index = ToIndex(ServiceIndex::kBusStop);
      break;
    }
    case BusServiceKind::kNone:
      bus_service_slot_ = {};
      return false;
  }
  if (bus_service_slot_.response_sent) {
    if (complete) {
      SaturatingIncrement(&counters_.late_response_drops);
      bus_service_slot_ = {};
    }
    return false;
  }
  if (!complete) {
    SaturatingIncrement(&counters_.service_timeouts);
  }
  const bool sent = SendServiceResponse(
      service_index, &bus_service_slot_.request_id, response, result);
  if (complete || !sent) {
    bus_service_slot_ = {};
  } else {
    bus_service_slot_.response_sent = true;
  }
  return true;
}

bool MicroRosRuntime::SendServiceResponse(std::size_t service_index,
                                          rmw_request_id_t* request_id,
                                          void* response,
                                          mentor_pi::mcu::Result result) {
  if (lifecycle_.state() != SessionState::kActive ||
      service_index >= services_.size()) {
    SaturatingIncrement(&counters_.late_response_drops);
    return false;
  }
  if (!active_slice_budget_.TryStartBlockingOperation(
          ActiveWorkClass::kService)) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    {mentor_pi::mcu::ResultCode::kIoError,
                     static_cast<std::uint16_t>(service_index)});
    return false;
  }
  if (!result.ok()) {
    counters_.last_error_uptime_ms = NowMs();
    counters_.last_error_detail = result.detail;
    counters_.last_error_code = static_cast<std::uint8_t>(result.code);
    if (service_index == ToIndex(ServiceIndex::kMotorModel) ||
        service_index == ToIndex(ServiceIndex::kMotorAdrc)) {
      counters_.last_error_source = ErrorSource::kMotors;
    } else if (service_index == ToIndex(ServiceIndex::kPwmOffsets)) {
      counters_.last_error_source = ErrorSource::kPwmServos;
    } else if (service_index == ToIndex(ServiceIndex::kBatteryThreshold)) {
      counters_.last_error_source = ErrorSource::kBattery;
    } else {
      counters_.last_error_source = ErrorSource::kBusServos;
    }
  }
  const auto send = static_cast<rcl_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kSendResponse, kReliableOperationTimeoutMs,
      [this, service_index, request_id, response]() {
        return static_cast<std::int32_t>(
            rcl_send_response(&services_[service_index], request_id, response));
      }));
  if (arena_.invariant_violated()) {
    RequestTeardown(TeardownReason::kMemoryViolation, ErrorSource::kMemory,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return false;
  }
  if (send != RCL_RET_OK) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(send));
    return false;
  }
  SaturatingIncrement(&counters_.service_completions);
  if (result.code == mentor_pi::mcu::ResultCode::kPartial) {
    SaturatingIncrement(&counters_.service_partial_results);
  }
  return true;
}

}  // namespace mentor_pi_mcu::app::microros
