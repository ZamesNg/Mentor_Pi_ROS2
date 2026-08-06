// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "runtime_internal.h"

extern "C" {
#include "rcl/node_options.h"
#include "rmw/qos_profiles.h"
#include "rmw_microros/rmw_microros.h"
}

#include "mentor_pi_mcu/app/microros/transport_adapter.h"

namespace mentor_pi_mcu::app::microros {
namespace {

template <typename Integer>
void SaturatingIncrement(Integer* value) {
  if (*value != std::numeric_limits<Integer>::max()) {
    ++(*value);
  }
}

mentor_pi::mcu::Result RclError(rcl_ret_t result) {
  return {mentor_pi::mcu::ResultCode::kIoError,
          static_cast<std::uint16_t>(result)};
}

rmw_qos_profile_t TopicQos(bool reliable, std::size_t depth) {
  rmw_qos_profile_t profile = rmw_qos_profile_default;
  profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  profile.depth = depth;
  profile.reliability = reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                 : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return profile;
}

rmw_qos_profile_t ServiceQos() {
  rmw_qos_profile_t profile = rmw_qos_profile_services_default;
  profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  profile.depth = 1U;
  profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return profile;
}

const rosidl_message_type_support_t* PublisherTypeSupport(std::size_t index) {
  switch (static_cast<PublisherIndex>(index)) {
    case PublisherIndex::kMotors:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg, MotorState);
    case PublisherIndex::kPwmServos:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         PwmServoState);
    case PublisherIndex::kImu:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg, ImuState);
    case PublisherIndex::kButtons:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         ButtonEvent);
    case PublisherIndex::kBattery:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         BatteryState);
    case PublisherIndex::kHeartbeat:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg, Heartbeat);
    case PublisherIndex::kDiagnostics:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         ControllerDiagnostics);
    case PublisherIndex::kCount:
      return nullptr;
  }
  return nullptr;
}

const rosidl_message_type_support_t* SubscriptionTypeSupport(
    std::size_t index) {
  switch (static_cast<SubscriptionIndex>(index)) {
    case SubscriptionIndex::kMotors:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         MotorCommand);
    case SubscriptionIndex::kPwmServos:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         PwmServoCommand);
    case SubscriptionIndex::kBusServos:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         BusServoCommand);
    case SubscriptionIndex::kLeds:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg, LedCommand);
    case SubscriptionIndex::kBuzzer:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         BuzzerCommand);
    case SubscriptionIndex::kRgb:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg, RgbCommand);
    case SubscriptionIndex::kOled:
      return ROSIDL_GET_MSG_TYPE_SUPPORT(mentor_pi_interfaces, msg,
                                         OledCommand);
    case SubscriptionIndex::kCount:
      return nullptr;
  }
  return nullptr;
}

const rosidl_service_type_support_t* ServiceTypeSupport(std::size_t index) {
  switch (static_cast<ServiceIndex>(index)) {
    case ServiceIndex::kMotorModel:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         SetMotorModel);
    case ServiceIndex::kPwmOffsets:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         SetPwmServoOffsets);
    case ServiceIndex::kBusGetState:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         GetBusServoState);
    case ServiceIndex::kBusConfigure:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         ConfigureBusServo);
    case ServiceIndex::kBusStop:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         StopBusServos);
    case ServiceIndex::kBatteryThreshold:
      return ROSIDL_GET_SRV_TYPE_SUPPORT(mentor_pi_interfaces, srv,
                                         SetBatteryThreshold);
    case ServiceIndex::kCount:
      return nullptr;
  }
  return nullptr;
}

MicroRosRuntime g_runtime;

}  // namespace

MicroRosRuntime& RuntimeInstance() { return g_runtime; }

bool MicroRosRuntime::Configure(const RuntimeHooks& hooks) {
  if (configured_ || !RuntimeHooksAreComplete(hooks)) {
    return false;
  }
  hooks_ = hooks;
  if (!arena_.Initialize()) {
    return false;
  }
  allocator_ = arena_.allocator();
  if (!arena_.InstallAsRcutilsDefault() || !ConfigureCustomTransport()) {
    return false;
  }
  last_monotonic_ms_ = NowMs();
  configured_ = true;
  return true;
}

void MicroRosRuntime::RunOnce() {
  if (!configured_) {
    return;
  }
  // A sealed-arena violation is a firmware invariant failure, not a
  // reconnectable session error. Allow the already-requested best-effort
  // teardown to run once, then stop advancing this task's heartbeat so the
  // safety supervisor withholds IWDG refresh and the MCU returns to safe boot.
  if (fatal_memory_violation_ &&
      lifecycle_.state() != SessionState::kTeardown) {
    hooks_.emergency_stop_motors(hooks_.context);
    return;
  }
  const std::uint32_t now_ms = NowMs();
  UpdateExtendedMonotonic(now_ms);
  switch (lifecycle_.state()) {
    case SessionState::kSafeBoot:
      SafeBootStep(now_ms);
      break;
    case SessionState::kWaitAgent:
      WaitAgentStep(now_ms);
      break;
    case SessionState::kCreateEntities:
      CreateEntitiesStep(now_ms);
      break;
    case SessionState::kActive:
      ActiveStep(now_ms);
      break;
    case SessionState::kTeardown:
      TeardownStep(now_ms);
      break;
    case SessionState::kBackoff:
      BackoffStep(now_ms);
      break;
  }
  if (!fatal_memory_violation_) {
    AdvanceHeartbeat();
  }
}

[[noreturn]] void MicroRosRuntime::RunForever() {
  for (;;) {
    RunOnce();
    const std::uint32_t wait_ms =
        lifecycle_.state() == SessionState::kBackoff ? 10U : 1U;
    hooks_.wait_milliseconds(hooks_.context, wait_ms);
  }
}

void MicroRosRuntime::SafeBootStep(std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  hooks_.set_session_active(hooks_.context, false, 0U);
  hooks_.emergency_stop_motors(hooks_.context);
  hooks_.invalidate_session_work(hooks_.context, 0U);
  lifecycle_.LeaveSafeBoot();
}

void MicroRosRuntime::WaitAgentStep(std::uint32_t now_ms) {
  // Each disconnected ping opens a fresh physical transport instance whose
  // platform error latch starts clear, so diagnostics must account its flags
  // independently from the preceding session or probe.
  accounted_transport_flags_ = 0U;
  const auto ping = static_cast<rmw_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kAgentPing, kWaitAgentPingTimeoutMs, []() {
        return static_cast<std::int32_t>(
            rmw_uros_ping_agent(static_cast<int>(kWaitAgentPingTimeoutMs), 1U));
      }));
  const bool available = ping == RMW_RET_OK;
  const auto transport = ReadTransportSnapshot();
  if (transport.error_flags != 0U) {
    RecordTransportFault(transport.error_flags);
  }
  AdvanceHeartbeat();
  lifecycle_.OnAgentPing(available, now_ms);
}

void MicroRosRuntime::CreateEntitiesStep(std::uint32_t now_ms) {
  if (!create_cursor_.started()) {
    PrepareCreateEntities(now_ms);
  }
  const EntityCreateStep step = create_cursor_.current();
  if (!create_cursor_.CurrentOperationFits(NowMs())) {
    FailCreateEntities({mentor_pi::mcu::ResultCode::kTimeout,
                        static_cast<std::uint16_t>(step.kind)});
    return;
  }

  const std::int32_t result = ExecuteCreateStep(step);
  if (result != static_cast<std::int32_t>(RCL_RET_OK)) {
    FailCreateEntities(RclError(static_cast<rcl_ret_t>(result)));
    return;
  }
  create_cursor_.Advance();
  if (create_cursor_.PhaseDeadlineReached(NowMs())) {
    FailCreateEntities({mentor_pi::mcu::ResultCode::kTimeout,
                        static_cast<std::uint16_t>(step.kind)});
    return;
  }
  if (!create_cursor_.complete()) {
    return;
  }

  arena_.SealActive();
  if (arena_.invariant_violated()) {
    create_cursor_.Clear();
    RequestTeardown(TeardownReason::kMemoryViolation, ErrorSource::kMemory,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return;
  }
  create_cursor_.Clear();
  lifecycle_.OnEntitiesCreated(true, NowMs());
  InitializeSchedules(NowMs());
  hooks_.set_session_active(hooks_.context, true,
                            lifecycle_.session_generation());
}

void MicroRosRuntime::ActiveStep(std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  const ActiveWorkClass work_class = active_work_scheduler_.Next();
  active_slice_budget_.Reset(work_class);
  const auto before = ReadTransportSnapshot();
  if (before.error_flags != 0U) {
    RecordTransportFault(before.error_flags);
    RequestTeardown(ClassifyTransportFault(before.error_flags),
                    ErrorSource::kTransport,
                    {mentor_pi::mcu::ResultCode::kIoError, before.error_flags});
    return;
  }
  if (arena_.invariant_violated()) {
    RequestTeardown(TeardownReason::kMemoryViolation, ErrorSource::kMemory,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return;
  }

  const std::uint32_t spin_start_ms = NowMs();
  const auto spin = static_cast<rcl_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kExecutorSpin, kExecutorWaitMs, [this]() {
        return static_cast<std::int32_t>(
            rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(kExecutorWaitMs)));
      }));
  const std::uint32_t after_spin_ms = NowMs();
  if (after_spin_ms - spin_start_ms > 2U) {
    SaturatingIncrement(&counters_.executor_overruns);
  }
  if (spin != RCL_RET_OK && spin != RCL_RET_TIMEOUT) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    RclError(spin));
    return;
  }
  if (arena_.invariant_violated()) {
    RequestTeardown(TeardownReason::kMemoryViolation, ErrorSource::kMemory,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return;
  }

  if (work_class == ActiveWorkClass::kService) {
    PumpServices(after_spin_ms);
    if (lifecycle_.state() != SessionState::kActive) {
      return;
    }
  }
  PublishDueTelemetry(NowMs(),
                      work_class == ActiveWorkClass::kReliableTelemetry);
  if (lifecycle_.state() != SessionState::kActive) {
    return;
  }
  if (work_class == ActiveWorkClass::kMaintenance) {
    RunOneMaintenanceOperation(NowMs());
    if (lifecycle_.state() != SessionState::kActive) {
      return;
    }
  }

  const auto after = ReadTransportSnapshot();
  if (after.error_flags != 0U) {
    RecordTransportFault(after.error_flags);
    RequestTeardown(ClassifyTransportFault(after.error_flags),
                    ErrorSource::kTransport,
                    {mentor_pi::mcu::ResultCode::kIoError, after.error_flags});
  }
}

void MicroRosRuntime::RunOneMaintenanceOperation(std::uint32_t now_ms) {
  if (now_ms - last_agent_ping_ms_ >= kActivePingPeriodMs) {
    if (!active_slice_budget_.TryStartBlockingOperation(
            ActiveWorkClass::kMaintenance)) {
      RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                      {mentor_pi::mcu::ResultCode::kIoError, 0U});
      return;
    }
    last_agent_ping_ms_ = now_ms;
    const auto ping = static_cast<rmw_ret_t>(InvokeMiddleware(
        MiddlewareBoundary::kAgentPing, kActivePingTimeoutMs, []() {
          return static_cast<std::int32_t>(
              rmw_uros_ping_agent(static_cast<int>(kActivePingTimeoutMs), 1U));
        }));
    if (ping == RMW_RET_OK) {
      consecutive_ping_failures_ = 0U;
    } else {
      ++consecutive_ping_failures_;
      if (consecutive_ping_failures_ >= kActivePingFailureLimit) {
        RequestTeardown(TeardownReason::kAgentLost, ErrorSource::kTransport,
                        {mentor_pi::mcu::ResultCode::kTimeout, 0U});
        return;
      }
    }
    AdvanceHeartbeat();
    return;
  }

  const std::uint32_t sync_period =
      !time_synchronized_ || time_sync_retry_pending_ ? kTimeSyncRetryMs
                                                      : kTimeResyncPeriodMs;
  if (now_ms - last_time_sync_attempt_ms_ < sync_period) {
    return;
  }
  if (!active_slice_budget_.TryStartBlockingOperation(
          ActiveWorkClass::kMaintenance)) {
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return;
  }
  static_cast<void>(SynchronizeTime(now_ms, kActiveTimeSyncTimeoutMs, false));
}

void MicroRosRuntime::TeardownStep(std::uint32_t now_ms) {
  if (!destroy_cursor_.started()) {
    PrepareDestroyEntities(now_ms);
  }

  // Skipping a non-constructed object is local, fixed work. Continue until this
  // slice either starts exactly one middleware boundary or completes cleanup.
  while (!destroy_cursor_.complete()) {
    EntityDestroyStep step = destroy_cursor_.current();
    if (!destroy_cursor_.remote_waits_abandoned() &&
        step.kind != EntityDestroyStepKind::kDisableRemoteWaits &&
        !destroy_cursor_.CurrentRemoteOperationFits(NowMs())) {
      if (support_constructed_) {
        destroy_cursor_.RequestRemoteWaitAbandon();
      } else {
        destroy_cursor_.MarkRemoteWaitsAbandonedWithoutBoundary();
      }
      continue;
    }

    step = destroy_cursor_.current();
    if (!DestroyStepIsApplicable(step)) {
      destroy_cursor_.Advance();
      continue;
    }

    const rcl_ret_t result = ExecuteDestroyStep(step);
    if (step.kind != EntityDestroyStepKind::kSetContextDestroyTimeout &&
        step.kind != EntityDestroyStepKind::kDisableRemoteWaits) {
      RecordDestroyResult(result);
    }
    destroy_cursor_.Advance();
    if (destroy_cursor_.complete()) {
      CompleteDestroyEntities();
    }
    return;
  }

  CompleteDestroyEntities();
}

void MicroRosRuntime::BackoffStep(std::uint32_t now_ms) {
  static_cast<void>(lifecycle_.AdvanceBackoff(now_ms));
}

void MicroRosRuntime::PrepareCreateEntities(std::uint32_t now_ms) {
  create_cursor_.Reset(now_ms);
  arena_.PrepareForCreate();
  InitializeRosStorage();
  support_ = {};
  node_ = rcl_get_zero_initialized_node();
  for (auto& publisher : publishers_) {
    publisher = rcl_get_zero_initialized_publisher();
  }
  for (auto& subscription : subscriptions_) {
    subscription = rcl_get_zero_initialized_subscription();
  }
  for (auto& service : services_) {
    service = rcl_get_zero_initialized_service();
  }
  executor_ = rclc_executor_get_zero_initialized_executor();
  publisher_constructed_.fill(false);
  subscription_constructed_.fill(false);
  service_constructed_.fill(false);
  support_constructed_ = false;
  node_constructed_ = false;
  executor_constructed_ = false;
  accounted_transport_flags_ = 0U;
  consecutive_ping_failures_ = 0U;
  time_synchronized_ = false;
  time_offset_initialized_ = false;
  time_sync_retry_pending_ = false;
  last_stamp_ns_.fill(0);
}

std::int32_t MicroRosRuntime::ExecuteCreateStep(const EntityCreateStep& step) {
  switch (step.kind) {
    case EntityCreateStepKind::kSupportInit: {
      const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kEntityCreate, step.deadline_ms, [this]() {
            return static_cast<std::int32_t>(
                rclc_support_init(&support_, 0, nullptr, &allocator_));
          }));
      support_constructed_ = result == RCL_RET_OK;
      return static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kSetContextCreateTimeout: {
      rmw_context_t* const context =
          rcl_context_get_rmw_context(&support_.context);
      const auto result = static_cast<rmw_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kEntityCreate, step.deadline_ms, [context]() {
            return static_cast<std::int32_t>(
                rmw_uros_set_context_entity_creation_session_timeout(
                    context, static_cast<int>(kCreateCallTimeoutMs)));
          }));
      return static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kSetContextDestroyTimeout: {
      rmw_context_t* const context =
          rcl_context_get_rmw_context(&support_.context);
      const auto result = static_cast<rmw_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kEntityCreate, step.deadline_ms, [context]() {
            return static_cast<std::int32_t>(
                rmw_uros_set_context_entity_destroy_session_timeout(
                    context, static_cast<int>(kDestroyCallTimeoutMs)));
          }));
      return static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kNodeInit: {
      rcl_node_options_t options = rcl_node_get_default_options();
      options.enable_rosout = false;
      const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kEntityCreate, step.deadline_ms,
          [this, &options]() {
            return static_cast<std::int32_t>(rclc_node_init_with_options(
                &node_, "controller", "/mentor_pi", &support_, &options));
          }));
      node_constructed_ = result == RCL_RET_OK;
      return static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kPublisherInit:
      if (step.index >= publishers_.size()) {
        return static_cast<std::int32_t>(RCL_RET_ERROR);
      }
      return CreatePublisher(step.index, PublisherTypeSupport(step.index),
                             kPublisherEndpoints[step.index]);
    case EntityCreateStepKind::kSetPublisherTimeout:
      return SetPublisherTimeout(step.index);
    case EntityCreateStepKind::kSubscriptionInit:
      if (step.index >= subscriptions_.size()) {
        return static_cast<std::int32_t>(RCL_RET_ERROR);
      }
      return CreateSubscription(step.index, SubscriptionTypeSupport(step.index),
                                kSubscriptionEndpoints[step.index]);
    case EntityCreateStepKind::kServiceInit:
      if (step.index >= services_.size()) {
        return static_cast<std::int32_t>(RCL_RET_ERROR);
      }
      return CreateService(step.index, ServiceTypeSupport(step.index),
                           kServiceEndpoints[step.index]);
    case EntityCreateStepKind::kSetServiceTimeout:
      return SetServiceTimeout(step.index);
    case EntityCreateStepKind::kExecutorInit: {
      const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kEntityCreate, step.deadline_ms, [this]() {
            return static_cast<std::int32_t>(
                rclc_executor_init(&executor_, &support_.context,
                                   kSubscriptionCount, &allocator_));
          }));
      executor_constructed_ = result == RCL_RET_OK;
      return static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kExecutorAddSubscription:
      return AddExecutorSubscription(step.index);
    case EntityCreateStepKind::kExecutorPrime: {
      // Humble rclc lazily allocates its wait set on the first spin. Prime it
      // while the creation arena is open so ACTIVE can enforce a hard seal.
      const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
          MiddlewareBoundary::kExecutorSpin, step.deadline_ms, [this]() {
            return static_cast<std::int32_t>(
                rclc_executor_spin_some(&executor_, 0U));
          }));
      return result == RCL_RET_OK || result == RCL_RET_TIMEOUT
                 ? static_cast<std::int32_t>(RCL_RET_OK)
                 : static_cast<std::int32_t>(result);
    }
    case EntityCreateStepKind::kInitialTimeSync:
      static_cast<void>(SynchronizeTime(NowMs(), step.deadline_ms, true));
      return static_cast<std::int32_t>(RCL_RET_OK);
    case EntityCreateStepKind::kComplete:
      return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  return static_cast<std::int32_t>(RCL_RET_ERROR);
}

void MicroRosRuntime::FailCreateEntities(mentor_pi::mcu::Result result) {
  create_cursor_.Clear();
  RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor, result);
}

std::int32_t MicroRosRuntime::CreatePublisher(
    std::size_t index, const rosidl_message_type_support_t* type_support,
    const TopicEndpoint& endpoint) {
  if (index >= publishers_.size() || type_support == nullptr) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  const rmw_qos_profile_t qos =
      TopicQos(endpoint.reliability == Reliability::kReliable, endpoint.depth);
  const auto result = static_cast<rcl_ret_t>(
      InvokeMiddleware(MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs,
                       [this, index, type_support, &endpoint, &qos]() {
                         return static_cast<std::int32_t>(rclc_publisher_init(
                             &publishers_[index], &node_, type_support,
                             endpoint.relative_name, &qos));
                       }));
  publisher_constructed_[index] = result == RCL_RET_OK;
  return static_cast<std::int32_t>(result);
}

std::int32_t MicroRosRuntime::SetPublisherTimeout(std::size_t index) {
  if (index >= publishers_.size() || !publisher_constructed_[index]) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  rmw_publisher_t* const rmw_publisher =
      rcl_publisher_get_rmw_handle(&publishers_[index]);
  const auto timeout_result = static_cast<rmw_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs,
      [rmw_publisher]() {
        return static_cast<std::int32_t>(rmw_uros_set_publisher_session_timeout(
            rmw_publisher, static_cast<int>(kReliableOperationTimeoutMs)));
      }));
  return static_cast<std::int32_t>(timeout_result);
}

std::int32_t MicroRosRuntime::CreateSubscription(
    std::size_t index, const rosidl_message_type_support_t* type_support,
    const TopicEndpoint& endpoint) {
  if (index >= subscriptions_.size() || type_support == nullptr) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  const rmw_qos_profile_t qos =
      TopicQos(endpoint.reliability == Reliability::kReliable, endpoint.depth);
  const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs,
      [this, index, type_support, &endpoint, &qos]() {
        return static_cast<std::int32_t>(
            rclc_subscription_init(&subscriptions_[index], &node_, type_support,
                                   endpoint.relative_name, &qos));
      }));
  subscription_constructed_[index] = result == RCL_RET_OK;
  return static_cast<std::int32_t>(result);
}

std::int32_t MicroRosRuntime::CreateService(
    std::size_t index, const rosidl_service_type_support_t* type_support,
    const ServiceEndpoint& endpoint) {
  if (index >= services_.size() || type_support == nullptr) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  const rmw_qos_profile_t qos = ServiceQos();
  const auto result = static_cast<rcl_ret_t>(
      InvokeMiddleware(MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs,
                       [this, index, type_support, &endpoint, &qos]() {
                         return static_cast<std::int32_t>(rclc_service_init(
                             &services_[index], &node_, type_support,
                             endpoint.relative_name, &qos));
                       }));
  service_constructed_[index] = result == RCL_RET_OK;
  return static_cast<std::int32_t>(result);
}

std::int32_t MicroRosRuntime::SetServiceTimeout(std::size_t index) {
  if (index >= services_.size() || !service_constructed_[index]) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  rmw_service_t* const rmw_service =
      rcl_service_get_rmw_handle(&services_[index]);
  const auto timeout_result = static_cast<rmw_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs, [rmw_service]() {
        return static_cast<std::int32_t>(rmw_uros_set_service_session_timeout(
            rmw_service, static_cast<int>(kReliableOperationTimeoutMs)));
      }));
  return static_cast<std::int32_t>(timeout_result);
}

std::int32_t MicroRosRuntime::AddExecutorSubscription(std::size_t index) {
  if (index >= subscriptions_.size()) {
    return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  void* message = nullptr;
  void (*callback)(const void*) = nullptr;
  switch (static_cast<SubscriptionIndex>(index)) {
    case SubscriptionIndex::kMotors:
      message = &subscription_messages_.motors;
      callback = &MotorSubscriptionCallback;
      break;
    case SubscriptionIndex::kPwmServos:
      message = &subscription_messages_.pwm_servos;
      callback = &PwmServoSubscriptionCallback;
      break;
    case SubscriptionIndex::kBusServos:
      message = &subscription_messages_.bus_servos;
      callback = &BusServoSubscriptionCallback;
      break;
    case SubscriptionIndex::kLeds:
      message = &subscription_messages_.leds;
      callback = &LedSubscriptionCallback;
      break;
    case SubscriptionIndex::kBuzzer:
      message = &subscription_messages_.buzzer;
      callback = &BuzzerSubscriptionCallback;
      break;
    case SubscriptionIndex::kRgb:
      message = &subscription_messages_.rgb;
      callback = &RgbSubscriptionCallback;
      break;
    case SubscriptionIndex::kOled:
      message = &subscription_messages_.oled;
      callback = &OledSubscriptionCallback;
      break;
    case SubscriptionIndex::kCount:
      return static_cast<std::int32_t>(RCL_RET_ERROR);
  }
  const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kEntityCreate, kCreateCallTimeoutMs,
      [this, index, message, callback]() {
        return static_cast<std::int32_t>(
            rclc_executor_add_subscription(&executor_, &subscriptions_[index],
                                           message, callback, ON_NEW_DATA));
      }));
  return static_cast<std::int32_t>(result);
}

void MicroRosRuntime::PrepareDestroyEntities(std::uint32_t now_ms) {
  hooks_.set_session_active(hooks_.context, false,
                            lifecycle_.session_generation());
  hooks_.emergency_stop_motors(hooks_.context);
  hooks_.invalidate_session_work(hooks_.context,
                                 lifecycle_.session_generation());
  ClearServiceSlots();
  arena_.BeginDestroy();
  destroy_cursor_.Reset(now_ms);
}

bool MicroRosRuntime::DestroyStepIsApplicable(
    const EntityDestroyStep& step) const {
  switch (step.kind) {
    case EntityDestroyStepKind::kSetContextDestroyTimeout:
    case EntityDestroyStepKind::kDisableRemoteWaits:
      return support_constructed_;
    case EntityDestroyStepKind::kExecutorFini:
      return executor_constructed_;
    case EntityDestroyStepKind::kServiceFini:
      return step.index < service_constructed_.size() &&
             service_constructed_[step.index];
    case EntityDestroyStepKind::kSubscriptionFini:
      return step.index < subscription_constructed_.size() &&
             subscription_constructed_[step.index];
    case EntityDestroyStepKind::kPublisherFini:
      return step.index < publisher_constructed_.size() &&
             publisher_constructed_[step.index];
    case EntityDestroyStepKind::kNodeFini:
      return node_constructed_;
    case EntityDestroyStepKind::kSupportFini:
      return support_constructed_;
    case EntityDestroyStepKind::kComplete:
      return false;
  }
  return false;
}

rcl_ret_t MicroRosRuntime::ExecuteDestroyStep(const EntityDestroyStep& step) {
  const auto invoke = [this, &step](auto operation) {
    return static_cast<rcl_ret_t>(InvokeMiddleware(
        MiddlewareBoundary::kEntityFinalize, step.deadline_ms, operation));
  };
  switch (step.kind) {
    case EntityDestroyStepKind::kSetContextDestroyTimeout: {
      rmw_context_t* const context =
          rcl_context_get_rmw_context(&support_.context);
      return invoke([context]() {
        return static_cast<std::int32_t>(
            rmw_uros_set_context_entity_destroy_session_timeout(
                context, static_cast<int>(kDestroyCallTimeoutMs)));
      });
    }
    case EntityDestroyStepKind::kExecutorFini: {
      const rcl_ret_t result = invoke([this]() {
        return static_cast<std::int32_t>(rclc_executor_fini(&executor_));
      });
      executor_constructed_ = false;
      return result;
    }
    case EntityDestroyStepKind::kServiceFini: {
      const rcl_ret_t result = invoke([this, &step]() {
        return static_cast<std::int32_t>(
            rcl_service_fini(&services_[step.index], &node_));
      });
      service_constructed_[step.index] = false;
      return result;
    }
    case EntityDestroyStepKind::kSubscriptionFini: {
      const rcl_ret_t result = invoke([this, &step]() {
        return static_cast<std::int32_t>(
            rcl_subscription_fini(&subscriptions_[step.index], &node_));
      });
      subscription_constructed_[step.index] = false;
      return result;
    }
    case EntityDestroyStepKind::kPublisherFini: {
      const rcl_ret_t result = invoke([this, &step]() {
        return static_cast<std::int32_t>(
            rcl_publisher_fini(&publishers_[step.index], &node_));
      });
      publisher_constructed_[step.index] = false;
      return result;
    }
    case EntityDestroyStepKind::kNodeFini: {
      const rcl_ret_t result = invoke([this]() {
        return static_cast<std::int32_t>(rcl_node_fini(&node_));
      });
      node_constructed_ = false;
      return result;
    }
    case EntityDestroyStepKind::kSupportFini: {
      const rcl_ret_t result = invoke([this]() {
        return static_cast<std::int32_t>(rclc_support_fini(&support_));
      });
      support_constructed_ = false;
      return result;
    }
    case EntityDestroyStepKind::kDisableRemoteWaits: {
      rmw_context_t* const context =
          rcl_context_get_rmw_context(&support_.context);
      return invoke([context]() {
        return static_cast<std::int32_t>(
            rmw_uros_set_context_entity_destroy_session_timeout(context, 0));
      });
    }
    case EntityDestroyStepKind::kComplete:
      return RCL_RET_ERROR;
  }
  return RCL_RET_ERROR;
}

void MicroRosRuntime::RecordDestroyResult(rcl_ret_t result) {
  if (result == RCL_RET_OK) {
    return;
  }
  counters_.last_error_uptime_ms = NowMs();
  counters_.last_error_detail = static_cast<std::uint16_t>(result);
  counters_.last_error_code =
      static_cast<std::uint8_t>(mentor_pi::mcu::ResultCode::kIoError);
  counters_.last_error_source = ErrorSource::kExecutor;
}

void MicroRosRuntime::CompleteDestroyEntities() {
  CloseCustomTransport();
  arena_.ResetAfterDestroy();
  InitializeRosStorage();
  time_synchronized_ = false;
  time_offset_initialized_ = false;
  time_sync_retry_pending_ = false;
  last_stamp_ns_.fill(0);
  create_cursor_.Clear();
  destroy_cursor_.Clear();
  if (arena_.invariant_violated()) {
    fatal_memory_violation_ = true;
  }
  lifecycle_.OnTeardownComplete(NowMs());
}

void MicroRosRuntime::InitializeRosStorage() {
  subscription_messages_ = {};
  publication_messages_ = {};
  service_messages_ = {};
  subscription_messages_.oled.line_1.data =
      subscription_messages_.oled_line_1.data();
  subscription_messages_.oled.line_1.size = 0U;
  subscription_messages_.oled.line_1.capacity =
      subscription_messages_.oled_line_1.size();
  subscription_messages_.oled.line_2.data =
      subscription_messages_.oled_line_2.data();
  subscription_messages_.oled.line_2.size = 0U;
  subscription_messages_.oled.line_2.capacity =
      subscription_messages_.oled_line_2.size();
}

bool MicroRosRuntime::SynchronizeTime(std::uint32_t now_ms,
                                      std::uint32_t timeout_ms,
                                      bool initial_attempt) {
  static_cast<void>(initial_attempt);
  last_time_sync_attempt_ms_ = now_ms;
  const auto result = static_cast<rmw_ret_t>(InvokeMiddleware(
      MiddlewareBoundary::kTimeSync, timeout_ms, [timeout_ms]() {
        return static_cast<std::int32_t>(
            rmw_uros_sync_session(static_cast<int>(timeout_ms)));
      }));
  AdvanceHeartbeat();
  if (result != RMW_RET_OK) {
    time_sync_retry_pending_ = true;
    return false;
  }
  const std::int64_t epoch_ns = rmw_uros_epoch_nanos();
  if (epoch_ns <= 0) {
    time_sync_retry_pending_ = true;
    return false;
  }
  UpdateExtendedMonotonic(NowMs());
  const std::int64_t monotonic_ns =
      static_cast<std::int64_t>(extended_monotonic_ms_ * 1000000ULL);
  const std::int64_t candidate_offset = epoch_ns - monotonic_ns;
  // Install both positive and negative corrections. SetStamp() clamps each
  // topic to its last emitted value, so a negative correction is naturally
  // deferred (the topic pauses until corrected epoch time catches up) rather
  // than being ignored forever or making a timestamp regress.
  epoch_offset_ns_ = candidate_offset;
  time_offset_initialized_ = true;
  time_synchronized_ = true;
  time_sync_retry_pending_ = false;
  last_time_sync_success_ms_ = NowMs();
  return true;
}

void MicroRosRuntime::SetStamp(std::uint32_t source_ms,
                               std::size_t publisher_index,
                               builtin_interfaces__msg__Time* stamp) {
  if (!time_synchronized_ || !time_offset_initialized_) {
    stamp->sec = 0;
    stamp->nanosec = 0U;
    return;
  }
  const std::uint64_t extended_source = ExtendTimestamp(source_ms);
  std::int64_t stamp_ns = epoch_offset_ns_ + static_cast<std::int64_t>(
                                                 extended_source * 1000000ULL);
  stamp_ns = std::max<std::int64_t>(stamp_ns, 0);
  stamp_ns = std::max(stamp_ns, last_stamp_ns_[publisher_index]);
  last_stamp_ns_[publisher_index] = stamp_ns;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  const std::int64_t seconds = stamp_ns / kNanosecondsPerSecond;
  stamp->sec = seconds > std::numeric_limits<std::int32_t>::max()
                   ? std::numeric_limits<std::int32_t>::max()
                   : static_cast<std::int32_t>(seconds);
  stamp->nanosec = static_cast<std::uint32_t>(stamp_ns % kNanosecondsPerSecond);
}

std::uint64_t MicroRosRuntime::ExtendTimestamp(
    std::uint32_t timestamp_ms) const {
  const std::int64_t delta =
      static_cast<std::int32_t>(timestamp_ms - last_monotonic_ms_);
  if (delta < 0 &&
      static_cast<std::uint64_t>(-delta) > extended_monotonic_ms_) {
    return 0U;
  }
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(extended_monotonic_ms_) + delta);
}

void MicroRosRuntime::UpdateExtendedMonotonic(std::uint32_t now_ms) {
  const std::uint32_t delta = now_ms - last_monotonic_ms_;
  extended_monotonic_ms_ += delta;
  last_monotonic_ms_ = now_ms;
}

void MicroRosRuntime::RequestTeardown(TeardownReason reason, ErrorSource source,
                                      mentor_pi::mcu::Result result) {
  if (reason != TeardownReason::kMemoryViolation) {
    const auto transport = ReadTransportSnapshot();
    if (transport.error_flags != 0U) {
      RecordTransportFault(transport.error_flags);
      reason = ClassifyTransportFault(transport.error_flags);
      source = ErrorSource::kTransport;
      result = {mentor_pi::mcu::ResultCode::kIoError, transport.error_flags};
    }
  }
  hooks_.set_session_active(hooks_.context, false,
                            lifecycle_.session_generation());
  hooks_.emergency_stop_motors(hooks_.context);
  hooks_.invalidate_session_work(hooks_.context,
                                 lifecycle_.session_generation());
  counters_.last_error_uptime_ms = NowMs();
  counters_.last_error_detail = result.detail;
  counters_.last_error_code = static_cast<std::uint8_t>(result.code);
  counters_.last_error_source = source;
  if (reason == TeardownReason::kMemoryViolation) {
    fatal_memory_violation_ = true;
  }
  lifecycle_.BeginTeardown(reason);
}

void MicroRosRuntime::RecordCommandResult(std::size_t subscription_index,
                                          mentor_pi::mcu::Result result,
                                          bool overwrote_unread,
                                          ErrorSource source) {
  SaturatingIncrement(&counters_.command_messages);
  if (overwrote_unread &&
      subscription_index < counters_.mailbox_overwrites.size()) {
    SaturatingIncrement(&counters_.mailbox_overwrites[subscription_index]);
  }
  if (!result.ok()) {
    SaturatingIncrement(&counters_.command_rejections);
    counters_.last_error_uptime_ms = NowMs();
    counters_.last_error_detail = result.detail;
    counters_.last_error_code = static_cast<std::uint8_t>(result.code);
    counters_.last_error_source = source;
  }
}

void MicroRosRuntime::RecordTransportFault(std::uint8_t flags) {
  const std::uint8_t new_flags =
      static_cast<std::uint8_t>(flags & ~accounted_transport_flags_);
  accounted_transport_flags_ |= flags;
  using mentor_pi_mcu::platform::stm32::Usart1Error;
  const auto bit = [](Usart1Error error) {
    return static_cast<std::uint8_t>(error);
  };
  if ((new_flags & bit(Usart1Error::kFraming)) != 0U) {
    SaturatingIncrement(counters_.usart1_errors.data());
  }
  if ((new_flags & bit(Usart1Error::kNoise)) != 0U) {
    SaturatingIncrement(counters_.usart1_errors.data() + 1U);
  }
  if ((new_flags & bit(Usart1Error::kOverrun)) != 0U) {
    SaturatingIncrement(counters_.usart1_errors.data() + 2U);
  }
  if ((new_flags & bit(Usart1Error::kParity)) != 0U) {
    SaturatingIncrement(counters_.usart1_errors.data() + 3U);
  }
  if ((new_flags & bit(Usart1Error::kRxRingOverrun)) != 0U) {
    SaturatingIncrement(&counters_.transport_rx_overruns);
  }
  if ((new_flags & bit(Usart1Error::kTxTimeout)) != 0U) {
    SaturatingIncrement(&counters_.transport_tx_timeouts);
  }
}

void MicroRosRuntime::ClearServiceSlots() {
  motor_model_slot_ = {};
  pwm_offsets_slot_ = {};
  battery_threshold_slot_ = {};
  bus_service_slot_ = {};
}

ServiceToken MicroRosRuntime::NewServiceToken() {
  request_generation_ = NextNonzeroGeneration(request_generation_);
  return {lifecycle_.session_generation(), request_generation_};
}

bool MicroRosRuntime::IsCurrentToken(ServiceToken token) const {
  return token.session_generation == lifecycle_.session_generation();
}

void MicroRosRuntime::AdvanceHeartbeat() const {
  hooks_.advance_task_heartbeat(hooks_.context);
}

std::uint32_t MicroRosRuntime::NowMs() const {
  return hooks_.monotonic_milliseconds(hooks_.context);
}

std::uint32_t MicroRosRuntime::NowUs() const {
  return hooks_.monotonic_microseconds(hooks_.context);
}

bool ConfigureMicroRosRuntime(const RuntimeHooks& hooks) {
  return RuntimeInstance().Configure(hooks);
}

void RunMicroRosRuntimeOnce() { RuntimeInstance().RunOnce(); }

[[noreturn]] void RunMicroRosRuntime() { RuntimeInstance().RunForever(); }

SessionState MicroRosSessionState() { return RuntimeInstance().state(); }

#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
bool ConfigureMicroRosMiddlewareFaultForTesting(
    const MiddlewareFaultPlan& plan) {
  return RuntimeInstance().ConfigureMiddlewareFaultForTesting(plan);
}

void ClearMicroRosMiddlewareFaultForTesting() {
  RuntimeInstance().ClearMiddlewareFaultForTesting();
}

MiddlewareBoundaryStats MicroRosMiddlewareStatsForTesting(
    MiddlewareBoundary boundary) {
  return RuntimeInstance().MiddlewareStatsForTesting(boundary);
}
#endif

}  // namespace mentor_pi_mcu::app::microros

extern "C" [[noreturn]] void MentorPiMicroRosTaskMain(void* context) {
  static_cast<void>(context);
  mentor_pi_mcu::app::microros::RunMicroRosRuntime();
}
