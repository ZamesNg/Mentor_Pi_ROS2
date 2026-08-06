// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "mentor_pi_mcu/app/microros/transport_adapter.h"
#include "runtime_internal.h"

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::uint32_t kMotorPublishPeriodMs = 20U;
constexpr std::uint32_t kPwmPublishPeriodMs = 50U;
constexpr std::uint32_t kImuPublishPeriodMs = 20U;
constexpr std::uint32_t kButtonPublishPeriodMs = 50U;
constexpr std::uint32_t kBatteryPublishPeriodMs = 1000U;
constexpr std::uint32_t kHeartbeatPublishPeriodMs = 500U;
constexpr std::uint32_t kDiagnosticsPublishPeriodMs = 1000U;

enum class BestEffortPublisherIndex : std::uint8_t {
  kMotors = 0U,
  kPwmServos,
  kImu,
};

enum class ReliablePublisherIndex : std::uint8_t {
  kButtons = 0U,
  kBattery,
  kHeartbeat,
  kDiagnostics,
};

static_assert(static_cast<std::size_t>(BestEffortPublisherIndex::kImu) + 1U ==
              kBestEffortPublisherCount);
static_assert(static_cast<std::size_t>(ReliablePublisherIndex::kDiagnostics) +
                  1U ==
              kReliablePublisherCount);

constexpr std::uint16_t kHeartbeatTimeSynchronized = 1U;
constexpr std::uint16_t kHeartbeatMotorWatchdogActive = 2U;
constexpr std::uint16_t kHeartbeatLowBattery = 4U;
constexpr std::uint16_t kHeartbeatImuHealthy = 8U;
constexpr std::uint16_t kHeartbeatBusServoBusy = 16U;

constexpr std::size_t ToIndex(PublisherIndex index) {
  return static_cast<std::size_t>(index);
}

bool PeriodElapsed(std::uint32_t now_ms, std::uint32_t last_ms,
                   std::uint32_t period_ms) {
  return now_ms - last_ms >= period_ms;
}

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

}  // namespace

void MicroRosRuntime::InitializeSchedules(std::uint32_t now_ms) {
  last_agent_ping_ms_ = now_ms;
  last_time_sync_attempt_ms_ = now_ms;
  last_motor_publish_ms_ = now_ms - kMotorPublishPeriodMs;
  last_pwm_publish_ms_ = now_ms - kPwmPublishPeriodMs;
  last_imu_publish_ms_ = now_ms - kImuPublishPeriodMs;
  last_button_publish_ms_ = now_ms - kButtonPublishPeriodMs;
  last_battery_publish_ms_ = now_ms - kBatteryPublishPeriodMs;
  last_heartbeat_publish_ms_ = now_ms - kHeartbeatPublishPeriodMs;
  last_diagnostics_publish_ms_ = now_ms - kDiagnosticsPublishPeriodMs;
  service_completion_cursor_.Reset();
  service_request_cursor_.Reset();
  busy_bus_request_cursor_.Reset();
  best_effort_publisher_cursor_.Reset();
  reliable_publisher_cursor_.Reset();
  active_work_scheduler_.Reset();
  alternate_bus_get_first_ = true;
  active_slice_budget_.Reset(ActiveWorkClass::kService);
}

void MicroRosRuntime::PublishDueTelemetry(std::uint32_t now_ms,
                                          bool allow_reliable) {
  PublishOneDueBestEffortTelemetry(now_ms);
  if (lifecycle_.state() != SessionState::kActive) {
    return;
  }
  if (allow_reliable) {
    PublishOneDueReliableTelemetry(now_ms);
  }
}

void MicroRosRuntime::PublishOneDueBestEffortTelemetry(std::uint32_t now_ms) {
  for (std::size_t offset = 0U; offset < kBestEffortPublisherCount; ++offset) {
    const std::size_t publisher = best_effort_publisher_cursor_.Peek(offset);
    bool due = false;
    switch (static_cast<BestEffortPublisherIndex>(publisher)) {
      case BestEffortPublisherIndex::kMotors:
        due = PeriodElapsed(now_ms, last_motor_publish_ms_,
                            kMotorPublishPeriodMs);
        break;
      case BestEffortPublisherIndex::kPwmServos:
        due = PeriodElapsed(now_ms, last_pwm_publish_ms_, kPwmPublishPeriodMs);
        break;
      case BestEffortPublisherIndex::kImu:
        due = PeriodElapsed(now_ms, last_imu_publish_ms_, kImuPublishPeriodMs);
        break;
    }
    if (!due) {
      continue;
    }

    switch (static_cast<BestEffortPublisherIndex>(publisher)) {
      case BestEffortPublisherIndex::kMotors:
        last_motor_publish_ms_ = now_ms;
        PublishMotorState(now_ms);
        break;
      case BestEffortPublisherIndex::kPwmServos:
        last_pwm_publish_ms_ = now_ms;
        PublishPwmServoState(now_ms);
        break;
      case BestEffortPublisherIndex::kImu:
        last_imu_publish_ms_ = now_ms;
        PublishImuState(now_ms);
        break;
    }
    best_effort_publisher_cursor_.AdvancePast(publisher);
    return;
  }
}

void MicroRosRuntime::PublishOneDueReliableTelemetry(std::uint32_t now_ms) {
  for (std::size_t offset = 0U; offset < kReliablePublisherCount; ++offset) {
    const std::size_t publisher = reliable_publisher_cursor_.Peek(offset);
    bool due = false;
    switch (static_cast<ReliablePublisherIndex>(publisher)) {
      case ReliablePublisherIndex::kButtons:
        due = PeriodElapsed(now_ms, last_button_publish_ms_,
                            kButtonPublishPeriodMs);
        break;
      case ReliablePublisherIndex::kBattery:
        due = PeriodElapsed(now_ms, last_battery_publish_ms_,
                            kBatteryPublishPeriodMs);
        break;
      case ReliablePublisherIndex::kHeartbeat:
        due = PeriodElapsed(now_ms, last_heartbeat_publish_ms_,
                            kHeartbeatPublishPeriodMs);
        break;
      case ReliablePublisherIndex::kDiagnostics:
        due = PeriodElapsed(now_ms, last_diagnostics_publish_ms_,
                            kDiagnosticsPublishPeriodMs);
        break;
    }
    if (!due) {
      continue;
    }

    bool attempted = false;
    switch (static_cast<ReliablePublisherIndex>(publisher)) {
      case ReliablePublisherIndex::kButtons:
        attempted = PublishButtonEvent(now_ms);
        break;
      case ReliablePublisherIndex::kBattery:
        last_battery_publish_ms_ = now_ms;
        attempted = PublishBatteryState(now_ms);
        break;
      case ReliablePublisherIndex::kHeartbeat:
        last_heartbeat_publish_ms_ = now_ms;
        attempted = PublishHeartbeat(now_ms);
        break;
      case ReliablePublisherIndex::kDiagnostics:
        last_diagnostics_publish_ms_ = now_ms;
        attempted = PublishDiagnostics(now_ms);
        break;
    }
    if (lifecycle_.state() != SessionState::kActive) {
      return;
    }
    if (attempted) {
      reliable_publisher_cursor_.AdvancePast(publisher);
      return;
    }
  }
}

void MicroRosRuntime::PublishMotorState(std::uint32_t now_ms) {
  MotorTelemetry snapshot = motor_telemetry_cache_;
  if (hooks_.read_motor_telemetry(hooks_.context, &snapshot)) {
    motor_telemetry_cache_ = snapshot;
    has_motor_telemetry_ = true;
  } else if (!has_motor_telemetry_) {
    snapshot.timestamp_ms = now_ms;
  }
  auto& message = publication_messages_.motors;
  SetStamp(snapshot.timestamp_ms, ToIndex(PublisherIndex::kMotors),
           &message.stamp);
  std::copy(snapshot.target_rps.begin(), snapshot.target_rps.end(),
            message.target_rps);
  std::copy(snapshot.measured_rps.begin(), snapshot.measured_rps.end(),
            message.measured_rps);
  std::copy(snapshot.encoder_count.begin(), snapshot.encoder_count.end(),
            message.encoder_count);
  message.motor_model = static_cast<std::uint8_t>(snapshot.motor_model);
  message.watchdog_stop_mask = snapshot.watchdog_stop_mask;
  static_cast<void>(Publish(ToIndex(PublisherIndex::kMotors), &message, false));
}

void MicroRosRuntime::PublishPwmServoState(std::uint32_t now_ms) {
  PwmServoTelemetry snapshot = pwm_telemetry_cache_;
  if (hooks_.read_pwm_servo_telemetry(hooks_.context, &snapshot)) {
    pwm_telemetry_cache_ = snapshot;
    has_pwm_telemetry_ = true;
  } else if (!has_pwm_telemetry_) {
    snapshot.timestamp_ms = now_ms;
  }
  auto& message = publication_messages_.pwm_servos;
  SetStamp(snapshot.timestamp_ms, ToIndex(PublisherIndex::kPwmServos),
           &message.stamp);
  std::copy(snapshot.target_pulse_width_us.begin(),
            snapshot.target_pulse_width_us.end(),
            message.target_pulse_width_us);
  std::copy(snapshot.output_pulse_width_us.begin(),
            snapshot.output_pulse_width_us.end(),
            message.output_pulse_width_us);
  std::copy(snapshot.offset_us.begin(), snapshot.offset_us.end(),
            message.offset_us);
  message.moving_mask = snapshot.moving_mask;
  static_cast<void>(
      Publish(ToIndex(PublisherIndex::kPwmServos), &message, false));
}

void MicroRosRuntime::PublishImuState(std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  ImuTelemetry snapshot = imu_telemetry_cache_;
  ImuTelemetry update{};
  if (hooks_.read_imu_telemetry(hooks_.context, &update)) {
    if (update.valid) {
      imu_telemetry_cache_ = update;
      has_imu_sample_ = true;
    } else {
      // A failed read changes validity only. Retain the last valid vectors and
      // sample time as required by the wire contract.
      imu_telemetry_cache_.valid = false;
    }
    snapshot = imu_telemetry_cache_;
  }
  auto& message = publication_messages_.imu;
  if (has_imu_sample_) {
    SetStamp(snapshot.timestamp_ms, ToIndex(PublisherIndex::kImu),
             &message.stamp);
  } else {
    message.stamp.sec = 0;
    message.stamp.nanosec = 0U;
  }
  std::copy(snapshot.angular_velocity_rad_s.begin(),
            snapshot.angular_velocity_rad_s.end(),
            message.angular_velocity_rad_s);
  std::copy(snapshot.linear_acceleration_m_s2.begin(),
            snapshot.linear_acceleration_m_s2.end(),
            message.linear_acceleration_m_s2);
  message.valid = snapshot.valid;
  static_cast<void>(Publish(ToIndex(PublisherIndex::kImu), &message, false));
}

bool MicroRosRuntime::PublishButtonEvent(std::uint32_t now_ms) {
  mentor_pi::mcu::ButtonEvent event{};
  if (!hooks_.pop_button_event(hooks_.context, &event)) {
    return false;
  }
  last_button_publish_ms_ = now_ms;
  auto& message = publication_messages_.button;
  SetStamp(event.timestamp_ms, ToIndex(PublisherIndex::kButtons),
           &message.stamp);
  message.button_id = event.button_id;
  message.event = static_cast<std::uint8_t>(event.event);
  static_cast<void>(Publish(ToIndex(PublisherIndex::kButtons), &message, true));
  return true;
}

bool MicroRosRuntime::PublishBatteryState(std::uint32_t now_ms) {
  BatteryTelemetry snapshot = battery_telemetry_cache_;
  if (hooks_.read_battery_telemetry(hooks_.context, &snapshot)) {
    battery_telemetry_cache_ = snapshot;
    has_battery_telemetry_ = true;
  } else if (!has_battery_telemetry_) {
    snapshot.timestamp_ms = now_ms;
    snapshot.valid = false;
  }
  auto& message = publication_messages_.battery;
  SetStamp(snapshot.timestamp_ms, ToIndex(PublisherIndex::kBattery),
           &message.stamp);
  message.voltage_mv = snapshot.voltage_mv;
  message.low_threshold_mv = snapshot.low_threshold_mv;
  message.valid = snapshot.valid;
  message.below_threshold = snapshot.below_threshold;
  static_cast<void>(Publish(ToIndex(PublisherIndex::kBattery), &message, true));
  return true;
}

bool MicroRosRuntime::PublishHeartbeat(std::uint32_t now_ms) {
  HealthSnapshot health{};
  hooks_.read_health(hooks_.context, &health);
  auto& message = publication_messages_.heartbeat;
  SetStamp(now_ms, ToIndex(PublisherIndex::kHeartbeat), &message.stamp);
  message.sequence = heartbeat_sequence_;
  ++heartbeat_sequence_;
  message.uptime_ms = now_ms;
  message.agent_session_id = lifecycle_.session_generation();

  std::uint16_t flags = 0U;
  if (time_synchronized_) {
    flags |= kHeartbeatTimeSynchronized;
  }
  if (health.motor_watchdog_active) {
    flags |= kHeartbeatMotorWatchdogActive;
  }
  if (health.low_battery) {
    flags |= kHeartbeatLowBattery;
  }
  if (health.imu_healthy) {
    flags |= kHeartbeatImuHealthy;
  }
  if (health.bus_servo_busy) {
    flags |= kHeartbeatBusServoBusy;
  }
  message.flags = flags;
  if (health.output_processing_fault) {
    message.state = static_cast<std::uint8_t>(HeartbeatState::kFault);
  } else if (!time_synchronized_ || health.nonfatal_degraded) {
    message.state = static_cast<std::uint8_t>(HeartbeatState::kDegraded);
  } else {
    message.state = static_cast<std::uint8_t>(HeartbeatState::kReady);
  }
  static_cast<void>(
      Publish(ToIndex(PublisherIndex::kHeartbeat), &message, true));
  return true;
}

bool MicroRosRuntime::PublishDiagnostics(std::uint32_t now_ms) {
  WorkerDiagnostics worker{};
  hooks_.read_worker_diagnostics(hooks_.context, &worker);
  const auto transport = ReadTransportSnapshot();
  auto& message = publication_messages_.diagnostics;
  SetStamp(now_ms, ToIndex(PublisherIndex::kDiagnostics), &message.stamp);
  message.transport_rx_bytes = transport.rx_wire_bytes;
  message.transport_tx_bytes = transport.tx_wire_bytes;
  message.uptime_ms = now_ms;
  message.session_generation = lifecycle_.session_generation();
  message.agent_reconnects = lifecycle_.agent_reconnects();
  message.command_messages = counters_.command_messages;
  message.command_rejections = counters_.command_rejections;
  std::copy(counters_.mailbox_overwrites.begin(),
            counters_.mailbox_overwrites.end(), message.mailbox_overwrites);
  message.button_event_drops = worker.button_event_drops;
  message.publication_errors = counters_.publication_errors;
  message.service_requests = counters_.service_requests;
  message.service_completions = counters_.service_completions;
  message.service_busy_rejections = counters_.service_busy_rejections;
  message.service_timeouts = counters_.service_timeouts;
  message.service_partial_results = counters_.service_partial_results;
  message.late_response_drops = counters_.late_response_drops;
  std::copy(worker.motor_lease_expiries.begin(),
            worker.motor_lease_expiries.end(), message.motor_lease_expiries);
  std::copy(worker.motor_command_rejections.begin(),
            worker.motor_command_rejections.end(),
            message.motor_command_rejections);
  message.motor_watchdog_trips = worker.motor_watchdog_trips;
  message.motor_command_consumptions = worker.motor_command_consumptions;
  message.motor_command_age_over_20_ms = worker.motor_command_age_over_20_ms;
  message.motor_command_max_age_us = worker.motor_command_max_age_us;
  message.executor_overruns = counters_.executor_overruns;
  std::copy(worker.peripheral_errors.begin(), worker.peripheral_errors.end(),
            message.peripheral_errors);
  std::copy(worker.peripheral_timeouts.begin(),
            worker.peripheral_timeouts.end(), message.peripheral_timeouts);
  std::copy(counters_.usart1_errors.begin(), counters_.usart1_errors.end(),
            message.usart1_errors);
  message.usart1_rx_dma_high_water_bytes = transport.rx_high_water_bytes;
  message.transport_rx_overruns = counters_.transport_rx_overruns;
  message.transport_tx_timeouts = counters_.transport_tx_timeouts;
  message.maximum_transport_wait_us = transport.maximum_wait_us;
  std::copy(worker.task_missed_releases.begin(),
            worker.task_missed_releases.end(), message.task_missed_releases);
  std::copy(worker.task_max_execution_us.begin(),
            worker.task_max_execution_us.end(), message.task_max_execution_us);
  std::copy(worker.task_stack_high_water_bytes.begin(),
            worker.task_stack_high_water_bytes.end(),
            message.task_stack_high_water_bytes);
  std::copy(worker.task_heartbeat_age_ms.begin(),
            worker.task_heartbeat_age_ms.end(), message.task_heartbeat_age_ms);
  std::copy(worker.free_ram_bytes.begin(), worker.free_ram_bytes.end(),
            message.free_ram_bytes);
  std::copy(worker.minimum_free_ram_bytes.begin(),
            worker.minimum_free_ram_bytes.end(),
            message.minimum_free_ram_bytes);
  message.flash_used_bytes = worker.flash_used_bytes;
  message.flash_total_bytes = worker.flash_total_bytes;
  message.post_seal_allocation_attempts = arena_.post_seal_attempts();
  message.session_state = static_cast<std::uint8_t>(lifecycle_.state());
  message.last_teardown_reason =
      static_cast<std::uint8_t>(lifecycle_.teardown_reason());
  message.last_reset_reason = worker.last_reset_reason;
  message.last_watchdog_task = worker.last_watchdog_task;

  const bool runtime_error_is_newer =
      worker.last_error_uptime_ms == 0U ||
      (counters_.last_error_uptime_ms != 0U &&
       static_cast<std::int32_t>(counters_.last_error_uptime_ms -
                                 worker.last_error_uptime_ms) >= 0);
  if (runtime_error_is_newer) {
    message.last_error_uptime_ms = counters_.last_error_uptime_ms;
    message.last_error_detail = counters_.last_error_detail;
    message.last_error_code = counters_.last_error_code;
    message.last_error_source =
        static_cast<std::uint8_t>(counters_.last_error_source);
  } else {
    message.last_error_uptime_ms = worker.last_error_uptime_ms;
    message.last_error_detail = worker.last_error_detail;
    message.last_error_code = worker.last_error_code;
    message.last_error_source = worker.last_error_source;
  }

  static_cast<void>(
      Publish(ToIndex(PublisherIndex::kDiagnostics), &message, true));
  return true;
}

bool MicroRosRuntime::Publish(std::size_t publisher_index, const void* message,
                              bool reliable) {
  if (lifecycle_.state() != SessionState::kActive ||
      publisher_index >= publishers_.size()) {
    return false;
  }
  if (reliable && !active_slice_budget_.TryStartBlockingOperation(
                      ActiveWorkClass::kReliableTelemetry)) {
    SaturatingIncrement(&counters_.publication_errors);
    RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                    {mentor_pi::mcu::ResultCode::kIoError,
                     static_cast<std::uint16_t>(publisher_index)});
    return false;
  }
  const MiddlewareBoundary boundary =
      reliable ? MiddlewareBoundary::kReliablePublish
               : MiddlewareBoundary::kBestEffortPublish;
  const std::uint32_t deadline_ms =
      reliable ? kReliableOperationTimeoutMs : kNonblockingCallDeadlineMs;
  const auto result = static_cast<rcl_ret_t>(InvokeMiddleware(
      boundary, deadline_ms, [this, publisher_index, message]() {
        return static_cast<std::int32_t>(
            rcl_publish(&publishers_[publisher_index], message, nullptr));
      }));
  if (arena_.invariant_violated()) {
    SaturatingIncrement(&counters_.publication_errors);
    RequestTeardown(TeardownReason::kMemoryViolation, ErrorSource::kMemory,
                    {mentor_pi::mcu::ResultCode::kIoError, 0U});
    return false;
  }
  if (result == RCL_RET_OK) {
    return true;
  }
  SaturatingIncrement(&counters_.publication_errors);
  RequestTeardown(TeardownReason::kEntityError, ErrorSource::kExecutor,
                  RclError(result));
  return false;
}

}  // namespace mentor_pi_mcu::app::microros
