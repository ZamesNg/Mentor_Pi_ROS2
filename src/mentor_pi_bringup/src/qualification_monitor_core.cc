// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_monitor_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mentor_pi_bringup {
namespace {

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr std::uint32_t kNanosecondsLimit = UINT32_C(1000000000);
constexpr std::uint16_t kMinimumBatteryThresholdMv = 5000U;
constexpr std::uint16_t kMaximumBatteryVoltageAndThresholdMv = 20000U;
constexpr std::uint8_t kHeartbeatBooting = 0U;
constexpr std::uint8_t kHeartbeatReady = 1U;
constexpr std::uint8_t kHeartbeatDegraded = 2U;
constexpr std::uint8_t kSessionActive = 3U;
constexpr std::size_t kImuPeripheralIndex = 1U;
constexpr std::uint8_t kUnsupportedResult = 6U;
constexpr std::uint8_t kImuErrorSource = 9U;
constexpr std::uint16_t kUnverifiedImuDetail = 1U;
constexpr std::uint32_t kMaximumMotorCommandAgeUs = 100000U;
constexpr std::array<double, kQualificationStreamCount> kExpectedRatesHz{
    2.0, 1.0, 50.0, 20.0, 50.0, 1.0, 0.0};
constexpr double kMaximumButtonEventRateHz = 20.0;

bool IsTimestampZero(const QualificationTimestamp& stamp) {
  return stamp.seconds == 0 && stamp.nanoseconds == 0U;
}

bool IsTimestampBefore(const QualificationTimestamp& left,
                       const QualificationTimestamp& right) {
  return left.seconds < right.seconds || (left.seconds == right.seconds &&
                                          left.nanoseconds < right.nanoseconds);
}

void SaturatingIncrement(std::uint64_t* value) {
  if (*value != std::numeric_limits<std::uint64_t>::max()) {
    ++(*value);
  }
}

}  // namespace

bool IsValidQualificationBatteryState(std::uint16_t voltage_mv,
                                      std::uint16_t low_threshold_mv,
                                      bool valid) {
  return valid && voltage_mv > 0U &&
         voltage_mv <= kMaximumBatteryVoltageAndThresholdMv &&
         low_threshold_mv >= kMinimumBatteryThresholdMv &&
         low_threshold_mv <= kMaximumBatteryVoltageAndThresholdMv;
}

QualificationMonitorCore::QualificationMonitorCore(
    QualificationConfiguration configuration)
    : configuration_(configuration) {
  if (configuration_.duration_ns <= 0 ||
      configuration_.discovery_timeout_ns < 0 ||
      configuration_.discovery_timeout_ns > configuration_.duration_ns ||
      !std::isfinite(configuration_.rate_tolerance_fraction) ||
      configuration_.rate_tolerance_fraction < 0.0 ||
      configuration_.rate_tolerance_fraction > 1.0 ||
      !std::isfinite(configuration_.maximum_transport_bytes_per_second) ||
      configuration_.maximum_transport_bytes_per_second <= 0.0) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
  }
}

void QualificationMonitorCore::ObservePublisher(
    QualificationStream stream, bool present,
    std::int64_t observation_time_ns) {
  const std::size_t index = StreamIndex(stream);
  if (index >= streams_.size()) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }
  if (observation_time_ns < configuration_.start_time_ns) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
  }

  StreamState& state = streams_[index];
  if (!present && state.publisher_seen && state.publisher_present) {
    lost_publisher_mask_ |= StreamBit(stream);
    AddFailure(QualificationFailure::kPublisherLost);
  }
  if (present) {
    state.publisher_seen = true;
  }
  state.publisher_present = present;
}

void QualificationMonitorCore::ObserveHeartbeat(
    const HeartbeatObservation& observation) {
  if (time_synchronization_seen_ && !observation.time_synchronized) {
    AddFailure(QualificationFailure::kTimeSynchronizationLost);
  }
  time_synchronization_seen_ =
      time_synchronization_seen_ || observation.time_synchronized;
  last_heartbeat_time_synchronized_ = observation.time_synchronized;
  last_heartbeat_imu_healthy_ = observation.imu_healthy;
  CountSample(QualificationStream::kHeartbeat, observation.stamp,
              observation.arrival_time_ns, true);

  if (observation.agent_session_id == 0U) {
    AddFailure(QualificationFailure::kSessionInvalid);
  }
  if (!heartbeat_seen_) {
    heartbeat_seen_ = true;
    agent_session_id_ = observation.agent_session_id;
    last_heartbeat_sequence_ = observation.sequence;
    last_heartbeat_uptime_ms_ = observation.uptime_ms;
  } else {
    if (observation.agent_session_id != agent_session_id_) {
      AddFailure(QualificationFailure::kSessionChanged);
    }
    if (!IsForwardProgress(observation.sequence, last_heartbeat_sequence_)) {
      AddFailure(QualificationFailure::kSequenceRegression);
    } else {
      heartbeat_progress_seen_ = true;
    }
    if (!IsForwardProgress(observation.uptime_ms, last_heartbeat_uptime_ms_)) {
      AddFailure(QualificationFailure::kUptimeRegression);
    }
    last_heartbeat_sequence_ = observation.sequence;
    last_heartbeat_uptime_ms_ = observation.uptime_ms;
  }
  if (diagnostics_seen_ &&
      observation.agent_session_id != diagnostics_session_generation_) {
    AddFailure(QualificationFailure::kSessionChanged);
  }
  if (configuration_.imu_characterization_mode) {
    if (observation.imu_healthy) {
      AddFailure(QualificationFailure::kImuCharacterizationMismatch);
    }
    if (observation.state == kHeartbeatDegraded) {
      heartbeat_degraded_seen_ = true;
    } else if (observation.state != kHeartbeatBooting ||
               heartbeat_degraded_seen_) {
      AddFailure(QualificationFailure::kHeartbeatNotReady);
      AddFailure(QualificationFailure::kImuCharacterizationMismatch);
    }
  } else if (observation.state == kHeartbeatReady) {
    heartbeat_ready_seen_ = true;
  } else if (heartbeat_ready_seen_ ||
             (observation.state != kHeartbeatBooting &&
              observation.state != kHeartbeatDegraded)) {
    AddFailure(QualificationFailure::kHeartbeatNotReady);
  }
  last_heartbeat_state_ = observation.state;
}

void QualificationMonitorCore::ObserveDiagnostics(
    const DiagnosticsObservation& observation) {
  CountSample(QualificationStream::kDiagnostics, observation.stamp,
              observation.arrival_time_ns, true);

  if (observation.session_generation == 0U) {
    AddFailure(QualificationFailure::kSessionInvalid);
  }
  if (heartbeat_seen_ && observation.session_generation != agent_session_id_) {
    AddFailure(QualificationFailure::kSessionChanged);
  }
  if (observation.session_state != kSessionActive) {
    AddFailure(QualificationFailure::kSessionNotActive);
  }
  if (observation.post_seal_allocation_attempts != 0U) {
    AddFailure(QualificationFailure::kPostSealAllocation);
  }
  if (configuration_.imu_characterization_mode &&
      !DiagnosticsMatchImuCharacterization(observation)) {
    AddFailure(QualificationFailure::kImuCharacterizationMismatch);
  }

  if (!diagnostics_seen_) {
    diagnostics_seen_ = true;
    diagnostics_session_generation_ = observation.session_generation;
    last_diagnostics_uptime_ms_ = observation.uptime_ms;
    last_diagnostic_counters_ = observation.monotonic_counters;
    baseline_transport_error_total_ = observation.transport_error_total;
    baseline_diagnostic_fault_total_ = observation.diagnostic_fault_total;
    baseline_reset_reason_ = observation.last_reset_reason;
    first_transport_rx_bytes_ = observation.transport_rx_bytes;
    first_transport_tx_bytes_ = observation.transport_tx_bytes;
    first_transport_time_ns_ = observation.arrival_time_ns;
    baseline_zero_motor_commands_published_ = zero_motor_commands_published_;
    baseline_command_messages_ = observation.command_messages;
    baseline_motor_mailbox_overwrites_ = observation.motor_mailbox_overwrites;
    baseline_motor_command_consumptions_ =
        observation.motor_command_consumptions;
    baseline_motor_command_age_over_20_ms_ =
        observation.motor_command_age_over_20_ms;
    motor_command_age_baseline_clean_ =
        observation.motor_command_consumptions == 0U &&
        observation.motor_command_age_over_20_ms == 0U &&
        observation.motor_command_max_age_us == 0U;
    if (configuration_.motor_command_age_evidence_required &&
        !motor_command_age_baseline_clean_) {
      AddFailure(QualificationFailure::kMotorCommandAgeBaselineContaminated);
    }
  } else {
    if (observation.session_generation != diagnostics_session_generation_) {
      AddFailure(QualificationFailure::kSessionChanged);
    }
    if (!IsForwardProgress(observation.uptime_ms,
                           last_diagnostics_uptime_ms_)) {
      AddFailure(QualificationFailure::kUptimeRegression);
    } else {
      diagnostics_progress_seen_ = true;
    }
    for (std::size_t index = 0; index < last_diagnostic_counters_.size();
         ++index) {
      if (observation.monotonic_counters[index] <
          last_diagnostic_counters_[index]) {
        AddFailure(QualificationFailure::kDiagnosticCounterRegression);
      }
    }
    if (observation.command_messages < last_command_messages_ ||
        observation.motor_mailbox_overwrites < last_motor_mailbox_overwrites_ ||
        observation.motor_command_consumptions <
            last_motor_command_consumptions_ ||
        observation.motor_command_age_over_20_ms <
            last_motor_command_age_over_20_ms_ ||
        observation.motor_command_max_age_us < last_motor_command_max_age_us_) {
      AddFailure(QualificationFailure::kMotorCommandAgeCounterRegression);
    }
    if (observation.transport_error_total != baseline_transport_error_total_) {
      AddFailure(QualificationFailure::kTransportErrorChanged);
    }
    if (observation.diagnostic_fault_total !=
        baseline_diagnostic_fault_total_) {
      AddFailure(QualificationFailure::kDiagnosticFaultChanged);
    }
    if (observation.last_reset_reason != baseline_reset_reason_) {
      AddFailure(QualificationFailure::kResetReasonChanged);
    }

    if (observation.arrival_time_ns > last_transport_time_ns_ &&
        observation.transport_rx_bytes >= last_transport_rx_bytes_ &&
        observation.transport_tx_bytes >= last_transport_tx_bytes_) {
      const double elapsed_seconds =
          static_cast<double>(observation.arrival_time_ns -
                              last_transport_time_ns_) /
          kNanosecondsPerSecond;
      const double interval_bytes =
          static_cast<double>(observation.transport_rx_bytes -
                              last_transport_rx_bytes_) +
          static_cast<double>(observation.transport_tx_bytes -
                              last_transport_tx_bytes_);
      const double interval_rate = interval_bytes / elapsed_seconds;
      maximum_transport_interval_bytes_per_second_ =
          std::max(maximum_transport_interval_bytes_per_second_, interval_rate);
      if (interval_rate >= configuration_.maximum_transport_bytes_per_second) {
        AddFailure(QualificationFailure::kTransportRateExceeded);
      }
    }
    last_diagnostics_uptime_ms_ = observation.uptime_ms;
    last_diagnostic_counters_ = observation.monotonic_counters;
  }
  last_transport_rx_bytes_ = observation.transport_rx_bytes;
  last_transport_tx_bytes_ = observation.transport_tx_bytes;
  last_transport_time_ns_ = observation.arrival_time_ns;
  last_diagnostics_zero_motor_commands_published_ =
      zero_motor_commands_published_;
  last_command_messages_ = observation.command_messages;
  last_motor_mailbox_overwrites_ = observation.motor_mailbox_overwrites;
  last_motor_command_consumptions_ = observation.motor_command_consumptions;
  last_motor_command_age_over_20_ms_ = observation.motor_command_age_over_20_ms;
  last_motor_command_max_age_us_ = observation.motor_command_max_age_us;
}

void QualificationMonitorCore::ObserveImu(const ImuObservation& observation) {
  if (configuration_.imu_characterization_mode) {
    CountImuCharacterizationSample(observation);
    return;
  }
  CountSample(QualificationStream::kImu, observation.stamp,
              observation.arrival_time_ns,
              observation.valid && observation.vectors_finite);
}

void QualificationMonitorCore::ObserveTelemetry(
    QualificationStream stream, const QualificationTimestamp& stamp,
    std::int64_t arrival_time_ns, bool valid) {
  if (stream == QualificationStream::kHeartbeat ||
      stream == QualificationStream::kDiagnostics ||
      stream == QualificationStream::kImu ||
      stream == QualificationStream::kCount) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }
  CountSample(stream, stamp, arrival_time_ns, valid);
}

void QualificationMonitorCore::RecordZeroMotorCommand() {
  SaturatingIncrement(&zero_motor_commands_published_);
}

void QualificationMonitorCore::ObserveMotorCommandPublisherCount(
    std::size_t publisher_count, std::int64_t observation_time_ns) {
  if (!configuration_.motor_command_age_evidence_required) {
    return;
  }
  if (observation_time_ns < configuration_.start_time_ns) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }
  motor_command_publisher_count_seen_ = true;
  last_motor_command_publisher_count_ = publisher_count;
  const bool discovery_complete =
      observation_time_ns - configuration_.start_time_ns >=
      configuration_.discovery_timeout_ns;
  if (publisher_count > 1U || (discovery_complete && publisher_count != 1U)) {
    AddFailure(QualificationFailure::kMotorCommandPublisherInvalid);
  }
}

bool QualificationMonitorCore::MotorCommandAgeBaselineReady() const {
  return diagnostics_seen_ && motor_command_age_baseline_clean_;
}

void QualificationMonitorCore::Tick(std::int64_t now_ns) {
  if (now_ns < configuration_.start_time_ns) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }
  if (now_ns - configuration_.start_time_ns <
      configuration_.discovery_timeout_ns) {
    return;
  }
  for (std::size_t index = 0; index < streams_.size(); ++index) {
    if (!streams_[index].publisher_present) {
      missing_publisher_mask_ |= static_cast<std::uint8_t>(UINT8_C(1) << index);
      AddFailure(QualificationFailure::kPublisherMissing);
    }
  }
}

QualificationSummary QualificationMonitorCore::Finish(
    std::int64_t finish_time_ns) {
  Tick(finish_time_ns);
  if (finish_time_ns < configuration_.start_time_ns ||
      finish_time_ns - configuration_.start_time_ns <
          configuration_.duration_ns) {
    AddFailure(QualificationFailure::kDurationIncomplete);
  }

  QualificationSummary summary;
  double monitor_elapsed_seconds = 0.0;
  if (finish_time_ns > configuration_.start_time_ns) {
    monitor_elapsed_seconds =
        static_cast<double>(finish_time_ns - configuration_.start_time_ns) /
        kNanosecondsPerSecond;
  }
  for (std::size_t index = 0; index < streams_.size(); ++index) {
    const QualificationStream stream = static_cast<QualificationStream>(index);
    const StreamState& state = streams_[index];
    if (!state.publisher_present) {
      missing_publisher_mask_ |= StreamBit(stream);
      AddFailure(QualificationFailure::kPublisherMissing);
    }

    summary.streams[index].sample_count = state.sample_count;
    const double expected_rate = kExpectedRatesHz[index];
    const bool characterization_imu =
        configuration_.imu_characterization_mode &&
        stream == QualificationStream::kImu;
    if (expected_rate > 0.0 && state.sample_count >= 2U &&
        monitor_elapsed_seconds > 0.0) {
      summary.streams[index].observed_rate_hz =
          static_cast<double>(state.sample_count - 1U) /
          monitor_elapsed_seconds;
    } else if (expected_rate == 0.0 && monitor_elapsed_seconds > 0.0) {
      summary.streams[index].observed_rate_hz =
          static_cast<double>(state.sample_count) / monitor_elapsed_seconds;
    }

    if (characterization_imu) {
      if (state.sample_count > 1U) {
        rate_failure_mask_ |= StreamBit(stream);
        AddFailure(QualificationFailure::kImuCharacterizationMismatch);
      }
    } else if (expected_rate > 0.0) {
      const double minimum_rate =
          expected_rate * (1.0 - configuration_.rate_tolerance_fraction);
      const double maximum_rate =
          expected_rate * (1.0 + configuration_.rate_tolerance_fraction);
      const double observed_rate = summary.streams[index].observed_rate_hz;
      if (state.sample_count < 2U || observed_rate < minimum_rate ||
          observed_rate > maximum_rate) {
        rate_failure_mask_ |= StreamBit(stream);
        AddFailure(QualificationFailure::kRateOutOfRange);
      }
    } else if (state.sample_count >= 2U &&
               summary.streams[index].observed_rate_hz >
                   kMaximumButtonEventRateHz *
                       (1.0 + configuration_.rate_tolerance_fraction)) {
      rate_failure_mask_ |= StreamBit(stream);
      AddFailure(QualificationFailure::kRateOutOfRange);
    }
  }

  if (configuration_.imu_characterization_mode) {
    if (!heartbeat_seen_ || !heartbeat_progress_seen_ ||
        !heartbeat_degraded_seen_ ||
        last_heartbeat_state_ != kHeartbeatDegraded ||
        last_heartbeat_imu_healthy_) {
      AddFailure(QualificationFailure::kHeartbeatNotReady);
      AddFailure(QualificationFailure::kImuCharacterizationMismatch);
    }
    if (!time_synchronization_seen_ || !last_heartbeat_time_synchronized_) {
      AddFailure(QualificationFailure::kTimeSynchronizationLost);
      AddFailure(QualificationFailure::kImuCharacterizationMismatch);
    }
    if (!diagnostics_seen_) {
      AddFailure(QualificationFailure::kImuCharacterizationMismatch);
    }
  } else if (!heartbeat_seen_ || !heartbeat_progress_seen_ ||
             !heartbeat_ready_seen_ ||
             last_heartbeat_state_ != kHeartbeatReady) {
    AddFailure(QualificationFailure::kHeartbeatNotReady);
  }
  if (!diagnostics_seen_ || !diagnostics_progress_seen_) {
    AddFailure(QualificationFailure::kSessionNotActive);
  }

  if (configuration_.motor_command_age_evidence_required) {
    if (!motor_command_publisher_count_seen_ ||
        last_motor_command_publisher_count_ != 1U) {
      AddFailure(QualificationFailure::kMotorCommandPublisherInvalid);
    }
    if (!motor_command_age_baseline_clean_) {
      AddFailure(QualificationFailure::kMotorCommandAgeBaselineContaminated);
    }

    const bool evidence_counters_valid =
        last_command_messages_ >= baseline_command_messages_ &&
        last_motor_mailbox_overwrites_ >= baseline_motor_mailbox_overwrites_ &&
        last_motor_command_consumptions_ >=
            baseline_motor_command_consumptions_ &&
        last_motor_command_age_over_20_ms_ >=
            baseline_motor_command_age_over_20_ms_;
    if (!evidence_counters_valid) {
      AddFailure(QualificationFailure::kMotorCommandAgeCounterRegression);
    } else {
      summary.motor_command_messages = static_cast<std::uint64_t>(
          last_command_messages_ - baseline_command_messages_);
      summary.motor_mailbox_overwrites = static_cast<std::uint64_t>(
          last_motor_mailbox_overwrites_ - baseline_motor_mailbox_overwrites_);
      summary.motor_command_consumptions =
          static_cast<std::uint64_t>(last_motor_command_consumptions_ -
                                     baseline_motor_command_consumptions_);
      summary.motor_command_age_over_20_ms =
          static_cast<std::uint64_t>(last_motor_command_age_over_20_ms_ -
                                     baseline_motor_command_age_over_20_ms_);
      summary.motor_commands_accounted =
          summary.motor_command_consumptions + summary.motor_mailbox_overwrites;
    }
    summary.motor_command_max_age_us = last_motor_command_max_age_us_;
    if (last_diagnostics_zero_motor_commands_published_ >=
        baseline_zero_motor_commands_published_) {
      summary.zero_motor_commands_in_evidence_window =
          last_diagnostics_zero_motor_commands_published_ -
          baseline_zero_motor_commands_published_;
    } else {
      AddFailure(QualificationFailure::kMotorCommandAgeCounterRegression);
    }

    if (summary.zero_motor_commands_in_evidence_window == 0U ||
        summary.motor_command_consumptions == 0U) {
      AddFailure(QualificationFailure::kMotorCommandAgeEvidenceMissing);
    }
    if (summary.motor_command_age_over_20_ms >
        summary.motor_command_consumptions / UINT64_C(100)) {
      AddFailure(QualificationFailure::kMotorCommandAgeP99Exceeded);
    }
    if (summary.motor_command_max_age_us >= kMaximumMotorCommandAgeUs) {
      AddFailure(QualificationFailure::kMotorCommandAgeMaximumExceeded);
    }

    const std::uint64_t required_accounted =
        summary.zero_motor_commands_in_evidence_window -
        (summary.zero_motor_commands_in_evidence_window / UINT64_C(20));
    const bool lower_bound_met =
        summary.motor_commands_accounted + UINT64_C(1) >= required_accounted;
    const bool upper_bound_met =
        summary.motor_commands_accounted <=
        summary.zero_motor_commands_in_evidence_window + UINT64_C(1);
    const bool command_counter_coherent =
        summary.motor_commands_accounted <=
        summary.motor_command_messages + UINT64_C(1);
    if (!lower_bound_met || !upper_bound_met || !command_counter_coherent) {
      AddFailure(QualificationFailure::kMotorCommandAcceptanceOutOfRange);
    }
  }

  if (diagnostics_seen_ && last_transport_time_ns_ > first_transport_time_ns_ &&
      last_transport_rx_bytes_ >= first_transport_rx_bytes_ &&
      last_transport_tx_bytes_ >= first_transport_tx_bytes_) {
    const double elapsed_seconds =
        static_cast<double>(last_transport_time_ns_ -
                            first_transport_time_ns_) /
        kNanosecondsPerSecond;
    const double received_bytes = static_cast<double>(
        last_transport_rx_bytes_ - first_transport_rx_bytes_);
    const double transmitted_bytes = static_cast<double>(
        last_transport_tx_bytes_ - first_transport_tx_bytes_);
    summary.transport_bytes_per_second =
        (received_bytes + transmitted_bytes) / elapsed_seconds;
    if (summary.transport_bytes_per_second >=
        configuration_.maximum_transport_bytes_per_second) {
      AddFailure(QualificationFailure::kTransportRateExceeded);
    }
  }
  summary.maximum_transport_interval_bytes_per_second =
      maximum_transport_interval_bytes_per_second_;

  summary.failure_mask = failure_mask_;
  summary.missing_publisher_mask = missing_publisher_mask_;
  summary.lost_publisher_mask = lost_publisher_mask_;
  summary.rate_failure_mask = rate_failure_mask_;
  summary.timestamp_failure_mask = timestamp_failure_mask_;
  summary.invalid_telemetry_mask = invalid_telemetry_mask_;
  summary.agent_session_id = agent_session_id_;
  summary.zero_motor_commands_published = zero_motor_commands_published_;
  summary.passed = failure_mask_ == 0U;
  return summary;
}

bool QualificationMonitorCore::IsForwardProgress(std::uint32_t current,
                                                 std::uint32_t previous) {
  const std::uint32_t difference = current - previous;
  return difference != 0U && difference < UINT32_C(0x80000000);
}

std::size_t QualificationMonitorCore::StreamIndex(QualificationStream stream) {
  return static_cast<std::size_t>(stream);
}

std::uint8_t QualificationMonitorCore::StreamBit(QualificationStream stream) {
  const std::size_t index = StreamIndex(stream);
  if (index >= kQualificationStreamCount) {
    return 0U;
  }
  return static_cast<std::uint8_t>(UINT8_C(1) << index);
}

void QualificationMonitorCore::AddFailure(QualificationFailure failure) {
  failure_mask_ |= QualificationFailureBit(failure);
}

void QualificationMonitorCore::CountSample(QualificationStream stream,
                                           const QualificationTimestamp& stamp,
                                           std::int64_t arrival_time_ns,
                                           bool valid) {
  const std::size_t index = StreamIndex(stream);
  if (index >= streams_.size()) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }

  StreamState& state = streams_[index];
  if (arrival_time_ns < configuration_.start_time_ns ||
      (state.sample_count != 0U && arrival_time_ns < state.last_arrival_ns)) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
  }
  state.last_arrival_ns = arrival_time_ns;
  SaturatingIncrement(&state.sample_count);
  CheckTimestamp(stream, stamp);
  if (!valid) {
    invalid_telemetry_mask_ |= StreamBit(stream);
    AddFailure(QualificationFailure::kInvalidTelemetry);
  }
}

void QualificationMonitorCore::CountImuCharacterizationSample(
    const ImuObservation& observation) {
  StreamState& state = streams_[StreamIndex(QualificationStream::kImu)];
  if (observation.arrival_time_ns < configuration_.start_time_ns ||
      (state.sample_count != 0U &&
       observation.arrival_time_ns < state.last_arrival_ns)) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
  }
  state.last_arrival_ns = observation.arrival_time_ns;
  SaturatingIncrement(&state.sample_count);

  if (state.sample_count > 1U) {
    AddFailure(QualificationFailure::kImuCharacterizationMismatch);
  }

  if (!IsTimestampZero(observation.stamp)) {
    timestamp_failure_mask_ |= StreamBit(QualificationStream::kImu);
    AddFailure(QualificationFailure::kTimestampInvalid);
    AddFailure(QualificationFailure::kImuCharacterizationMismatch);
  }
  if (observation.valid || !observation.vectors_finite) {
    invalid_telemetry_mask_ |= StreamBit(QualificationStream::kImu);
    AddFailure(QualificationFailure::kImuCharacterizationMismatch);
  }
}

void QualificationMonitorCore::CheckTimestamp(
    QualificationStream stream, const QualificationTimestamp& stamp) {
  const std::size_t index = StreamIndex(stream);
  if (index >= streams_.size()) {
    AddFailure(QualificationFailure::kInvalidConfiguration);
    return;
  }
  StreamState& state = streams_[index];
  if (stamp.seconds < 0 || stamp.nanoseconds >= kNanosecondsLimit) {
    timestamp_failure_mask_ |= StreamBit(stream);
    AddFailure(QualificationFailure::kTimestampInvalid);
    return;
  }
  if (IsTimestampZero(stamp)) {
    if (time_synchronization_seen_ || state.stamp_seen) {
      timestamp_failure_mask_ |= StreamBit(stream);
      AddFailure(QualificationFailure::kTimestampInvalid);
    }
    return;
  }
  if (state.stamp_seen && IsTimestampBefore(stamp, state.last_stamp)) {
    timestamp_failure_mask_ |= StreamBit(stream);
    AddFailure(QualificationFailure::kTimestampRegression);
  }
  state.last_stamp = stamp;
  state.stamp_seen = true;
}

bool QualificationMonitorCore::DiagnosticsMatchImuCharacterization(
    const DiagnosticsObservation& observation) {
  if (observation.diagnostic_fault_total != 1U ||
      observation.transport_error_total != 0U ||
      observation.last_error_code != kUnsupportedResult ||
      observation.last_error_source != kImuErrorSource ||
      observation.last_error_detail != kUnverifiedImuDetail ||
      observation.peripheral_errors[kImuPeripheralIndex] != 1U) {
    return false;
  }
  for (std::size_t index = 0U; index < observation.peripheral_errors.size();
       ++index) {
    if ((index != kImuPeripheralIndex &&
         observation.peripheral_errors[index] != 0U) ||
        observation.peripheral_timeouts[index] != 0U) {
      return false;
    }
  }
  return true;
}

}  // namespace mentor_pi_bringup
