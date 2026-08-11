// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_CORE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_CORE_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi_bringup {

enum class QualificationStream : std::uint8_t {
  kHeartbeat = 0,
  kDiagnostics,
  kMotorState,
  kPwmServoState,
  kImu,
  kBattery,
  kButtonEvents,
  kCount,
};

constexpr std::size_t kQualificationStreamCount =
    static_cast<std::size_t>(QualificationStream::kCount);
constexpr std::size_t kMonotonicDiagnosticCounterCount = 70U;
constexpr std::size_t kPeripheralDiagnosticCount = 8U;

struct QualificationTimestamp {
  std::int32_t seconds = 0;
  std::uint32_t nanoseconds = 0;
};

struct QualificationConfiguration {
  std::int64_t start_time_ns = 0;
  std::int64_t duration_ns = INT64_C(60000000000);
  std::int64_t discovery_timeout_ns = INT64_C(5000000000);
  double rate_tolerance_fraction = 0.05;
  double maximum_transport_bytes_per_second = 70000.0;
  bool imu_characterization_mode = false;
  // Development-only first-board mode. This may excuse only the exact,
  // continuously retried SSD1306 initialization NACK caused by no installed
  // OLED. Strict preflight and release qualification never enable it.
  bool allow_missing_oled = false;
  bool motor_command_age_evidence_required = false;
};

struct HeartbeatObservation {
  QualificationTimestamp stamp{};
  std::int64_t arrival_time_ns = 0;
  std::uint32_t sequence = 0;
  std::uint32_t uptime_ms = 0;
  std::uint32_t agent_session_id = 0;
  std::uint8_t state = 0;
  bool time_synchronized = false;
  bool imu_healthy = false;
};

struct DiagnosticsObservation {
  QualificationTimestamp stamp{};
  std::int64_t arrival_time_ns = 0;
  std::uint32_t uptime_ms = 0;
  std::uint32_t session_generation = 0;
  std::array<std::uint64_t, kMonotonicDiagnosticCounterCount>
      monotonic_counters{};
  std::uint64_t transport_rx_bytes = 0;
  std::uint64_t transport_tx_bytes = 0;
  std::uint64_t transport_error_total = 0;
  std::uint64_t diagnostic_fault_total = 0;
  std::uint32_t command_messages = 0;
  std::uint32_t motor_mailbox_overwrites = 0;
  std::uint32_t motor_command_consumptions = 0;
  std::uint32_t motor_command_age_over_20_ms = 0;
  std::uint32_t motor_command_max_age_us = 0;
  std::array<std::uint32_t, kPeripheralDiagnosticCount> peripheral_errors{};
  std::array<std::uint32_t, kPeripheralDiagnosticCount> peripheral_timeouts{};
  std::uint32_t post_seal_allocation_attempts = 0;
  std::uint16_t last_error_detail = 0;
  std::uint8_t session_state = 0;
  std::uint8_t last_reset_reason = 0;
  std::uint8_t last_error_code = 0;
  std::uint8_t last_error_source = 0;
};

struct ImuObservation {
  QualificationTimestamp stamp{};
  std::int64_t arrival_time_ns = 0;
  bool valid = false;
  bool vectors_finite = false;
};

enum class QualificationFailure : std::uint8_t {
  kInvalidConfiguration = 0,
  kDurationIncomplete,
  kPublisherMissing,
  kPublisherLost,
  kSessionInvalid,
  kSessionChanged,
  kSequenceRegression,
  kUptimeRegression,
  kTimeSynchronizationLost,
  kTimestampInvalid,
  kTimestampRegression,
  kRateOutOfRange,
  kDiagnosticCounterRegression,
  kTransportErrorChanged,
  kDiagnosticFaultChanged,
  kResetReasonChanged,
  kPostSealAllocation,
  kSessionNotActive,
  kHeartbeatNotReady,
  kTransportRateExceeded,
  kInvalidTelemetry,
  kImuCharacterizationMismatch,
  kMotorCommandAgeBaselineContaminated,
  kMotorCommandAgeEvidenceMissing,
  kMotorCommandAgeP99Exceeded,
  kMotorCommandAgeMaximumExceeded,
  kMotorCommandAcceptanceOutOfRange,
  kMotorCommandAgeCounterRegression,
  kMotorCommandPublisherInvalid,
};

constexpr std::uint64_t QualificationFailureBit(QualificationFailure failure) {
  return UINT64_C(1) << static_cast<std::uint8_t>(failure);
}

struct QualificationStreamSummary {
  std::uint64_t sample_count = 0;
  double observed_rate_hz = 0.0;
};

struct QualificationSummary {
  bool passed = false;
  std::uint64_t failure_mask = 0;
  std::uint8_t missing_publisher_mask = 0;
  std::uint8_t lost_publisher_mask = 0;
  std::uint8_t rate_failure_mask = 0;
  std::uint8_t timestamp_failure_mask = 0;
  std::uint8_t invalid_telemetry_mask = 0;
  std::array<QualificationStreamSummary, kQualificationStreamCount> streams{};
  std::uint32_t agent_session_id = 0;
  double transport_bytes_per_second = 0.0;
  double maximum_transport_interval_bytes_per_second = 0.0;
  std::uint64_t zero_motor_commands_published = 0;
  std::uint64_t zero_motor_commands_in_evidence_window = 0;
  std::uint64_t motor_command_messages = 0;
  std::uint64_t motor_command_consumptions = 0;
  std::uint64_t motor_mailbox_overwrites = 0;
  std::uint64_t motor_commands_accounted = 0;
  std::uint64_t motor_command_age_over_20_ms = 0;
  std::uint32_t motor_command_max_age_us = 0;
};

// Strict-preflight validation for BatteryState. The public controller contract
// declares samples above 20,000 mV invalid even though the wire field can
// represent a larger uint16 value.
bool IsValidQualificationBatteryState(std::uint16_t voltage_mv,
                                      std::uint16_t low_threshold_mv,
                                      bool valid);

// USB-only first-board characterization deliberately has no battery input.
// Accept only the exact fail-closed wire state; strict qualification continues
// to require IsValidQualificationBatteryState().
bool IsAbsentQualificationBatteryState(std::uint16_t voltage_mv,
                                       std::uint16_t low_threshold_mv,
                                       bool valid, bool below_threshold);

class QualificationMonitorCore final {
 public:
  explicit QualificationMonitorCore(QualificationConfiguration configuration);

  void ObservePublisher(QualificationStream stream, bool present,
                        std::int64_t observation_time_ns);
  void ObserveHeartbeat(const HeartbeatObservation& observation);
  void ObserveDiagnostics(const DiagnosticsObservation& observation);
  void ObserveImu(const ImuObservation& observation);
  void ObserveTelemetry(QualificationStream stream,
                        const QualificationTimestamp& stamp,
                        std::int64_t arrival_time_ns, bool valid);
  void ObserveMotorCommandPublisherCount(std::size_t publisher_count,
                                         std::int64_t observation_time_ns);
  void RecordZeroMotorCommand();
  bool MotorCommandAgeBaselineReady() const;
  void Tick(std::int64_t now_ns);
  QualificationSummary Finish(std::int64_t finish_time_ns);

 private:
  struct StreamState {
    bool publisher_present = false;
    bool publisher_seen = false;
    bool stamp_seen = false;
    std::uint64_t sample_count = 0;
    std::int64_t last_arrival_ns = 0;
    QualificationTimestamp last_stamp{};
  };

  static bool IsForwardProgress(std::uint32_t current, std::uint32_t previous);
  static std::size_t StreamIndex(QualificationStream stream);
  static std::uint8_t StreamBit(QualificationStream stream);
  void AddFailure(QualificationFailure failure);
  void CountSample(QualificationStream stream,
                   const QualificationTimestamp& stamp,
                   std::int64_t arrival_time_ns, bool valid);
  void CheckTimestamp(QualificationStream stream,
                      const QualificationTimestamp& stamp);
  bool DiagnosticsMatchFirstBoardMode(
      const DiagnosticsObservation& observation) const;
  std::uint64_t EffectiveDiagnosticFaultTotal(
      const DiagnosticsObservation& observation) const;

  QualificationConfiguration configuration_{};
  std::array<StreamState, kQualificationStreamCount> streams_{};
  std::uint64_t failure_mask_ = 0;
  std::uint8_t missing_publisher_mask_ = 0;
  std::uint8_t lost_publisher_mask_ = 0;
  std::uint8_t rate_failure_mask_ = 0;
  std::uint8_t timestamp_failure_mask_ = 0;
  std::uint8_t invalid_telemetry_mask_ = 0;

  bool heartbeat_seen_ = false;
  bool heartbeat_progress_seen_ = false;
  bool heartbeat_ready_seen_ = false;
  bool time_synchronization_seen_ = false;
  bool last_heartbeat_time_synchronized_ = false;
  bool last_heartbeat_imu_healthy_ = false;
  std::uint32_t agent_session_id_ = 0;
  std::uint32_t last_heartbeat_sequence_ = 0;
  std::uint32_t last_heartbeat_uptime_ms_ = 0;
  std::uint8_t last_heartbeat_state_ = 0;

  bool diagnostics_seen_ = false;
  bool diagnostics_progress_seen_ = false;
  bool last_diagnostics_match_first_board_mode_ = false;
  std::uint32_t diagnostics_session_generation_ = 0;
  std::uint32_t last_diagnostics_uptime_ms_ = 0;
  std::array<std::uint64_t, kMonotonicDiagnosticCounterCount>
      last_diagnostic_counters_{};
  std::uint64_t baseline_transport_error_total_ = 0;
  std::uint64_t baseline_diagnostic_fault_total_ = 0;
  std::uint8_t baseline_reset_reason_ = 0;
  std::uint64_t first_transport_rx_bytes_ = 0;
  std::uint64_t first_transport_tx_bytes_ = 0;
  std::uint64_t last_transport_rx_bytes_ = 0;
  std::uint64_t last_transport_tx_bytes_ = 0;
  std::int64_t first_transport_time_ns_ = 0;
  std::int64_t last_transport_time_ns_ = 0;
  double maximum_transport_interval_bytes_per_second_ = 0.0;
  std::uint64_t zero_motor_commands_published_ = 0;
  bool motor_command_age_baseline_clean_ = false;
  bool motor_command_publisher_count_seen_ = false;
  std::size_t last_motor_command_publisher_count_ = 0U;
  std::uint64_t baseline_zero_motor_commands_published_ = 0U;
  std::uint64_t last_diagnostics_zero_motor_commands_published_ = 0U;
  std::uint32_t baseline_command_messages_ = 0U;
  std::uint32_t baseline_motor_mailbox_overwrites_ = 0U;
  std::uint32_t baseline_motor_command_consumptions_ = 0U;
  std::uint32_t baseline_motor_command_age_over_20_ms_ = 0U;
  std::uint32_t last_command_messages_ = 0U;
  std::uint32_t last_motor_mailbox_overwrites_ = 0U;
  std::uint32_t last_motor_command_consumptions_ = 0U;
  std::uint32_t last_motor_command_age_over_20_ms_ = 0U;
  std::uint32_t last_motor_command_max_age_us_ = 0U;
};

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_CORE_H_
