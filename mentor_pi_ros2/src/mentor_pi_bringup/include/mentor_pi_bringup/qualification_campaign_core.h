// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_CORE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_CORE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace mentor_pi_bringup {

constexpr std::size_t kCampaignCommandCount = 7U;
constexpr std::size_t kCampaignServiceCount = 6U;
constexpr std::size_t kCampaignTelemetryCount = 7U;
constexpr std::size_t kCampaignDiagnosticCounterCount = 70U;
constexpr std::size_t kMaximumCampaignTransitions = 100U;

enum class CampaignMode : std::uint8_t {
  kLoad500 = 0,
  kSoak,
  kReconnectUsb,
  kReconnectAgent,
  kResetMcu,
};

enum class CampaignCommand : std::uint8_t {
  kMotor = 0,
  kPwmServo,
  kBusServo,
  kLed,
  kBuzzer,
  kRgb,
  kOled,
  kCount,
};

enum class CampaignService : std::uint8_t {
  kMotorModel = 0,
  kPwmOffsets,
  kBusState,
  kBusConfigure,
  kBusStop,
  kBatteryThreshold,
  kCount,
};

enum class CampaignTelemetry : std::uint8_t {
  kHeartbeat = 0,
  kDiagnostics,
  kMotorState,
  kPwmServoState,
  kImu,
  kBattery,
  kButtonEvents,
  kCount,
};

enum class CampaignMetricStatus : std::uint8_t {
  kPass = 0,
  kFail,
  kNotObserved,
  kNotApplicable,
};

enum class CampaignFailure : std::uint8_t {
  kInvalidConfiguration = 0,
  kEvidenceNotStarted,
  kDurationIncomplete,
  kScheduleMissed,
  kCommandCountMismatch,
  kNonzeroMotorAttempt,
  kInvalidTelemetry,
  kTelemetryRate,
  kSessionChanged,
  kSessionInvalid,
  kSessionCycleMissing,
  kEndpointLost,
  kEndpointRecoveryTimeout,
  kUnexpectedReset,
  kResetCycleMissing,
  kDiagnosticRegression,
  kDiagnosticError,
  kTransportRate,
  kMotorAgeBaseline,
  kMotorAgeEvidenceMissing,
  kMotorAgeP99,
  kMotorAgeMaximum,
  kServiceFailure,
  kServiceCoverage,
};

constexpr std::uint64_t CampaignFailureBit(CampaignFailure failure) {
  return UINT64_C(1) << static_cast<std::uint8_t>(failure);
}

struct CampaignProfile {
  CampaignMode mode = CampaignMode::kLoad500;
  std::int64_t canonical_duration_ns = INT64_C(3600000000000);
  struct Rate {
    std::uint32_t numerator = 0U;
    std::uint32_t denominator_seconds = 1U;
  };
  std::array<Rate, kCampaignCommandCount> command_rates{};
  std::int64_t service_round_period_ns = INT64_C(60000000000);
  std::uint32_t expected_cycles = 0U;
  bool continuous_session_required = true;
};

const char* CampaignModeName(CampaignMode mode);
std::optional<CampaignMode> ParseCampaignMode(const std::string& value);
CampaignProfile CampaignProfileForMode(CampaignMode mode);
std::uint64_t ScheduledSlotCount(std::int64_t duration_ns,
                                 CampaignProfile::Rate rate);

struct CampaignDueWork {
  std::uint8_t command_mask = 0U;
  bool service_round = false;
  std::uint8_t missed_command_mask = 0U;
  std::uint64_t missed_service_rounds = 0U;
  bool duration_complete = false;
};

class CampaignScheduler final {
 public:
  CampaignScheduler(CampaignProfile profile, std::int64_t duration_ns);

  bool Start(std::int64_t start_time_ns);
  CampaignDueWork Poll(std::int64_t now_ns);
  const std::array<std::uint64_t, kCampaignCommandCount>& skipped_commands()
      const {
    return skipped_commands_;
  }
  std::uint64_t skipped_service_rounds() const {
    return skipped_service_rounds_;
  }
  std::int64_t start_time_ns() const { return start_time_ns_; }
  std::int64_t duration_ns() const { return duration_ns_; }

 private:
  CampaignProfile profile_{};
  std::int64_t start_time_ns_ = 0;
  std::int64_t duration_ns_ = 0;
  std::array<std::uint64_t, kCampaignCommandCount> accounted_command_slots_{};
  std::array<std::uint64_t, kCampaignCommandCount> skipped_commands_{};
  std::uint64_t accounted_service_slots_ = 0U;
  std::uint64_t skipped_service_rounds_ = 0U;
  bool started_ = false;
};

struct CampaignHeartbeatObservation {
  std::int64_t arrival_time_ns = 0;
  std::uint32_t sequence = 0U;
  std::uint32_t uptime_ms = 0U;
  std::uint32_t session_id = 0U;
  std::uint8_t state = 0U;
};

struct CampaignDiagnosticsObservation {
  std::int64_t arrival_time_ns = 0;
  std::uint64_t transport_rx_bytes = 0U;
  std::uint64_t transport_tx_bytes = 0U;
  std::uint64_t hard_error_total = 0U;
  std::array<std::uint64_t, kCampaignDiagnosticCounterCount>
      monotonic_counters{};
  std::uint32_t uptime_ms = 0U;
  std::uint32_t session_generation = 0U;
  std::uint32_t motor_command_consumptions = 0U;
  std::uint32_t motor_command_age_over_20_ms = 0U;
  std::uint32_t motor_command_max_age_us = 0U;
  std::uint32_t post_seal_allocation_attempts = 0U;
  std::uint8_t session_state = 0U;
  std::uint8_t reset_reason = 0U;
};

struct CampaignTransition {
  std::uint32_t cycle = 0U;
  std::uint32_t previous_session_id = 0U;
  std::uint32_t new_session_id = 0U;
  std::uint64_t heartbeat_gap_ms = 0U;
  std::uint64_t connected_interval_ms = 0U;
  std::uint64_t endpoint_recovery_ms = 0U;
  bool endpoint_recovered = false;
  bool heartbeat_outage_observed = false;
  bool mcu_uptime_regression_observed = false;
  std::uint8_t reset_reason = 255U;
};

struct CampaignConfiguration {
  CampaignProfile profile{};
  std::int64_t duration_ns = INT64_C(3600000000000);
  std::int64_t heartbeat_loss_ns = INT64_C(1000000000);
  std::int64_t endpoint_recovery_limit_ns = INT64_C(5000000000);
  std::int64_t minimum_telemetry_gap_ns = 0;
  double maximum_transport_bytes_per_second = 70000.0;
  bool require_button_stimulus = false;
  bool require_valid_imu = true;
};

struct CampaignSummary {
  CampaignMode mode = CampaignMode::kLoad500;
  CampaignProfile profile{};
  std::int64_t configured_duration_ns = 0;
  bool execution_passed = false;
  bool release_qualified = false;
  bool canonical_profile = false;
  std::uint64_t failure_mask = 0U;
  std::int64_t evidence_start_time_ns = 0;
  std::int64_t evidence_finish_time_ns = 0;
  std::array<std::uint64_t, kCampaignCommandCount> command_publications{};
  std::array<std::uint64_t, kCampaignCommandCount> skipped_commands{};
  std::array<std::uint64_t, kCampaignServiceCount> service_requests{};
  std::array<std::uint64_t, kCampaignServiceCount> service_successes{};
  std::array<std::uint64_t, kCampaignServiceCount> service_failures{};
  std::array<std::uint64_t, kCampaignServiceCount> service_skips{};
  std::array<std::uint64_t, kCampaignTelemetryCount> telemetry_samples{};
  std::array<double, kCampaignTelemetryCount> telemetry_rates_hz{};
  std::uint64_t motor_command_consumptions = 0U;
  std::uint64_t motor_command_age_over_20_ms = 0U;
  std::uint32_t motor_command_max_age_us = 0U;
  double maximum_transport_interval_bytes_per_second = 0.0;
  std::uint32_t initial_session_id = 0U;
  std::uint32_t final_session_id = 0U;
  std::uint32_t observed_cycles = 0U;
  std::uint32_t observed_mcu_resets = 0U;
  std::size_t stored_transition_count = 0U;
  std::array<CampaignTransition, kMaximumCampaignTransitions> transitions{};
  CampaignMetricStatus internal_wire_traffic =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus complete_one_second_wire_windows =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus independent_wire_capture =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus motor_age_p99 = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus motor_age_maximum = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus physical_motor_response =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus physical_endpoint_recovery =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus usb_outage_rotation =
      CampaignMetricStatus::kNotApplicable;
  CampaignMetricStatus stack_headroom = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus allocation_trace = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus button_stimulus = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus imu_function = CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus connected_telemetry_gaps =
      CampaignMetricStatus::kNotObserved;
  CampaignMetricStatus bus_stale_replay = CampaignMetricStatus::kNotObserved;
};

class QualificationCampaignCore final {
 public:
  explicit QualificationCampaignCore(CampaignConfiguration configuration);

  void ObserveHeartbeat(const CampaignHeartbeatObservation& observation);
  void ObserveDiagnostics(const CampaignDiagnosticsObservation& observation);
  void ObserveEndpointGraph(bool complete, std::int64_t observation_time_ns);
  void ObserveTelemetry(CampaignTelemetry stream, std::int64_t arrival_time_ns,
                        bool valid);
  bool BeginEvidence(std::int64_t start_time_ns);
  CampaignDueWork Poll(std::int64_t now_ns);
  bool RecordMotorCommand(std::uint8_t update_mask,
                          const std::array<float, 4U>& targets);
  void RecordCommand(CampaignCommand command);
  void RecordServiceResult(CampaignService service, std::uint8_t result_code,
                           bool request_sent, bool timed_out);
  void RecordServiceSkipped(CampaignService service);
  bool evidence_started() const { return evidence_started_; }
  bool failed() const { return failure_mask_ != 0U; }
  void LatchHarnessFailure(CampaignFailure failure) { AddFailure(failure); }
  bool CampaignComplete(std::int64_t now_ns) const;
  CampaignSummary Finish(std::int64_t finish_time_ns);

 private:
  static std::size_t CommandIndex(CampaignCommand command);
  static std::size_t ServiceIndex(CampaignService service);
  static std::size_t TelemetryIndex(CampaignTelemetry stream);
  static bool IsForwardProgress(std::uint32_t current, std::uint32_t previous);
  void AddFailure(CampaignFailure failure);
  void StartTransition(const CampaignHeartbeatObservation& observation,
                       bool uptime_regression);
  void AccumulateDiagnosticDelta(
      const CampaignDiagnosticsObservation& observation);
  void ResetDiagnosticBaseline(
      const CampaignDiagnosticsObservation& observation);
  void FinalizeRates(CampaignSummary* summary);
  void FinalizeCoverage(CampaignSummary* summary);

  CampaignConfiguration configuration_{};
  CampaignScheduler scheduler_;
  std::uint64_t failure_mask_ = 0U;
  bool evidence_started_ = false;
  std::int64_t evidence_start_time_ns_ = 0;

  bool heartbeat_seen_ = false;
  bool heartbeat_outage_active_ = false;
  std::int64_t heartbeat_outage_observed_time_ns_ = 0;
  std::int64_t last_heartbeat_time_ns_ = 0;
  std::uint32_t initial_session_id_ = 0U;
  std::uint32_t current_session_id_ = 0U;
  std::uint32_t last_heartbeat_sequence_ = 0U;
  std::uint32_t last_heartbeat_uptime_ms_ = 0U;

  bool diagnostics_seen_ = false;
  CampaignDiagnosticsObservation last_diagnostics_{};
  std::uint8_t initial_reset_reason_ = 0U;
  std::uint64_t accumulated_motor_consumptions_ = 0U;
  std::uint64_t accumulated_motor_age_over_20_ms_ = 0U;
  std::uint32_t motor_command_max_age_us_ = 0U;
  double maximum_transport_interval_bytes_per_second_ = 0.0;
  bool diagnostic_interval_observed_ = false;

  bool endpoint_graph_complete_ = false;
  bool endpoint_graph_observed_ = false;
  std::int64_t last_endpoint_graph_time_ns_ = 0;
  std::int64_t pending_transition_time_ns_ = 0;
  std::int64_t last_endpoint_recovery_time_ns_ = 0;
  std::size_t pending_transition_index_ = kMaximumCampaignTransitions;
  std::size_t last_transition_index_ = kMaximumCampaignTransitions;
  bool unbound_mcu_reset_observed_ = false;
  std::uint8_t unbound_mcu_reset_reason_ = 255U;
  std::uint32_t observed_cycles_ = 0U;
  std::uint32_t observed_mcu_resets_ = 0U;
  std::array<CampaignTransition, kMaximumCampaignTransitions> transitions_{};
  std::size_t stored_transition_count_ = 0U;

  std::array<std::uint64_t, kCampaignCommandCount> command_publications_{};
  std::array<std::uint64_t, kCampaignServiceCount> service_requests_{};
  std::array<std::uint64_t, kCampaignServiceCount> service_successes_{};
  std::array<std::uint64_t, kCampaignServiceCount> service_failures_{};
  std::array<std::uint64_t, kCampaignServiceCount> service_skips_{};
  std::array<std::uint64_t, kCampaignTelemetryCount> telemetry_samples_{};
  std::array<bool, kCampaignTelemetryCount> telemetry_invalid_{};
  std::array<bool, kCampaignTelemetryCount> telemetry_time_seen_{};
  std::array<std::int64_t, kCampaignTelemetryCount> first_telemetry_time_ns_{};
  std::array<std::int64_t, kCampaignTelemetryCount> last_telemetry_time_ns_{};
  std::array<bool, kCampaignTelemetryCount> telemetry_gap_baseline_seen_{};
  std::array<std::int64_t, kCampaignTelemetryCount>
      last_connected_telemetry_time_ns_{};
  std::array<std::uint8_t, kCampaignTelemetryCount>
      current_connected_telemetry_samples_{};
  std::int64_t connected_telemetry_epoch_start_ns_ = -1;
  bool connected_telemetry_gap_observed_ = false;
  bool connected_telemetry_gap_failed_ = false;
  std::array<bool, kCampaignTelemetryCount>
      connected_telemetry_gap_observed_by_stream_{};
};

struct CampaignEvidenceMetadata {
  std::string run_id;
  std::string source_revision;
  std::string firmware_sha256;
  std::string host_revision;
  std::string ros_distribution;
  std::string board_serial;
  std::string fixture_revision;
  std::string campaign_mode;
  std::string start_time_utc;
  std::string finish_time_utc;
};

const char* CampaignMetricStatusName(CampaignMetricStatus status);
bool IsValidCampaignEvidenceToken(const std::string& value,
                                  std::size_t maximum_length);
bool IsValidSha256(const std::string& value);
bool WriteCampaignEvidence(const std::string& directory,
                           const CampaignEvidenceMetadata& metadata,
                           const CampaignSummary& summary, std::string* error);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_CORE_H_
