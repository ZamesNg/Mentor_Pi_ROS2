// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_campaign_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace mentor_pi_bringup {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = INT64_C(1000000000);
constexpr std::uint32_t kHeartbeatReady = 1U;
constexpr std::uint32_t kHeartbeatDegraded = 2U;
constexpr std::uint8_t kSessionActive = 3U;
constexpr std::uint32_t kMaximumMotorAgeUs = 100000U;
constexpr std::uint8_t kResetPowerOn = 0U;
constexpr std::uint8_t kResetPin = 1U;
constexpr std::uint8_t kResetSoftware = 2U;
constexpr std::uint8_t kResetBrownout = 5U;
constexpr std::uint8_t kResetUnknown = 255U;
constexpr std::array<double, kCampaignTelemetryCount> kTelemetryRatesHz{
    2.0, 1.0, 50.0, 20.0, 50.0, 1.0, 0.0};
constexpr double kTelemetryRateTolerance = 0.05;

std::uint64_t SaturatingAdd(std::uint64_t left, std::uint64_t right) {
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return left + right;
}

std::uint64_t RateSlotsAtOrBefore(std::int64_t elapsed_ns,
                                  CampaignProfile::Rate rate) {
  if (elapsed_ns < 0 || rate.numerator == 0U ||
      rate.denominator_seconds == 0U) {
    return 0U;
  }
  const std::uint64_t elapsed = static_cast<std::uint64_t>(elapsed_ns);
  const std::uint64_t denominator_ns =
      static_cast<std::uint64_t>(rate.denominator_seconds) *
      static_cast<std::uint64_t>(kNanosecondsPerSecond);
  const std::uint64_t whole_blocks = elapsed / denominator_ns;
  const std::uint64_t remainder = elapsed % denominator_ns;
  const std::uint64_t completed_slots =
      (whole_blocks * static_cast<std::uint64_t>(rate.numerator)) +
      ((remainder * static_cast<std::uint64_t>(rate.numerator)) /
       denominator_ns);
  return SaturatingAdd(completed_slots, 1U);
}

std::uint64_t ServiceSlotsAtOrBefore(std::int64_t elapsed_ns,
                                     std::int64_t period_ns) {
  if (elapsed_ns < 0 || period_ns <= 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(elapsed_ns / period_ns) + 1U;
}

std::uint8_t CommandBit(std::size_t index) {
  return static_cast<std::uint8_t>(UINT8_C(1) << index);
}

bool ProfilesEqual(const CampaignProfile& left, const CampaignProfile& right) {
  if (left.mode != right.mode ||
      left.canonical_duration_ns != right.canonical_duration_ns ||
      left.service_round_period_ns != right.service_round_period_ns ||
      left.expected_cycles != right.expected_cycles ||
      left.continuous_session_required != right.continuous_session_required) {
    return false;
  }
  for (std::size_t index = 0U; index < left.command_rates.size(); ++index) {
    if (left.command_rates[index].numerator !=
            right.command_rates[index].numerator ||
        left.command_rates[index].denominator_seconds !=
            right.command_rates[index].denominator_seconds) {
      return false;
    }
  }
  return true;
}

bool RequiresPeriodicTelemetry(const CampaignConfiguration& configuration,
                               std::size_t index) {
  if (index == static_cast<std::size_t>(CampaignTelemetry::kButtonEvents)) {
    return configuration.profile.continuous_session_required &&
           configuration.require_button_stimulus;
  }
  if (index == static_cast<std::size_t>(CampaignTelemetry::kImu) &&
      !configuration.require_valid_imu) {
    return false;
  }
  return index < kTelemetryRatesHz.size() && kTelemetryRatesHz[index] > 0.0;
}

double RequiredTelemetryRateHz(const CampaignConfiguration& configuration,
                               std::size_t index) {
  if (!RequiresPeriodicTelemetry(configuration, index)) {
    return 0.0;
  }
  if (index == static_cast<std::size_t>(CampaignTelemetry::kButtonEvents)) {
    return 1.0;
  }
  return kTelemetryRatesHz[index];
}

std::int64_t MaximumTelemetryGapNs(const CampaignConfiguration& configuration,
                                   std::size_t index) {
  const double rate_hz = RequiredTelemetryRateHz(configuration, index);
  if (rate_hz <= 0.0) {
    return 0;
  }
  const std::int64_t required_gap_ns = static_cast<std::int64_t>(
      2.0 * static_cast<double>(kNanosecondsPerSecond) / rate_hz);
  return std::max(required_gap_ns, configuration.minimum_telemetry_gap_ns);
}

bool IsAcceptedOperatorResetReason(std::uint8_t reset_reason) {
  return reset_reason == kResetPowerOn || reset_reason == kResetPin ||
         reset_reason == kResetSoftware || reset_reason == kResetBrownout;
}

}  // namespace

const char* CampaignModeName(CampaignMode mode) {
  switch (mode) {
    case CampaignMode::kLoad500:
      return "load500";
    case CampaignMode::kSoak:
      return "soak";
    case CampaignMode::kReconnectUsb:
      return "reconnect_usb";
    case CampaignMode::kReconnectAgent:
      return "reconnect_agent";
    case CampaignMode::kResetMcu:
      return "reset_mcu";
  }
  return "unknown";
}

std::optional<CampaignMode> ParseCampaignMode(const std::string& value) {
  if (value == "load500") {
    return CampaignMode::kLoad500;
  }
  if (value == "soak") {
    return CampaignMode::kSoak;
  }
  if (value == "reconnect_usb") {
    return CampaignMode::kReconnectUsb;
  }
  if (value == "reconnect_agent") {
    return CampaignMode::kReconnectAgent;
  }
  if (value == "reset_mcu") {
    return CampaignMode::kResetMcu;
  }
  return std::nullopt;
}

CampaignProfile CampaignProfileForMode(CampaignMode mode) {
  CampaignProfile profile;
  profile.mode = mode;
  profile.service_round_period_ns = INT64_C(60000000000);
  switch (mode) {
    case CampaignMode::kLoad500:
      profile.canonical_duration_ns = INT64_C(3600000000000);
      profile.command_rates = {{{500U, 1U},
                                {10U, 1U},
                                {2U, 1U},
                                {1U, 1U},
                                {1U, 10U},
                                {10U, 1U},
                                {1U, 5U}}};
      profile.continuous_session_required = true;
      return profile;
    case CampaignMode::kSoak:
      profile.canonical_duration_ns = INT64_C(86400000000000);
      profile.command_rates = {{{50U, 1U},
                                {20U, 1U},
                                {10U, 1U},
                                {1U, 1U},
                                {1U, 60U},
                                {30U, 1U},
                                {1U, 1U}}};
      profile.continuous_session_required = true;
      return profile;
    case CampaignMode::kReconnectUsb:
    case CampaignMode::kReconnectAgent:
    case CampaignMode::kResetMcu:
      profile.canonical_duration_ns = 0;
      profile.command_rates = {{{500U, 1U},
                                {10U, 1U},
                                {2U, 1U},
                                {1U, 1U},
                                {1U, 10U},
                                {10U, 1U},
                                {1U, 5U}}};
      profile.expected_cycles = 100U;
      profile.continuous_session_required = false;
      return profile;
  }
  return profile;
}

std::uint64_t ScheduledSlotCount(std::int64_t duration_ns,
                                 CampaignProfile::Rate rate) {
  if (duration_ns <= 0 || rate.numerator == 0U ||
      rate.denominator_seconds == 0U) {
    return 0U;
  }
  return RateSlotsAtOrBefore(duration_ns - 1, rate);
}

CampaignScheduler::CampaignScheduler(CampaignProfile profile,
                                     std::int64_t duration_ns)
    : profile_(profile), duration_ns_(duration_ns) {}

bool CampaignScheduler::Start(std::int64_t start_time_ns) {
  if (started_ || start_time_ns < 0 || duration_ns_ < 0 ||
      profile_.service_round_period_ns <= 0) {
    return false;
  }
  start_time_ns_ = start_time_ns;
  started_ = true;
  return true;
}

CampaignDueWork CampaignScheduler::Poll(std::int64_t now_ns) {
  CampaignDueWork work;
  if (!started_ || now_ns < start_time_ns_) {
    return work;
  }
  const std::int64_t elapsed_ns = now_ns - start_time_ns_;
  if (duration_ns_ > 0 && elapsed_ns >= duration_ns_) {
    work.duration_complete = true;
    return work;
  }

  for (std::size_t index = 0U; index < profile_.command_rates.size(); ++index) {
    const std::uint64_t due =
        RateSlotsAtOrBefore(elapsed_ns, profile_.command_rates[index]);
    if (due <= accounted_command_slots_[index]) {
      continue;
    }
    const std::uint64_t newly_due = due - accounted_command_slots_[index];
    if (newly_due > 1U) {
      const std::uint64_t skipped = newly_due - 1U;
      skipped_commands_[index] =
          SaturatingAdd(skipped_commands_[index], skipped);
      work.missed_command_mask = static_cast<std::uint8_t>(
          work.missed_command_mask | CommandBit(index));
    }
    accounted_command_slots_[index] = due;
    work.command_mask =
        static_cast<std::uint8_t>(work.command_mask | CommandBit(index));
  }

  const std::uint64_t service_due =
      ServiceSlotsAtOrBefore(elapsed_ns, profile_.service_round_period_ns);
  if (service_due > accounted_service_slots_) {
    const std::uint64_t newly_due = service_due - accounted_service_slots_;
    if (newly_due > 1U) {
      work.missed_service_rounds = newly_due - 1U;
      skipped_service_rounds_ =
          SaturatingAdd(skipped_service_rounds_, work.missed_service_rounds);
    }
    accounted_service_slots_ = service_due;
    work.service_round = true;
  }
  return work;
}

QualificationCampaignCore::QualificationCampaignCore(
    CampaignConfiguration configuration)
    : configuration_(configuration),
      scheduler_(configuration.profile, configuration.duration_ns) {
  if (configuration_.duration_ns < 0 ||
      (configuration_.profile.continuous_session_required &&
       configuration_.duration_ns <= 0) ||
      configuration_.heartbeat_loss_ns <= 0 ||
      configuration_.endpoint_recovery_limit_ns <= 0 ||
      configuration_.minimum_telemetry_gap_ns < 0 ||
      !std::isfinite(configuration_.maximum_transport_bytes_per_second) ||
      configuration_.maximum_transport_bytes_per_second <= 0.0 ||
      configuration_.profile.expected_cycles > kMaximumCampaignTransitions) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
  }
}

void QualificationCampaignCore::ObserveHeartbeat(
    const CampaignHeartbeatObservation& observation) {
  if (observation.session_id == 0U || observation.arrival_time_ns < 0 ||
      (observation.state != kHeartbeatReady &&
       observation.state != kHeartbeatDegraded)) {
    AddFailure(CampaignFailure::kSessionInvalid);
  }
  if (!heartbeat_seen_) {
    heartbeat_seen_ = true;
    initial_session_id_ = observation.session_id;
    current_session_id_ = observation.session_id;
    last_heartbeat_sequence_ = observation.sequence;
    last_heartbeat_uptime_ms_ = observation.uptime_ms;
    last_heartbeat_time_ns_ = observation.arrival_time_ns;
    return;
  }

  if (observation.arrival_time_ns <= last_heartbeat_time_ns_) {
    AddFailure(CampaignFailure::kSessionInvalid);
    return;
  }

  const bool session_changed = observation.session_id != current_session_id_;
  const bool uptime_regression =
      !IsForwardProgress(observation.uptime_ms, last_heartbeat_uptime_ms_);
  const bool sequence_regression =
      !IsForwardProgress(observation.sequence, last_heartbeat_sequence_);
  if (session_changed || (uptime_regression && sequence_regression)) {
    if (evidence_started_) {
      StartTransition(observation, uptime_regression);
    } else {
      current_session_id_ = observation.session_id;
    }
  } else if (uptime_regression || sequence_regression) {
    AddFailure(CampaignFailure::kSessionInvalid);
  }

  last_heartbeat_sequence_ = observation.sequence;
  last_heartbeat_uptime_ms_ = observation.uptime_ms;
  last_heartbeat_time_ns_ = observation.arrival_time_ns;
  heartbeat_outage_active_ = false;
}

void QualificationCampaignCore::ObserveDiagnostics(
    const CampaignDiagnosticsObservation& observation) {
  const bool accepted_reset_reason =
      IsAcceptedOperatorResetReason(observation.reset_reason);
  if (!accepted_reset_reason) {
    AddFailure(CampaignFailure::kUnexpectedReset);
  }
  if (observation.arrival_time_ns < 0 || observation.session_generation == 0U ||
      observation.session_state != kSessionActive) {
    AddFailure(CampaignFailure::kSessionInvalid);
  }
  if (observation.post_seal_allocation_attempts != 0U) {
    AddFailure(CampaignFailure::kDiagnosticError);
  }
  if (heartbeat_seen_ && configuration_.profile.continuous_session_required &&
      observation.session_generation != current_session_id_) {
    AddFailure(CampaignFailure::kSessionChanged);
  }
  if (!diagnostics_seen_) {
    diagnostics_seen_ = true;
    initial_reset_reason_ = observation.reset_reason;
    ResetDiagnosticBaseline(observation);
    return;
  }

  if (observation.arrival_time_ns <= last_diagnostics_.arrival_time_ns) {
    AddFailure(CampaignFailure::kDiagnosticRegression);
    return;
  }

  const bool generation_changed =
      observation.session_generation != last_diagnostics_.session_generation;
  const bool uptime_regression =
      !IsForwardProgress(observation.uptime_ms, last_diagnostics_.uptime_ms);
  if (generation_changed || uptime_regression) {
    if (evidence_started_ && uptime_regression) {
      if (configuration_.profile.mode != CampaignMode::kResetMcu) {
        AddFailure(CampaignFailure::kUnexpectedReset);
      }
      if (accepted_reset_reason) {
        ++observed_mcu_resets_;
        if (last_transition_index_ < stored_transition_count_ &&
            transitions_[last_transition_index_].new_session_id ==
                observation.session_generation) {
          transitions_[last_transition_index_].mcu_uptime_regression_observed =
              true;
          transitions_[last_transition_index_].reset_reason =
              observation.reset_reason;
        } else if (!unbound_mcu_reset_observed_) {
          unbound_mcu_reset_observed_ = true;
          unbound_mcu_reset_reason_ = observation.reset_reason;
        } else {
          AddFailure(CampaignFailure::kResetCycleMissing);
        }
      }
    }
    if (evidence_started_ &&
        configuration_.profile.continuous_session_required) {
      AddFailure(CampaignFailure::kSessionChanged);
    }
    if (!uptime_regression) {
      AccumulateDiagnosticDelta(observation);
    } else if (observation.hard_error_total != 0U ||
               observation.post_seal_allocation_attempts != 0U) {
      AddFailure(CampaignFailure::kDiagnosticError);
    }
    ResetDiagnosticBaseline(observation);
    return;
  }

  AccumulateDiagnosticDelta(observation);
  last_diagnostics_ = observation;
}

void QualificationCampaignCore::ObserveEndpointGraph(
    bool complete, std::int64_t observation_time_ns) {
  if (observation_time_ns < 0 ||
      (endpoint_graph_observed_ &&
       observation_time_ns <= last_endpoint_graph_time_ns_)) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return;
  }
  const bool was_complete = endpoint_graph_complete_;
  endpoint_graph_observed_ = true;
  last_endpoint_graph_time_ns_ = observation_time_ns;
  endpoint_graph_complete_ = complete;
  if (!complete) {
    telemetry_gap_baseline_seen_.fill(false);
    current_connected_telemetry_samples_.fill(0U);
    connected_telemetry_epoch_start_ns_ = -1;
  } else if (evidence_started_ && !was_complete) {
    telemetry_gap_baseline_seen_.fill(false);
    current_connected_telemetry_samples_.fill(0U);
    connected_telemetry_epoch_start_ns_ = observation_time_ns;
  }
  if (!evidence_started_) {
    return;
  }
  if (!complete && configuration_.profile.continuous_session_required) {
    AddFailure(CampaignFailure::kEndpointLost);
  }
  if (complete && pending_transition_index_ < stored_transition_count_) {
    CampaignTransition& transition = transitions_[pending_transition_index_];
    if (!transition.endpoint_recovered) {
      if (observation_time_ns < pending_transition_time_ns_) {
        AddFailure(CampaignFailure::kInvalidConfiguration);
        return;
      }
      const std::int64_t recovery_ns =
          observation_time_ns - pending_transition_time_ns_;
      transition.endpoint_recovery_ms =
          static_cast<std::uint64_t>(recovery_ns / INT64_C(1000000));
      transition.endpoint_recovered = true;
      last_endpoint_recovery_time_ns_ = observation_time_ns;
      if (recovery_ns > configuration_.endpoint_recovery_limit_ns) {
        AddFailure(CampaignFailure::kEndpointRecoveryTimeout);
      }
      pending_transition_index_ = kMaximumCampaignTransitions;
    }
  }
}

void QualificationCampaignCore::ObserveTelemetry(CampaignTelemetry stream,
                                                 std::int64_t arrival_time_ns,
                                                 bool valid) {
  if (!evidence_started_) {
    return;
  }
  const std::size_t index = TelemetryIndex(stream);
  if (index >= telemetry_samples_.size() ||
      arrival_time_ns < evidence_start_time_ns_) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return;
  }
  telemetry_samples_[index] = SaturatingAdd(telemetry_samples_[index], 1U);
  if (telemetry_time_seen_[index] &&
      arrival_time_ns <= last_telemetry_time_ns_[index]) {
    AddFailure(CampaignFailure::kInvalidTelemetry);
  }
  if (!telemetry_time_seen_[index]) {
    first_telemetry_time_ns_[index] = arrival_time_ns;
  }
  telemetry_time_seen_[index] = true;
  last_telemetry_time_ns_[index] = arrival_time_ns;
  if (endpoint_graph_complete_ && !heartbeat_outage_active_ &&
      connected_telemetry_epoch_start_ns_ >= 0 &&
      RequiresPeriodicTelemetry(configuration_, index)) {
    if (telemetry_gap_baseline_seen_[index]) {
      const std::int64_t gap_ns =
          arrival_time_ns - last_connected_telemetry_time_ns_[index];
      const std::int64_t maximum_gap_ns =
          MaximumTelemetryGapNs(configuration_, index);
      connected_telemetry_gap_observed_ = true;
      connected_telemetry_gap_observed_by_stream_[index] = true;
      if (gap_ns <= 0 || gap_ns > maximum_gap_ns) {
        connected_telemetry_gap_failed_ = true;
        AddFailure(CampaignFailure::kTelemetryRate);
      }
    } else if (arrival_time_ns - connected_telemetry_epoch_start_ns_ >
               MaximumTelemetryGapNs(configuration_, index)) {
      connected_telemetry_gap_failed_ = true;
      AddFailure(CampaignFailure::kTelemetryRate);
    }
    telemetry_gap_baseline_seen_[index] = true;
    last_connected_telemetry_time_ns_[index] = arrival_time_ns;
    if (valid && current_connected_telemetry_samples_[index] < 2U) {
      ++current_connected_telemetry_samples_[index];
    }
  }
  if (!valid) {
    telemetry_invalid_[index] = true;
    AddFailure(CampaignFailure::kInvalidTelemetry);
  }
}

bool QualificationCampaignCore::BeginEvidence(std::int64_t start_time_ns) {
  if (failure_mask_ != 0U) {
    return false;
  }
  if (evidence_started_ || !heartbeat_seen_ || !diagnostics_seen_ ||
      !endpoint_graph_complete_ || start_time_ns < 0 ||
      last_diagnostics_.session_generation != current_session_id_ ||
      last_diagnostics_.motor_command_consumptions != 0U ||
      last_diagnostics_.motor_command_age_over_20_ms != 0U ||
      last_diagnostics_.motor_command_max_age_us != 0U) {
    AddFailure(CampaignFailure::kMotorAgeBaseline);
    return false;
  }
  if (!scheduler_.Start(start_time_ns)) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return false;
  }
  evidence_started_ = true;
  evidence_start_time_ns_ = start_time_ns;
  initial_session_id_ = current_session_id_;
  telemetry_gap_baseline_seen_.fill(false);
  current_connected_telemetry_samples_.fill(0U);
  connected_telemetry_epoch_start_ns_ = start_time_ns;
  ResetDiagnosticBaseline(last_diagnostics_);
  return true;
}

CampaignDueWork QualificationCampaignCore::Poll(std::int64_t now_ns) {
  if (!evidence_started_) {
    return {};
  }
  if (configuration_.profile.continuous_session_required && heartbeat_seen_ &&
      diagnostics_seen_ &&
      last_diagnostics_.session_generation != current_session_id_) {
    AddFailure(CampaignFailure::kSessionChanged);
  }
  if (heartbeat_seen_ &&
      now_ns - last_heartbeat_time_ns_ >= configuration_.heartbeat_loss_ns) {
    if (!heartbeat_outage_active_) {
      heartbeat_outage_active_ = true;
      heartbeat_outage_observed_time_ns_ = now_ns;
      telemetry_gap_baseline_seen_.fill(false);
      current_connected_telemetry_samples_.fill(0U);
      connected_telemetry_epoch_start_ns_ = -1;
      if (configuration_.profile.continuous_session_required) {
        AddFailure(CampaignFailure::kSessionInvalid);
      }
      if (configuration_.profile.mode == CampaignMode::kReconnectUsb) {
        const std::int64_t connected_start =
            last_endpoint_recovery_time_ns_ > 0
                ? last_endpoint_recovery_time_ns_
                : evidence_start_time_ns_;
        if (last_heartbeat_time_ns_ - connected_start < INT64_C(10000000000)) {
          AddFailure(CampaignFailure::kSessionInvalid);
        }
      }
    }
  }
  if (pending_transition_index_ < stored_transition_count_ &&
      now_ns - pending_transition_time_ns_ >
          configuration_.endpoint_recovery_limit_ns) {
    AddFailure(CampaignFailure::kEndpointRecoveryTimeout);
  }
  if (endpoint_graph_complete_ && !heartbeat_outage_active_ &&
      connected_telemetry_epoch_start_ns_ >= 0 &&
      now_ns >= connected_telemetry_epoch_start_ns_) {
    for (std::size_t index = 0U; index < telemetry_samples_.size(); ++index) {
      if (!RequiresPeriodicTelemetry(configuration_, index)) {
        continue;
      }
      const std::int64_t baseline_ns =
          telemetry_gap_baseline_seen_[index]
              ? last_connected_telemetry_time_ns_[index]
              : connected_telemetry_epoch_start_ns_;
      if (now_ns - baseline_ns > MaximumTelemetryGapNs(configuration_, index)) {
        connected_telemetry_gap_failed_ = true;
        AddFailure(CampaignFailure::kTelemetryRate);
      }
    }
  }
  CampaignDueWork work = scheduler_.Poll(now_ns);
  if (work.missed_command_mask != 0U || work.missed_service_rounds != 0U) {
    AddFailure(CampaignFailure::kScheduleMissed);
  }
  return work;
}

bool QualificationCampaignCore::RecordMotorCommand(
    std::uint8_t update_mask, const std::array<float, 4U>& targets) {
  bool valid = update_mask != 0U && (update_mask & UINT8_C(0xF0)) == 0U;
  for (const float target : targets) {
    valid = valid && std::isfinite(target) && target == 0.0F;
  }
  if (!valid) {
    AddFailure(CampaignFailure::kNonzeroMotorAttempt);
    return false;
  }
  RecordCommand(CampaignCommand::kMotor);
  return true;
}

void QualificationCampaignCore::RecordCommand(CampaignCommand command) {
  const std::size_t index = CommandIndex(command);
  if (!evidence_started_ || index >= command_publications_.size()) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return;
  }
  command_publications_[index] =
      SaturatingAdd(command_publications_[index], 1U);
}

void QualificationCampaignCore::RecordServiceResult(CampaignService service,
                                                    std::uint8_t result_code,
                                                    bool request_sent,
                                                    bool timed_out) {
  const std::size_t index = ServiceIndex(service);
  if (!evidence_started_ || index >= service_requests_.size()) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return;
  }
  if (request_sent) {
    service_requests_[index] = SaturatingAdd(service_requests_[index], 1U);
  }
  if (!timed_out && result_code == 0U) {
    if (!request_sent) {
      service_failures_[index] = SaturatingAdd(service_failures_[index], 1U);
      AddFailure(CampaignFailure::kInvalidConfiguration);
      return;
    }
    service_successes_[index] = SaturatingAdd(service_successes_[index], 1U);
  } else {
    service_failures_[index] = SaturatingAdd(service_failures_[index], 1U);
    if (configuration_.profile.continuous_session_required) {
      AddFailure(CampaignFailure::kServiceFailure);
    }
  }
}

void QualificationCampaignCore::RecordServiceSkipped(CampaignService service) {
  const std::size_t index = ServiceIndex(service);
  if (index >= service_skips_.size()) {
    AddFailure(CampaignFailure::kInvalidConfiguration);
    return;
  }
  service_skips_[index] = SaturatingAdd(service_skips_[index], 1U);
  if (configuration_.profile.continuous_session_required) {
    AddFailure(CampaignFailure::kServiceCoverage);
  }
}

bool QualificationCampaignCore::CampaignComplete(std::int64_t now_ns) const {
  if (!evidence_started_) {
    return false;
  }
  if (configuration_.profile.continuous_session_required) {
    return now_ns - evidence_start_time_ns_ >= configuration_.duration_ns;
  }
  if (observed_cycles_ < configuration_.profile.expected_cycles ||
      pending_transition_index_ < stored_transition_count_) {
    return false;
  }
  for (std::size_t index = 0U;
       index < current_connected_telemetry_samples_.size(); ++index) {
    if (RequiresPeriodicTelemetry(configuration_, index) &&
        current_connected_telemetry_samples_[index] < 2U) {
      return false;
    }
  }
  return true;
}

CampaignSummary QualificationCampaignCore::Finish(std::int64_t finish_time_ns) {
  CampaignSummary summary;
  summary.mode = configuration_.profile.mode;
  summary.profile = configuration_.profile;
  summary.configured_duration_ns = configuration_.duration_ns;
  summary.evidence_start_time_ns = evidence_start_time_ns_;
  summary.evidence_finish_time_ns = finish_time_ns;
  summary.command_publications = command_publications_;
  summary.skipped_commands = scheduler_.skipped_commands();
  summary.service_requests = service_requests_;
  summary.service_successes = service_successes_;
  summary.service_failures = service_failures_;
  summary.service_skips = service_skips_;
  summary.telemetry_samples = telemetry_samples_;
  summary.motor_command_consumptions = accumulated_motor_consumptions_;
  summary.motor_command_age_over_20_ms = accumulated_motor_age_over_20_ms_;
  summary.motor_command_max_age_us = motor_command_max_age_us_;
  summary.maximum_transport_interval_bytes_per_second =
      maximum_transport_interval_bytes_per_second_;
  summary.initial_session_id = initial_session_id_;
  summary.final_session_id = current_session_id_;
  summary.observed_cycles = observed_cycles_;
  summary.observed_mcu_resets = observed_mcu_resets_;
  summary.stored_transition_count = stored_transition_count_;
  summary.transitions = transitions_;
  const CampaignProfile canonical_profile =
      CampaignProfileForMode(configuration_.profile.mode);
  summary.canonical_profile =
      ProfilesEqual(configuration_.profile, canonical_profile) &&
      configuration_.duration_ns ==
          configuration_.profile.canonical_duration_ns &&
      configuration_.heartbeat_loss_ns == INT64_C(1000000000) &&
      configuration_.endpoint_recovery_limit_ns == INT64_C(5000000000) &&
      configuration_.minimum_telemetry_gap_ns == 0 &&
      configuration_.maximum_transport_bytes_per_second == 70000.0 &&
      (!configuration_.profile.continuous_session_required ||
       configuration_.require_button_stimulus) &&
      configuration_.require_valid_imu &&
      (configuration_.profile.continuous_session_required ||
       configuration_.profile.expected_cycles == 100U);

  if (!evidence_started_) {
    AddFailure(CampaignFailure::kEvidenceNotStarted);
  } else if (configuration_.profile.continuous_session_required &&
             finish_time_ns - evidence_start_time_ns_ <
                 configuration_.duration_ns) {
    AddFailure(CampaignFailure::kDurationIncomplete);
  }
  if (heartbeat_seen_ && diagnostics_seen_ &&
      last_diagnostics_.session_generation != current_session_id_) {
    AddFailure(CampaignFailure::kSessionChanged);
  }

  if (configuration_.profile.continuous_session_required) {
    for (std::size_t index = 0U; index < command_publications_.size();
         ++index) {
      const std::uint64_t expected =
          ScheduledSlotCount(configuration_.duration_ns,
                             configuration_.profile.command_rates[index]);
      if (command_publications_[index] != expected ||
          summary.skipped_commands[index] != 0U) {
        AddFailure(CampaignFailure::kCommandCountMismatch);
      }
    }
    const std::uint64_t expected_service_rounds =
        static_cast<std::uint64_t>(
            (configuration_.duration_ns - 1) /
            configuration_.profile.service_round_period_ns) +
        1U;
    for (std::size_t index = 0U; index < service_requests_.size(); ++index) {
      if (service_requests_[index] != expected_service_rounds ||
          service_successes_[index] != expected_service_rounds ||
          service_failures_[index] != 0U || service_skips_[index] != 0U) {
        AddFailure(CampaignFailure::kServiceCoverage);
      }
    }
  } else {
    if (observed_cycles_ < configuration_.profile.expected_cycles) {
      AddFailure(CampaignFailure::kSessionCycleMissing);
    }
    if (configuration_.profile.mode == CampaignMode::kResetMcu &&
        observed_mcu_resets_ < configuration_.profile.expected_cycles) {
      AddFailure(CampaignFailure::kResetCycleMissing);
    }
    for (std::size_t index = 0U;
         index < connected_telemetry_gap_observed_by_stream_.size(); ++index) {
      if (index == TelemetryIndex(CampaignTelemetry::kButtonEvents)) {
        continue;
      }
      if (index == TelemetryIndex(CampaignTelemetry::kImu) &&
          !configuration_.require_valid_imu) {
        continue;
      }
      if (!connected_telemetry_gap_observed_by_stream_[index]) {
        AddFailure(CampaignFailure::kTelemetryRate);
      }
    }
  }

  for (std::size_t index = 0U; index < stored_transition_count_; ++index) {
    if (!transitions_[index].endpoint_recovered ||
        transitions_[index].endpoint_recovery_ms > 5000U) {
      AddFailure(CampaignFailure::kEndpointRecoveryTimeout);
    }
    if (configuration_.profile.mode == CampaignMode::kResetMcu &&
        !transitions_[index].mcu_uptime_regression_observed) {
      AddFailure(CampaignFailure::kResetCycleMissing);
    }
  }
  if (evidence_started_) {
    for (std::size_t index = 0U; index < telemetry_time_seen_.size(); ++index) {
      if (index == TelemetryIndex(CampaignTelemetry::kButtonEvents) &&
          (!configuration_.profile.continuous_session_required ||
           !configuration_.require_button_stimulus)) {
        continue;
      }
      if (index == TelemetryIndex(CampaignTelemetry::kImu) &&
          !configuration_.require_valid_imu) {
        continue;
      }
      const std::int64_t maximum_gap_ns =
          MaximumTelemetryGapNs(configuration_, index);
      if (!telemetry_time_seen_[index] ||
          first_telemetry_time_ns_[index] - evidence_start_time_ns_ < 0 ||
          first_telemetry_time_ns_[index] - evidence_start_time_ns_ >
              maximum_gap_ns ||
          finish_time_ns - last_telemetry_time_ns_[index] > maximum_gap_ns) {
        AddFailure(CampaignFailure::kTelemetryRate);
      }
    }
  }

  if (accumulated_motor_consumptions_ == 0U) {
    AddFailure(CampaignFailure::kMotorAgeEvidenceMissing);
    summary.motor_age_p99 = CampaignMetricStatus::kNotObserved;
    summary.motor_age_maximum = CampaignMetricStatus::kNotObserved;
  } else {
    if (accumulated_motor_age_over_20_ms_ * UINT64_C(100) >
        accumulated_motor_consumptions_) {
      AddFailure(CampaignFailure::kMotorAgeP99);
      summary.motor_age_p99 = CampaignMetricStatus::kFail;
    } else {
      summary.motor_age_p99 = CampaignMetricStatus::kPass;
    }
    if (motor_command_max_age_us_ >= kMaximumMotorAgeUs) {
      AddFailure(CampaignFailure::kMotorAgeMaximum);
      summary.motor_age_maximum = CampaignMetricStatus::kFail;
    } else {
      summary.motor_age_maximum = CampaignMetricStatus::kPass;
    }
  }

  if (diagnostic_interval_observed_) {
    summary.internal_wire_traffic =
        maximum_transport_interval_bytes_per_second_ <
                configuration_.maximum_transport_bytes_per_second
            ? CampaignMetricStatus::kPass
            : CampaignMetricStatus::kFail;
  }
  FinalizeRates(&summary);
  FinalizeCoverage(&summary);
  summary.usb_outage_rotation =
      configuration_.profile.mode == CampaignMode::kReconnectUsb
          ? CampaignMetricStatus::kNotObserved
          : CampaignMetricStatus::kNotApplicable;
  if (!connected_telemetry_gap_observed_) {
    summary.connected_telemetry_gaps = CampaignMetricStatus::kNotObserved;
  } else if (connected_telemetry_gap_failed_) {
    summary.connected_telemetry_gaps = CampaignMetricStatus::kFail;
  } else {
    summary.connected_telemetry_gaps = CampaignMetricStatus::kPass;
  }
  summary.failure_mask = failure_mask_;
  summary.execution_passed = failure_mask_ == 0U;
  summary.release_qualified = false;
  return summary;
}

std::size_t QualificationCampaignCore::CommandIndex(CampaignCommand command) {
  return static_cast<std::size_t>(command);
}

std::size_t QualificationCampaignCore::ServiceIndex(CampaignService service) {
  return static_cast<std::size_t>(service);
}

std::size_t QualificationCampaignCore::TelemetryIndex(
    CampaignTelemetry stream) {
  return static_cast<std::size_t>(stream);
}

bool QualificationCampaignCore::IsForwardProgress(std::uint32_t current,
                                                  std::uint32_t previous) {
  const std::uint32_t delta = current - previous;
  return delta != 0U && delta < UINT32_C(0x80000000);
}

void QualificationCampaignCore::AddFailure(CampaignFailure failure) {
  failure_mask_ |= CampaignFailureBit(failure);
}

void QualificationCampaignCore::StartTransition(
    const CampaignHeartbeatObservation& observation, bool uptime_regression) {
  if (configuration_.profile.continuous_session_required) {
    AddFailure(CampaignFailure::kSessionChanged);
  }
  if (pending_transition_index_ < stored_transition_count_ &&
      !transitions_[pending_transition_index_].endpoint_recovered) {
    AddFailure(CampaignFailure::kEndpointRecoveryTimeout);
  }
  ++observed_cycles_;
  if (stored_transition_count_ < transitions_.size()) {
    CampaignTransition& transition = transitions_[stored_transition_count_];
    transition.cycle = observed_cycles_;
    transition.previous_session_id = current_session_id_;
    transition.new_session_id = observation.session_id;
    const std::int64_t gap_ns = std::max<std::int64_t>(
        0, observation.arrival_time_ns - last_heartbeat_time_ns_);
    transition.heartbeat_gap_ms =
        static_cast<std::uint64_t>(gap_ns / INT64_C(1000000));
    const std::int64_t connected_start = last_endpoint_recovery_time_ns_ > 0
                                             ? last_endpoint_recovery_time_ns_
                                             : evidence_start_time_ns_;
    const std::int64_t connected_end = last_heartbeat_time_ns_;
    transition.connected_interval_ms = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, connected_end - connected_start) /
        INT64_C(1000000));
    transition.heartbeat_outage_observed = heartbeat_outage_active_;
    transition.mcu_uptime_regression_observed = false;
    transition.reset_reason = kResetUnknown;
    pending_transition_index_ = stored_transition_count_;
    ++stored_transition_count_;
  } else {
    AddFailure(CampaignFailure::kSessionCycleMissing);
  }
  if (configuration_.profile.mode == CampaignMode::kReconnectUsb &&
      !heartbeat_outage_active_) {
    AddFailure(CampaignFailure::kSessionInvalid);
  }
  if (configuration_.profile.mode != CampaignMode::kResetMcu &&
      uptime_regression) {
    AddFailure(CampaignFailure::kUnexpectedReset);
  }
  current_session_id_ = observation.session_id;
  endpoint_graph_complete_ = false;
  telemetry_gap_baseline_seen_.fill(false);
  current_connected_telemetry_samples_.fill(0U);
  connected_telemetry_epoch_start_ns_ = -1;
  pending_transition_time_ns_ = observation.arrival_time_ns;
  last_transition_index_ = observed_cycles_ <= stored_transition_count_
                               ? stored_transition_count_ - 1U
                               : kMaximumCampaignTransitions;
  if (unbound_mcu_reset_observed_ &&
      last_transition_index_ < stored_transition_count_) {
    transitions_[last_transition_index_].mcu_uptime_regression_observed = true;
    transitions_[last_transition_index_].reset_reason =
        unbound_mcu_reset_reason_;
    unbound_mcu_reset_observed_ = false;
    unbound_mcu_reset_reason_ = kResetUnknown;
  }
}

void QualificationCampaignCore::AccumulateDiagnosticDelta(
    const CampaignDiagnosticsObservation& observation) {
  if (observation.transport_rx_bytes < last_diagnostics_.transport_rx_bytes ||
      observation.transport_tx_bytes < last_diagnostics_.transport_tx_bytes ||
      observation.hard_error_total < last_diagnostics_.hard_error_total ||
      observation.motor_command_consumptions <
          last_diagnostics_.motor_command_consumptions ||
      observation.motor_command_age_over_20_ms <
          last_diagnostics_.motor_command_age_over_20_ms ||
      observation.motor_command_max_age_us <
          last_diagnostics_.motor_command_max_age_us) {
    AddFailure(CampaignFailure::kDiagnosticRegression);
    return;
  }
  for (std::size_t index = 0U; index < observation.monotonic_counters.size();
       ++index) {
    if (observation.monotonic_counters[index] <
        last_diagnostics_.monotonic_counters[index]) {
      AddFailure(CampaignFailure::kDiagnosticRegression);
      return;
    }
  }
  if (observation.hard_error_total != last_diagnostics_.hard_error_total) {
    AddFailure(CampaignFailure::kDiagnosticError);
  }
  if (configuration_.profile.mode != CampaignMode::kResetMcu &&
      observation.reset_reason != initial_reset_reason_) {
    AddFailure(CampaignFailure::kUnexpectedReset);
  }

  const std::int64_t interval_ns =
      observation.arrival_time_ns - last_diagnostics_.arrival_time_ns;
  if (interval_ns > 0) {
    const std::uint64_t byte_delta =
        observation.transport_rx_bytes - last_diagnostics_.transport_rx_bytes +
        observation.transport_tx_bytes - last_diagnostics_.transport_tx_bytes;
    const double rate = static_cast<double>(byte_delta) *
                        (static_cast<double>(kNanosecondsPerSecond) /
                         static_cast<double>(interval_ns));
    maximum_transport_interval_bytes_per_second_ =
        std::max(maximum_transport_interval_bytes_per_second_, rate);
    diagnostic_interval_observed_ = true;
    if (rate >= configuration_.maximum_transport_bytes_per_second) {
      AddFailure(CampaignFailure::kTransportRate);
    }
  }

  accumulated_motor_consumptions_ = SaturatingAdd(
      accumulated_motor_consumptions_,
      static_cast<std::uint64_t>(observation.motor_command_consumptions -
                                 last_diagnostics_.motor_command_consumptions));
  accumulated_motor_age_over_20_ms_ =
      SaturatingAdd(accumulated_motor_age_over_20_ms_,
                    static_cast<std::uint64_t>(
                        observation.motor_command_age_over_20_ms -
                        last_diagnostics_.motor_command_age_over_20_ms));
  motor_command_max_age_us_ =
      std::max(motor_command_max_age_us_, observation.motor_command_max_age_us);
}

void QualificationCampaignCore::ResetDiagnosticBaseline(
    const CampaignDiagnosticsObservation& observation) {
  last_diagnostics_ = observation;
  motor_command_max_age_us_ =
      std::max(motor_command_max_age_us_, observation.motor_command_max_age_us);
}

void QualificationCampaignCore::FinalizeRates(CampaignSummary* summary) {
  if (!evidence_started_ ||
      summary->evidence_finish_time_ns <= evidence_start_time_ns_) {
    return;
  }
  const double duration_seconds =
      static_cast<double>(summary->evidence_finish_time_ns -
                          evidence_start_time_ns_) /
      static_cast<double>(kNanosecondsPerSecond);
  for (std::size_t index = 0U; index < telemetry_samples_.size(); ++index) {
    summary->telemetry_rates_hz[index] =
        static_cast<double>(telemetry_samples_[index]) / duration_seconds;
    if (!configuration_.profile.continuous_session_required) {
      continue;
    }
    if (index == TelemetryIndex(CampaignTelemetry::kButtonEvents)) {
      if (configuration_.require_button_stimulus &&
          summary->telemetry_rates_hz[index] < 1.0) {
        AddFailure(CampaignFailure::kTelemetryRate);
      }
      continue;
    }
    if (index == TelemetryIndex(CampaignTelemetry::kImu) &&
        !configuration_.require_valid_imu) {
      continue;
    }
    const double expected = kTelemetryRatesHz[index];
    const double minimum = expected * (1.0 - kTelemetryRateTolerance);
    const double maximum = expected * (1.0 + kTelemetryRateTolerance);
    if (summary->telemetry_rates_hz[index] < minimum ||
        summary->telemetry_rates_hz[index] > maximum) {
      AddFailure(CampaignFailure::kTelemetryRate);
    }
  }
}

void QualificationCampaignCore::FinalizeCoverage(CampaignSummary* summary) {
  const std::size_t button_index =
      TelemetryIndex(CampaignTelemetry::kButtonEvents);
  if (configuration_.require_button_stimulus) {
    const bool button_observed = telemetry_samples_[button_index] > 0U &&
                                 !telemetry_invalid_[button_index];
    summary->button_stimulus = button_observed ? CampaignMetricStatus::kPass
                                               : CampaignMetricStatus::kFail;
    if (!button_observed) {
      AddFailure(CampaignFailure::kTelemetryRate);
    }
  } else {
    summary->button_stimulus = CampaignMetricStatus::kNotObserved;
  }
  const std::size_t imu_index = TelemetryIndex(CampaignTelemetry::kImu);
  if (!configuration_.require_valid_imu) {
    summary->imu_function = CampaignMetricStatus::kNotObserved;
  } else if (telemetry_samples_[imu_index] > 0U &&
             !telemetry_invalid_[imu_index]) {
    summary->imu_function = CampaignMetricStatus::kPass;
  } else {
    summary->imu_function = CampaignMetricStatus::kFail;
  }
}

}  // namespace mentor_pi_bringup
