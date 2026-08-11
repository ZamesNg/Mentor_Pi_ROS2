// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/motor_commissioning_core.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace mentor_pi_bringup {
namespace {

constexpr std::uint8_t kHeartbeatReady = 1U;
constexpr std::uint8_t kHeartbeatDegraded = 2U;
constexpr std::uint8_t kSessionActive = 3U;
constexpr float kTargetToleranceRps = 0.0001F;

bool IsReadyHeartbeatState(std::uint8_t state) {
  return state == kHeartbeatReady || state == kHeartbeatDegraded;
}

std::int64_t SaturatingDifference(std::int64_t left, std::int64_t right) {
  if (right > 0 && left < std::numeric_limits<std::int64_t>::min() + right) {
    return std::numeric_limits<std::int64_t>::min();
  }
  if (right < 0 && left > std::numeric_limits<std::int64_t>::max() + right) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return left - right;
}

}  // namespace

const char* MotorCommissioningFailureName(MotorCommissioningFailure failure) {
  switch (failure) {
    case MotorCommissioningFailure::kNone:
      return "NONE";
    case MotorCommissioningFailure::kInvalidAcknowledgement:
      return "INVALID_ACKNOWLEDGEMENT";
    case MotorCommissioningFailure::kInvalidMotorId:
      return "INVALID_MOTOR_ID";
    case MotorCommissioningFailure::kInvalidTarget:
      return "INVALID_TARGET";
    case MotorCommissioningFailure::kInvalidDuration:
      return "INVALID_DURATION";
    case MotorCommissioningFailure::kPrerequisiteTimeout:
      return "PREREQUISITE_TIMEOUT";
    case MotorCommissioningFailure::kMotionAuthorizationLost:
      return "MOTION_AUTHORIZATION_LOST";
    case MotorCommissioningFailure::kHeartbeatLost:
      return "HEARTBEAT_LOST";
    case MotorCommissioningFailure::kHeartbeatNotReady:
      return "HEARTBEAT_NOT_READY";
    case MotorCommissioningFailure::kSessionInvalid:
      return "SESSION_INVALID";
    case MotorCommissioningFailure::kSessionChanged:
      return "SESSION_CHANGED";
    case MotorCommissioningFailure::kMcuResetDetected:
      return "MCU_RESET_DETECTED";
    case MotorCommissioningFailure::kDiagnosticsLost:
      return "DIAGNOSTICS_LOST";
    case MotorCommissioningFailure::kDiagnosticsNotActive:
      return "DIAGNOSTICS_NOT_ACTIVE";
    case MotorCommissioningFailure::kDiagnosticsSessionMismatch:
      return "DIAGNOSTICS_SESSION_MISMATCH";
    case MotorCommissioningFailure::kMotorStateLost:
      return "MOTOR_STATE_LOST";
    case MotorCommissioningFailure::kMotorStateInvalid:
      return "MOTOR_STATE_INVALID";
    case MotorCommissioningFailure::kMotorStateUnexpectedTarget:
      return "MOTOR_STATE_UNEXPECTED_TARGET";
    case MotorCommissioningFailure::kMotorWatchdogTriggered:
      return "MOTOR_WATCHDOG_TRIGGERED";
    case MotorCommissioningFailure::kDriveCadenceLost:
      return "DRIVE_CADENCE_LOST";
    case MotorCommissioningFailure::kMotorOverspeed:
      return "MOTOR_OVERSPEED";
    case MotorCommissioningFailure::kMotorWrongDirection:
      return "MOTOR_WRONG_DIRECTION";
    case MotorCommissioningFailure::kUnselectedMotorMotion:
      return "UNSELECTED_MOTOR_MOTION";
    case MotorCommissioningFailure::kCommandPublisherConflict:
      return "COMMAND_PUBLISHER_CONFLICT";
    case MotorCommissioningFailure::kCommandRejected:
      return "COMMAND_REJECTED_OR_LOCKED_IMAGE";
    case MotorCommissioningFailure::kPreStopNotConfirmed:
      return "PRE_STOP_NOT_CONFIRMED";
    case MotorCommissioningFailure::kTargetNotObserved:
      return "TARGET_NOT_OBSERVED_OR_LOCKED_IMAGE";
    case MotorCommissioningFailure::kPhysicalResponseNotObserved:
      return "PHYSICAL_RESPONSE_NOT_OBSERVED";
    case MotorCommissioningFailure::kPostStopNotConfirmed:
      return "POST_STOP_NOT_CONFIRMED";
    case MotorCommissioningFailure::kInterrupted:
      return "INTERRUPTED";
  }
  return "UNKNOWN";
}

MotorCommissioningFailure ValidateMotorCommissioningConfiguration(
    const MotorCommissioningConfiguration& configuration) {
  if (std::string_view{configuration.acknowledgement} !=
      kMotorCommissioningAcknowledgement) {
    return MotorCommissioningFailure::kInvalidAcknowledgement;
  }
  if (configuration.motor_id < 1U || configuration.motor_id > 4U) {
    return MotorCommissioningFailure::kInvalidMotorId;
  }
  if (!std::isfinite(configuration.target_rps) ||
      std::fabs(configuration.target_rps) < 0.01F ||
      std::fabs(configuration.target_rps) > 0.25F) {
    return MotorCommissioningFailure::kInvalidTarget;
  }
  if (configuration.duration_ms < 100U || configuration.duration_ms > 5000U) {
    return MotorCommissioningFailure::kInvalidDuration;
  }
  return MotorCommissioningFailure::kNone;
}

MotorCommissioningCore::MotorCommissioningCore(
    MotorCommissioningConfiguration configuration)
    : configuration_(std::move(configuration)),
      phase_start_time_ms_(configuration_.start_time_ms) {
  failure_ = ValidateMotorCommissioningConfiguration(configuration_);
  if (failure_ != MotorCommissioningFailure::kNone) {
    phase_ = MotorCommissioningPhase::kFailed;
  }
}

void MotorCommissioningCore::ObserveMotionAuthorizationPublisher(bool valid) {
  motion_authorization_publisher_valid_ = valid;
}

void MotorCommissioningCore::ObserveMotionAuthorization(
    const CommissioningMotionAuthorizationObservation& observation) {
  motion_authorization_seen_ = true;
  motion_authorization_ = observation;
}

void MotorCommissioningCore::ObserveHeartbeat(
    const CommissioningHeartbeatObservation& observation) {
  if (heartbeat_seen_ &&
      observation.agent_session_id == heartbeat_.agent_session_id &&
      (SerialNumberRegressed(observation.uptime_ms, heartbeat_.uptime_ms) ||
       SerialNumberRegressed(observation.sequence, heartbeat_.sequence))) {
    heartbeat_discontinuity_ = true;
  }
  heartbeat_seen_ = true;
  heartbeat_ = observation;
}

void MotorCommissioningCore::ObserveDiagnostics(
    const CommissioningDiagnosticsObservation& observation) {
  if (diagnostics_seen_ &&
      observation.session_generation == diagnostics_.session_generation &&
      SerialNumberRegressed(observation.uptime_ms, diagnostics_.uptime_ms)) {
    diagnostics_discontinuity_ = true;
  }
  diagnostics_seen_ = true;
  diagnostics_ = observation;
}

void MotorCommissioningCore::ObserveMotorState(
    const CommissioningMotorStateObservation& observation) {
  motor_state_seen_ = true;
  motor_state_ = observation;
  if (!MotorStateIsFinite() || configuration_.motor_id < 1U ||
      configuration_.motor_id > 4U) {
    return;
  }

  const std::size_t index = SelectedMotorIndex();
  latest_encoder_ = observation.encoder_count;
  final_measured_rps_ = observation.measured_rps[index];
  peak_absolute_measured_rps_ = std::max(
      peak_absolute_measured_rps_, std::fabs(observation.measured_rps[index]));

  if (phase_ == MotorCommissioningPhase::kPreStop && pre_stop_commands_ > 0U &&
      observation.arrival_time_ms >= phase_start_time_ms_ &&
      MotorTargetsAreZero()) {
    pre_stop_zero_confirmed_ = true;
  }
  if (phase_ == MotorCommissioningPhase::kDrive && drive_commands_ > 0U &&
      observation.arrival_time_ms >= phase_start_time_ms_ &&
      SelectedTargetMatches(configuration_.target_rps)) {
    target_observed_ = true;
    const std::int64_t selected_delta = EncoderDelta(index);
    if (std::fabs(observation.measured_rps[index]) >= ResponseThresholdRps() &&
        SelectedResponseDirectionMatches() &&
        ((configuration_.target_rps > 0.0F &&
          selected_delta >= kMinimumEncoderResponseTicks) ||
         (configuration_.target_rps < 0.0F &&
          selected_delta <= -kMinimumEncoderResponseTicks))) {
      physical_response_observed_ = true;
    }
  }
  if (phase_ == MotorCommissioningPhase::kPostStop &&
      post_stop_commands_ > 0U &&
      observation.arrival_time_ms >= phase_start_time_ms_) {
    const bool targets_are_zero = MotorTargetsAreZero();
    // Remember a zero target independently from the final stationary proof.
    // This preserves the fail-closed check for a target that returns while the
    // measured velocity is still settling after the stop command.
    post_stop_zero_seen_ = post_stop_zero_seen_ || targets_are_zero;
    post_stop_zero_confirmed_ = targets_are_zero && MotorsAreStationary();
  }
}

void MotorCommissioningCore::ObserveCommandPublisherConflict(bool conflict) {
  command_publisher_conflict_ = conflict;
}

MotorCommissioningAction MotorCommissioningCore::Tick(std::int64_t now_ms) {
  if (complete()) {
    return {};
  }

  if (phase_ == MotorCommissioningPhase::kAwaitingPrerequisites) {
    if (command_publisher_conflict_) {
      BeginPostStop(MotorCommissioningFailure::kCommandPublisherConflict,
                    now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    if (heartbeat_discontinuity_ || diagnostics_discontinuity_) {
      BeginPostStop(MotorCommissioningFailure::kMcuResetDetected, now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    if (PrerequisitesSatisfied(now_ms)) {
      LockPrerequisites(now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    if (now_ms < configuration_.start_time_ms ||
        now_ms - configuration_.start_time_ms >= kPrerequisiteTimeoutMs) {
      BeginPostStop(MotorCommissioningFailure::kPrerequisiteTimeout, now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    return {};
  }

  const MotorCommissioningFailure runtime_failure = RuntimeFailure(now_ms);
  if (runtime_failure != MotorCommissioningFailure::kNone) {
    if (phase_ != MotorCommissioningPhase::kPostStop) {
      BeginPostStop(runtime_failure, now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    RememberFailure(runtime_failure);
  }

  if (phase_ == MotorCommissioningPhase::kPreStop) {
    if (now_ms - phase_start_time_ms_ >= kStopPhaseDurationMs &&
        pre_stop_commands_ >= kStopCommandCount) {
      if (!pre_stop_zero_confirmed_) {
        BeginPostStop(MotorCommissioningFailure::kPreStopNotConfirmed, now_ms);
        return {MotorCommissioningCommand::kStop};
      }
      phase_ = MotorCommissioningPhase::kDrive;
      phase_start_time_ms_ = now_ms;
      encoder_baseline_ = latest_encoder_;
      encoder_baseline_seen_ = true;
      peak_absolute_measured_rps_ = 0.0F;
      final_measured_rps_ = motor_state_.measured_rps[SelectedMotorIndex()];
      return {MotorCommissioningCommand::kDrive};
    }
    return {MotorCommissioningCommand::kStop};
  }

  if (phase_ == MotorCommissioningPhase::kDrive) {
    if (now_ms - phase_start_time_ms_ >=
        static_cast<std::int64_t>(configuration_.duration_ms)) {
      const MotorCommissioningFailure completion_failure =
          target_observed_ ? MotorCommissioningFailure::kNone
                           : MotorCommissioningFailure::kTargetNotObserved;
      BeginPostStop(completion_failure, now_ms);
      return {MotorCommissioningCommand::kStop};
    }
    return {MotorCommissioningCommand::kDrive};
  }

  if (phase_ == MotorCommissioningPhase::kPostStop) {
    if (now_ms - phase_start_time_ms_ >= kStopPhaseDurationMs &&
        post_stop_commands_ >= kStopCommandCount) {
      FinishPostStop();
      return {};
    }
    return {MotorCommissioningCommand::kStop};
  }

  return {};
}

void MotorCommissioningCore::RecordCommandPublished(
    MotorCommissioningCommand command, std::int64_t publish_time_ms) {
  switch (command) {
    case MotorCommissioningCommand::kNone:
      return;
    case MotorCommissioningCommand::kStop:
      if (phase_ == MotorCommissioningPhase::kPreStop) {
        ++pre_stop_commands_;
      } else if (phase_ == MotorCommissioningPhase::kPostStop) {
        ++post_stop_commands_;
      }
      return;
    case MotorCommissioningCommand::kDrive:
      if (phase_ == MotorCommissioningPhase::kDrive) {
        ++drive_commands_;
        last_drive_publish_time_ms_ = publish_time_ms;
      }
      return;
  }
}

void MotorCommissioningCore::RequestAbort(std::int64_t now_ms) {
  if (complete() || abort_requested_) {
    return;
  }
  abort_requested_ = true;
  BeginPostStop(MotorCommissioningFailure::kInterrupted, now_ms);
}

bool MotorCommissioningCore::complete() const {
  return phase_ == MotorCommissioningPhase::kSucceeded ||
         phase_ == MotorCommissioningPhase::kFailed;
}

MotorCommissioningPhase MotorCommissioningCore::phase() const { return phase_; }

MotorCommissioningSummary MotorCommissioningCore::summary() const {
  MotorCommissioningSummary result;
  result.passed = phase_ == MotorCommissioningPhase::kSucceeded;
  result.failure = failure_;
  result.phase = phase_;
  result.agent_session_id = locked_session_id_;
  result.motor_id = configuration_.motor_id;
  result.target_rps = configuration_.target_rps;
  result.duration_ms = configuration_.duration_ms;
  result.target_observed = target_observed_;
  result.physical_response_observed = physical_response_observed_;
  result.zero_confirmed = post_stop_zero_confirmed_;
  if (encoder_baseline_seen_) {
    const std::size_t index = SelectedMotorIndex();
    result.encoder_delta =
        SaturatingDifference(latest_encoder_[index], encoder_baseline_[index]);
  }
  result.peak_absolute_measured_rps = peak_absolute_measured_rps_;
  result.final_measured_rps = final_measured_rps_;
  result.pre_stop_commands = pre_stop_commands_;
  result.drive_commands = drive_commands_;
  result.post_stop_commands = post_stop_commands_;
  return result;
}

bool MotorCommissioningCore::PrerequisitesSatisfied(std::int64_t now_ms) const {
  return motion_authorization_publisher_valid_ && motion_authorization_seen_ &&
         heartbeat_seen_ &&
         IsFresh(heartbeat_.arrival_time_ms, now_ms, kHeartbeatFreshnessMs) &&
         heartbeat_.agent_session_id != 0U &&
         motion_authorization_.agent_session_id ==
             heartbeat_.agent_session_id &&
         IsReadyHeartbeatState(heartbeat_.state) && diagnostics_seen_ &&
         IsFresh(diagnostics_.arrival_time_ms, now_ms,
                 kDiagnosticsFreshnessMs) &&
         diagnostics_.session_generation == heartbeat_.agent_session_id &&
         diagnostics_.session_state == kSessionActive && motor_state_seen_ &&
         IsFresh(motor_state_.arrival_time_ms, now_ms,
                 kMotorStateFreshnessMs) &&
         MotorStateIsFinite() && MotorTargetsAreZero() &&
         MotorsAreStationary() && motor_state_.watchdog_stop_mask == 0U &&
         !heartbeat_discontinuity_ && !diagnostics_discontinuity_ &&
         !command_publisher_conflict_;
}

MotorCommissioningFailure MotorCommissioningCore::RuntimeFailure(
    std::int64_t now_ms) const {
  if (command_publisher_conflict_) {
    return MotorCommissioningFailure::kCommandPublisherConflict;
  }
  if (phase_ == MotorCommissioningPhase::kDrive && drive_commands_ > 0U &&
      (now_ms < last_drive_publish_time_ms_ ||
       now_ms - last_drive_publish_time_ms_ > kMaximumDriveCommandGapMs)) {
    return MotorCommissioningFailure::kDriveCadenceLost;
  }
  if (!motion_authorization_publisher_valid_ || !motion_authorization_seen_ ||
      motion_authorization_.configuration_generation !=
          locked_configuration_generation_ ||
      motion_authorization_.agent_session_id != locked_session_id_) {
    return MotorCommissioningFailure::kMotionAuthorizationLost;
  }
  if (!heartbeat_seen_ ||
      !IsFresh(heartbeat_.arrival_time_ms, now_ms, kHeartbeatFreshnessMs)) {
    return MotorCommissioningFailure::kHeartbeatLost;
  }
  if (heartbeat_.agent_session_id == 0U) {
    return MotorCommissioningFailure::kSessionInvalid;
  }
  if (heartbeat_.agent_session_id != locked_session_id_) {
    return MotorCommissioningFailure::kSessionChanged;
  }
  if (heartbeat_discontinuity_) {
    return MotorCommissioningFailure::kMcuResetDetected;
  }
  if (!IsReadyHeartbeatState(heartbeat_.state)) {
    return MotorCommissioningFailure::kHeartbeatNotReady;
  }
  if (!diagnostics_seen_ ||
      !IsFresh(diagnostics_.arrival_time_ms, now_ms, kDiagnosticsFreshnessMs)) {
    return MotorCommissioningFailure::kDiagnosticsLost;
  }
  if (diagnostics_.session_generation != locked_session_id_) {
    return MotorCommissioningFailure::kDiagnosticsSessionMismatch;
  }
  if (diagnostics_discontinuity_) {
    return MotorCommissioningFailure::kMcuResetDetected;
  }
  if (diagnostics_.session_state != kSessionActive) {
    return MotorCommissioningFailure::kDiagnosticsNotActive;
  }
  if (diagnostics_.command_rejections != baseline_command_rejections_ ||
      diagnostics_.motor_command_rejections[SelectedMotorIndex()] !=
          baseline_motor_rejections_) {
    return MotorCommissioningFailure::kCommandRejected;
  }
  if (diagnostics_.motor_lease_expiries != baseline_motor_lease_expiries_ ||
      diagnostics_.motor_watchdog_trips != baseline_motor_watchdog_trips_) {
    return MotorCommissioningFailure::kMotorWatchdogTriggered;
  }
  if (!motor_state_seen_ ||
      !IsFresh(motor_state_.arrival_time_ms, now_ms, kMotorStateFreshnessMs)) {
    return MotorCommissioningFailure::kMotorStateLost;
  }
  if (!MotorStateIsFinite()) {
    return MotorCommissioningFailure::kMotorStateInvalid;
  }
  if (motor_state_.watchdog_stop_mask != 0U) {
    return MotorCommissioningFailure::kMotorWatchdogTriggered;
  }
  const MotorCommissioningFailure target_failure = ValidateMotorStateForPhase();
  if (target_failure != MotorCommissioningFailure::kNone) {
    return target_failure;
  }
  return ValidatePhysicalMotion();
}

MotorCommissioningFailure MotorCommissioningCore::ValidateMotorStateForPhase()
    const {
  if (phase_ == MotorCommissioningPhase::kPreStop) {
    return MotorTargetsAreZero()
               ? MotorCommissioningFailure::kNone
               : MotorCommissioningFailure::kMotorStateUnexpectedTarget;
  }
  if (phase_ == MotorCommissioningPhase::kDrive ||
      phase_ == MotorCommissioningPhase::kPostStop) {
    const std::size_t selected_index = SelectedMotorIndex();
    for (std::size_t index = 0U; index < motor_state_.target_rps.size();
         ++index) {
      if (index == selected_index) {
        if (motor_state_.target_rps[index] == 0.0F ||
            std::fabs(motor_state_.target_rps[index] -
                      configuration_.target_rps) <= kTargetToleranceRps) {
          continue;
        }
      } else if (motor_state_.target_rps[index] == 0.0F) {
        continue;
      }
      return MotorCommissioningFailure::kMotorStateUnexpectedTarget;
    }
    if (phase_ == MotorCommissioningPhase::kPostStop && post_stop_zero_seen_ &&
        !MotorTargetsAreZero()) {
      return MotorCommissioningFailure::kMotorStateUnexpectedTarget;
    }
  }
  return MotorCommissioningFailure::kNone;
}

MotorCommissioningFailure MotorCommissioningCore::ValidatePhysicalMotion()
    const {
  if (phase_ != MotorCommissioningPhase::kDrive &&
      phase_ != MotorCommissioningPhase::kPostStop) {
    return MotorCommissioningFailure::kNone;
  }

  const std::size_t selected_index = SelectedMotorIndex();
  if (std::fabs(motor_state_.measured_rps[selected_index]) >
      kMaximumMeasuredRps) {
    return MotorCommissioningFailure::kMotorOverspeed;
  }
  for (std::size_t index = 0U; index < motor_state_.measured_rps.size();
       ++index) {
    if (index == selected_index) {
      continue;
    }
    const std::int64_t delta = EncoderDelta(index);
    if (std::fabs(motor_state_.measured_rps[index]) > kUnselectedMotionRps ||
        delta >= kMinimumEncoderResponseTicks ||
        delta <= -kMinimumEncoderResponseTicks) {
      return MotorCommissioningFailure::kUnselectedMotorMotion;
    }
  }

  if (phase_ == MotorCommissioningPhase::kDrive) {
    const std::int64_t delta = EncoderDelta(selected_index);
    if ((configuration_.target_rps > 0.0F &&
         delta <= -kMinimumEncoderResponseTicks) ||
        (configuration_.target_rps < 0.0F &&
         delta >= kMinimumEncoderResponseTicks)) {
      return MotorCommissioningFailure::kMotorWrongDirection;
    }
  }
  return MotorCommissioningFailure::kNone;
}

void MotorCommissioningCore::LockPrerequisites(std::int64_t now_ms) {
  locked_session_id_ = heartbeat_.agent_session_id;
  locked_configuration_generation_ =
      motion_authorization_.configuration_generation;
  baseline_command_rejections_ = diagnostics_.command_rejections;
  baseline_motor_rejections_ =
      diagnostics_.motor_command_rejections[SelectedMotorIndex()];
  baseline_motor_lease_expiries_ = diagnostics_.motor_lease_expiries;
  baseline_motor_watchdog_trips_ = diagnostics_.motor_watchdog_trips;
  encoder_baseline_seen_ = true;
  encoder_baseline_ = motor_state_.encoder_count;
  latest_encoder_ = encoder_baseline_;
  final_measured_rps_ = motor_state_.measured_rps[SelectedMotorIndex()];
  peak_absolute_measured_rps_ = std::fabs(final_measured_rps_);
  phase_ = MotorCommissioningPhase::kPreStop;
  phase_start_time_ms_ = now_ms;
  pre_stop_zero_confirmed_ = false;
}

void MotorCommissioningCore::BeginPostStop(MotorCommissioningFailure failure,
                                           std::int64_t now_ms) {
  RememberFailure(failure);
  phase_ = MotorCommissioningPhase::kPostStop;
  phase_start_time_ms_ = now_ms;
  post_stop_commands_ = 0U;
  post_stop_zero_confirmed_ = false;
  post_stop_zero_seen_ = false;
}

void MotorCommissioningCore::FinishPostStop() {
  if (!post_stop_zero_confirmed_ || !MotorTargetsAreZero() ||
      !MotorsAreStationary()) {
    RememberFailure(MotorCommissioningFailure::kPostStopNotConfirmed);
  }
  if (failure_ == MotorCommissioningFailure::kNone && !target_observed_) {
    RememberFailure(MotorCommissioningFailure::kTargetNotObserved);
  }
  if (failure_ == MotorCommissioningFailure::kNone &&
      !physical_response_observed_) {
    RememberFailure(MotorCommissioningFailure::kPhysicalResponseNotObserved);
  }
  if (failure_ == MotorCommissioningFailure::kNone && target_observed_ &&
      physical_response_observed_ && post_stop_zero_confirmed_ &&
      MotorTargetsAreZero() && MotorsAreStationary()) {
    phase_ = MotorCommissioningPhase::kSucceeded;
  } else {
    phase_ = MotorCommissioningPhase::kFailed;
  }
}

void MotorCommissioningCore::RememberFailure(
    MotorCommissioningFailure failure) {
  if (failure_ == MotorCommissioningFailure::kNone &&
      failure != MotorCommissioningFailure::kNone) {
    failure_ = failure;
  }
}

bool MotorCommissioningCore::IsFresh(std::int64_t arrival_time_ms,
                                     std::int64_t now_ms,
                                     std::int64_t maximum_age_ms) {
  return now_ms >= arrival_time_ms &&
         now_ms - arrival_time_ms <= maximum_age_ms;
}

bool MotorCommissioningCore::MotorTargetsAreZero() const {
  return std::all_of(motor_state_.target_rps.begin(),
                     motor_state_.target_rps.end(),
                     [](float target) { return target == 0.0F; });
}

bool MotorCommissioningCore::MotorsAreStationary() const {
  return std::all_of(
      motor_state_.measured_rps.begin(), motor_state_.measured_rps.end(),
      [](float measured) { return std::fabs(measured) < kMinimumResponseRps; });
}

bool MotorCommissioningCore::MotorStateIsFinite() const {
  const auto finite = [](float value) { return std::isfinite(value); };
  return std::all_of(motor_state_.target_rps.begin(),
                     motor_state_.target_rps.end(), finite) &&
         std::all_of(motor_state_.measured_rps.begin(),
                     motor_state_.measured_rps.end(), finite);
}

bool MotorCommissioningCore::SelectedTargetMatches(float expected) const {
  const std::size_t selected_index = SelectedMotorIndex();
  for (std::size_t index = 0U; index < motor_state_.target_rps.size();
       ++index) {
    const float required = index == selected_index ? expected : 0.0F;
    const bool matches = index == selected_index
                             ? std::fabs(motor_state_.target_rps[index] -
                                         required) <= kTargetToleranceRps
                             : motor_state_.target_rps[index] == 0.0F;
    if (!matches) {
      return false;
    }
  }
  return true;
}

bool MotorCommissioningCore::SelectedResponseDirectionMatches() const {
  const float measured = motor_state_.measured_rps[SelectedMotorIndex()];
  return (configuration_.target_rps > 0.0F && measured > 0.0F) ||
         (configuration_.target_rps < 0.0F && measured < 0.0F);
}

float MotorCommissioningCore::ResponseThresholdRps() const {
  return std::max(kMinimumResponseRps,
                  std::fabs(configuration_.target_rps) * 0.1F);
}

std::int64_t MotorCommissioningCore::EncoderDelta(std::size_t index) const {
  return SaturatingDifference(latest_encoder_[index], encoder_baseline_[index]);
}

bool MotorCommissioningCore::SerialNumberRegressed(std::uint32_t current,
                                                   std::uint32_t previous) {
  const std::uint32_t delta = current - previous;
  return delta >= UINT32_C(0x80000000);
}

std::size_t MotorCommissioningCore::SelectedMotorIndex() const {
  return static_cast<std::size_t>(configuration_.motor_id - 1U);
}

}  // namespace mentor_pi_bringup
