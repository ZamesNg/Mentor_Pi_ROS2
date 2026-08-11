// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/motor_commissioning_core.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace {

using mentor_pi_bringup::CommissioningDiagnosticsObservation;
using mentor_pi_bringup::CommissioningHeartbeatObservation;
using mentor_pi_bringup::CommissioningMotionAuthorizationObservation;
using mentor_pi_bringup::CommissioningMotorStateObservation;
using mentor_pi_bringup::kMotorCommissioningAcknowledgement;
using mentor_pi_bringup::MotorCommissioningAction;
using mentor_pi_bringup::MotorCommissioningCommand;
using mentor_pi_bringup::MotorCommissioningConfiguration;
using mentor_pi_bringup::MotorCommissioningCore;
using mentor_pi_bringup::MotorCommissioningFailure;
using mentor_pi_bringup::MotorCommissioningFailureName;
using mentor_pi_bringup::MotorCommissioningPhase;
using mentor_pi_bringup::ValidateMotorCommissioningConfiguration;

constexpr std::uint32_t kSessionId = 42U;
constexpr std::uint32_t kConfigurationGeneration = 7U;

int g_failures = 0;
std::int64_t g_observation_time_ms = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

MotorCommissioningConfiguration ValidConfiguration() {
  MotorCommissioningConfiguration configuration;
  configuration.acknowledgement =
      std::string(kMotorCommissioningAcknowledgement);
  configuration.motor_id = 3U;
  configuration.target_rps = -0.1F;
  configuration.duration_ms = 500U;
  configuration.start_time_ms = 0;
  return configuration;
}

CommissioningHeartbeatObservation HeartbeatAt(std::int64_t time_ms) {
  CommissioningHeartbeatObservation observation;
  observation.arrival_time_ms = time_ms;
  observation.sequence = static_cast<std::uint32_t>(time_ms / 50);
  observation.uptime_ms = static_cast<std::uint32_t>(1000 + time_ms);
  observation.agent_session_id = kSessionId;
  observation.state = 1U;
  return observation;
}

CommissioningDiagnosticsObservation DiagnosticsAt(std::int64_t time_ms) {
  CommissioningDiagnosticsObservation observation;
  observation.arrival_time_ms = time_ms;
  observation.uptime_ms = static_cast<std::uint32_t>(1000 + time_ms);
  observation.session_generation = kSessionId;
  observation.session_state = 3U;
  return observation;
}

CommissioningMotorStateObservation StateAt(std::int64_t time_ms,
                                           const std::array<float, 4>& targets,
                                           std::int64_t selected_encoder = 0,
                                           float selected_measured = 0.0F) {
  CommissioningMotorStateObservation observation;
  observation.arrival_time_ms = time_ms;
  observation.target_rps = targets;
  observation.measured_rps[2] = selected_measured;
  observation.encoder_count[2] = selected_encoder;
  return observation;
}

void ObserveHealthy(MotorCommissioningCore* core, std::int64_t time_ms,
                    const std::array<float, 4>& targets = {},
                    std::int64_t selected_encoder = 0,
                    float selected_measured = 0.0F) {
  g_observation_time_ms = time_ms;
  core->ObserveMotionAuthorizationPublisher(true);
  CommissioningMotionAuthorizationObservation authorization;
  authorization.arrival_time_ms = time_ms;
  authorization.configuration_generation = kConfigurationGeneration;
  authorization.agent_session_id = kSessionId;
  core->ObserveMotionAuthorization(authorization);
  core->ObserveHeartbeat(HeartbeatAt(time_ms));
  core->ObserveDiagnostics(DiagnosticsAt(time_ms));
  core->ObserveMotorState(
      StateAt(time_ms, targets, selected_encoder, selected_measured));
  core->ObserveCommandPublisherConflict(false);
}

void Record(MotorCommissioningCore* core,
            const MotorCommissioningAction& action) {
  core->RecordCommandPublished(action.command, g_observation_time_ms);
}

void EnterDrive(MotorCommissioningCore* core) {
  ObserveHealthy(core, 0);
  Record(core, core->Tick(0));
  for (std::int64_t time_ms = 50; time_ms <= 450; time_ms += 50) {
    ObserveHealthy(core, time_ms);
    Record(core, core->Tick(time_ms));
  }
  ObserveHealthy(core, 500);
  const MotorCommissioningAction action = core->Tick(500);
  Expect(action.command == MotorCommissioningCommand::kDrive,
         "pre-stop advances to drive after ten confirmed zeros");
  Record(core, action);
  Expect(core->phase() == MotorCommissioningPhase::kDrive,
         "core is in drive phase");
}

void FinishPostStopWithZero(MotorCommissioningCore* core,
                            std::int64_t post_start_ms,
                            std::int64_t selected_encoder = 0) {
  for (std::int64_t offset_ms = 50; offset_ms <= 450; offset_ms += 50) {
    ObserveHealthy(core, post_start_ms + offset_ms, {}, selected_encoder);
    Record(core, core->Tick(post_start_ms + offset_ms));
  }
  ObserveHealthy(core, post_start_ms + 500, {}, selected_encoder);
  Record(core, core->Tick(post_start_ms + 500));
}

void TestConfigurationValidation() {
  MotorCommissioningConfiguration configuration = ValidConfiguration();
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kNone,
         "valid configuration is accepted");

  configuration.acknowledgement = "MOTORS_RAISED";
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidAcknowledgement,
         "acknowledgement must match exactly");
  configuration = ValidConfiguration();
  configuration.motor_id = 0U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidMotorId,
         "motor zero is rejected");
  configuration.motor_id = 5U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidMotorId,
         "motor five is rejected");

  configuration = ValidConfiguration();
  configuration.target_rps = 0.0F;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "zero target is rejected");
  configuration.target_rps = 0.0099F;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "target below the measurable commissioning minimum is rejected");
  configuration.target_rps = 0.2501F;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "target above positive limit is rejected");
  configuration.target_rps = -0.2501F;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "target below negative limit is rejected");
  configuration.target_rps = std::numeric_limits<float>::quiet_NaN();
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "NaN target is rejected");
  configuration.target_rps = std::numeric_limits<float>::infinity();
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidTarget,
         "infinite target is rejected");

  configuration = ValidConfiguration();
  configuration.duration_ms = 99U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidDuration,
         "duration below 100 ms is rejected");
  configuration.duration_ms = 5001U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kInvalidDuration,
         "duration above 5000 ms is rejected");

  configuration = ValidConfiguration();
  configuration.motor_id = 1U;
  configuration.target_rps = 0.01F;
  configuration.duration_ms = 100U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kNone,
         "lower duration and positive target boundary are accepted");
  configuration.motor_id = 4U;
  configuration.target_rps = -0.25F;
  configuration.duration_ms = 5000U;
  Expect(ValidateMotorCommissioningConfiguration(configuration) ==
             MotorCommissioningFailure::kNone,
         "upper duration and negative target boundary are accepted");
}

void TestFailureNames() {
  struct TestCase {
    MotorCommissioningFailure failure;
    const char* name;
  };
  const std::array<TestCase, 30> cases{{
      {MotorCommissioningFailure::kNone, "NONE"},
      {MotorCommissioningFailure::kInvalidAcknowledgement,
       "INVALID_ACKNOWLEDGEMENT"},
      {MotorCommissioningFailure::kInvalidMotorId, "INVALID_MOTOR_ID"},
      {MotorCommissioningFailure::kInvalidTarget, "INVALID_TARGET"},
      {MotorCommissioningFailure::kInvalidDuration, "INVALID_DURATION"},
      {MotorCommissioningFailure::kPrerequisiteTimeout, "PREREQUISITE_TIMEOUT"},
      {MotorCommissioningFailure::kMotionAuthorizationLost,
       "MOTION_AUTHORIZATION_LOST"},
      {MotorCommissioningFailure::kHeartbeatLost, "HEARTBEAT_LOST"},
      {MotorCommissioningFailure::kHeartbeatNotReady, "HEARTBEAT_NOT_READY"},
      {MotorCommissioningFailure::kSessionInvalid, "SESSION_INVALID"},
      {MotorCommissioningFailure::kSessionChanged, "SESSION_CHANGED"},
      {MotorCommissioningFailure::kMcuResetDetected, "MCU_RESET_DETECTED"},
      {MotorCommissioningFailure::kDiagnosticsLost, "DIAGNOSTICS_LOST"},
      {MotorCommissioningFailure::kDiagnosticsNotActive,
       "DIAGNOSTICS_NOT_ACTIVE"},
      {MotorCommissioningFailure::kDiagnosticsSessionMismatch,
       "DIAGNOSTICS_SESSION_MISMATCH"},
      {MotorCommissioningFailure::kMotorStateLost, "MOTOR_STATE_LOST"},
      {MotorCommissioningFailure::kMotorStateInvalid, "MOTOR_STATE_INVALID"},
      {MotorCommissioningFailure::kMotorStateUnexpectedTarget,
       "MOTOR_STATE_UNEXPECTED_TARGET"},
      {MotorCommissioningFailure::kMotorWatchdogTriggered,
       "MOTOR_WATCHDOG_TRIGGERED"},
      {MotorCommissioningFailure::kDriveCadenceLost, "DRIVE_CADENCE_LOST"},
      {MotorCommissioningFailure::kMotorOverspeed, "MOTOR_OVERSPEED"},
      {MotorCommissioningFailure::kMotorWrongDirection,
       "MOTOR_WRONG_DIRECTION"},
      {MotorCommissioningFailure::kUnselectedMotorMotion,
       "UNSELECTED_MOTOR_MOTION"},
      {MotorCommissioningFailure::kCommandPublisherConflict,
       "COMMAND_PUBLISHER_CONFLICT"},
      {MotorCommissioningFailure::kCommandRejected,
       "COMMAND_REJECTED_OR_LOCKED_IMAGE"},
      {MotorCommissioningFailure::kPreStopNotConfirmed,
       "PRE_STOP_NOT_CONFIRMED"},
      {MotorCommissioningFailure::kTargetNotObserved,
       "TARGET_NOT_OBSERVED_OR_LOCKED_IMAGE"},
      {MotorCommissioningFailure::kPhysicalResponseNotObserved,
       "PHYSICAL_RESPONSE_NOT_OBSERVED"},
      {MotorCommissioningFailure::kPostStopNotConfirmed,
       "POST_STOP_NOT_CONFIRMED"},
      {MotorCommissioningFailure::kInterrupted, "INTERRUPTED"},
  }};

  for (const TestCase& test_case : cases) {
    Expect(std::string(MotorCommissioningFailureName(test_case.failure)) ==
               test_case.name,
           std::string("failure name is stable for ") + test_case.name);
  }
  Expect(std::string(MotorCommissioningFailureName(
             static_cast<MotorCommissioningFailure>(255U))) == "UNKNOWN",
         "unknown failure values have a stable fallback name");
}

void TestPassingRun() {
  MotorCommissioningCore core(ValidConfiguration());
  EnterDrive(&core);
  const std::array<float, 4> drive_targets{0.0F, 0.0F, -0.1F, 0.0F};
  for (std::int64_t time_ms = 550; time_ms <= 950; time_ms += 50) {
    const std::int64_t encoder = -(time_ms - 500) / 10;
    ObserveHealthy(&core, time_ms, drive_targets, encoder, -0.08F);
    Record(&core, core.Tick(time_ms));
  }
  ObserveHealthy(&core, 1000, drive_targets, -50, -0.05F);
  const MotorCommissioningAction first_stop = core.Tick(1000);
  Expect(first_stop.command == MotorCommissioningCommand::kStop,
         "drive duration ends with an immediate stop command");
  Record(&core, first_stop);
  FinishPostStopWithZero(&core, 1000, -50);

  const auto summary = core.summary();
  Expect(core.complete(), "passing run completes");
  Expect(summary.passed, "passing run reports success");
  Expect(summary.failure == MotorCommissioningFailure::kNone,
         "passing run has no failure");
  Expect(summary.agent_session_id == kSessionId,
         "passing run records the locked session");
  Expect(summary.target_observed, "selected target was observed");
  Expect(summary.physical_response_observed,
         "selected physical response was observed");
  Expect(summary.zero_confirmed, "post-stop zero was observed");
  Expect(summary.pre_stop_commands == 10U,
         "pre-stop sends exactly ten commands");
  Expect(summary.drive_commands == 10U,
         "500 ms drive sends exactly ten commands at 20 Hz");
  Expect(summary.post_stop_commands == 10U,
         "post-stop sends exactly ten commands");
  Expect(summary.encoder_delta == -50,
         "encoder delta spans the commissioned motion");
  Expect(std::fabs(summary.peak_absolute_measured_rps - 0.08F) < 0.0001F,
         "peak measured response is recorded");
}

void TestLockedImageAndRejectionFailures() {
  MotorCommissioningCore locked_core(ValidConfiguration());
  EnterDrive(&locked_core);
  for (std::int64_t time_ms = 550; time_ms <= 1000; time_ms += 50) {
    ObserveHealthy(&locked_core, time_ms);
    Record(&locked_core, locked_core.Tick(time_ms));
  }
  FinishPostStopWithZero(&locked_core, 1000);
  const auto locked_summary = locked_core.summary();
  Expect(!locked_summary.passed, "locked image cannot pass");
  Expect(
      locked_summary.failure == MotorCommissioningFailure::kTargetNotObserved,
      "unobserved target identifies a locked or non-admitting image");

  MotorCommissioningCore rejected_core(ValidConfiguration());
  EnterDrive(&rejected_core);
  ObserveHealthy(&rejected_core, 550);
  auto diagnostics = DiagnosticsAt(550);
  diagnostics.command_rejections = 1U;
  rejected_core.ObserveDiagnostics(diagnostics);
  rejected_core.ObserveMotorState(StateAt(550, {}));
  const auto action = rejected_core.Tick(550);
  Expect(action.command == MotorCommissioningCommand::kStop,
         "diagnostic rejection immediately changes to stop");
  Record(&rejected_core, action);
  Expect(rejected_core.summary().failure ==
             MotorCommissioningFailure::kCommandRejected,
         "diagnostic rejection identifies command rejection or locked image");
}

enum class AbortMutation : std::uint8_t {
  kGate,
  kHeartbeatStale,
  kHeartbeatState,
  kSessionZero,
  kSessionChange,
  kDiagnosticsStale,
  kDiagnosticsState,
  kDiagnosticsSession,
  kMotorStateStale,
  kMotorStateInvalid,
  kMotorStateTarget,
  kPublisherConflict,
};

void ApplyAbortMutation(MotorCommissioningCore* core, AbortMutation mutation,
                        std::int64_t time_ms) {
  ObserveHealthy(core, time_ms);
  switch (mutation) {
    case AbortMutation::kGate:
      core->ObserveMotionAuthorizationPublisher(false);
      break;
    case AbortMutation::kHeartbeatStale: {
      auto heartbeat = HeartbeatAt(time_ms);
      heartbeat.arrival_time_ms = time_ms - 1001;
      core->ObserveHeartbeat(heartbeat);
      break;
    }
    case AbortMutation::kHeartbeatState: {
      auto heartbeat = HeartbeatAt(time_ms);
      heartbeat.state = 3U;
      core->ObserveHeartbeat(heartbeat);
      break;
    }
    case AbortMutation::kSessionZero: {
      auto heartbeat = HeartbeatAt(time_ms);
      heartbeat.agent_session_id = 0U;
      core->ObserveHeartbeat(heartbeat);
      break;
    }
    case AbortMutation::kSessionChange: {
      auto heartbeat = HeartbeatAt(time_ms);
      heartbeat.agent_session_id = kSessionId + 1U;
      core->ObserveHeartbeat(heartbeat);
      break;
    }
    case AbortMutation::kDiagnosticsStale: {
      auto diagnostics = DiagnosticsAt(time_ms);
      diagnostics.arrival_time_ms = time_ms - 1501;
      core->ObserveDiagnostics(diagnostics);
      break;
    }
    case AbortMutation::kDiagnosticsState: {
      auto diagnostics = DiagnosticsAt(time_ms);
      diagnostics.session_state = 4U;
      core->ObserveDiagnostics(diagnostics);
      break;
    }
    case AbortMutation::kDiagnosticsSession: {
      auto diagnostics = DiagnosticsAt(time_ms);
      diagnostics.session_generation = kSessionId + 1U;
      core->ObserveDiagnostics(diagnostics);
      break;
    }
    case AbortMutation::kMotorStateStale:
      core->ObserveMotorState(StateAt(time_ms - 201, {}));
      break;
    case AbortMutation::kMotorStateInvalid: {
      auto state = StateAt(time_ms, {});
      state.measured_rps[0] = std::numeric_limits<float>::quiet_NaN();
      core->ObserveMotorState(state);
      break;
    }
    case AbortMutation::kMotorStateTarget: {
      const std::array<float, 4> targets{0.1F, 0.0F, 0.0F, 0.0F};
      core->ObserveMotorState(StateAt(time_ms, targets));
      break;
    }
    case AbortMutation::kPublisherConflict:
      core->ObserveCommandPublisherConflict(true);
      break;
  }
}

void TestRuntimeAbortCategories() {
  struct TestCase {
    AbortMutation mutation;
    MotorCommissioningFailure failure;
    const char* description;
  };
  const std::array<TestCase, 12> cases{{
      {AbortMutation::kGate,
       MotorCommissioningFailure::kMotionAuthorizationLost,
       "motion authorization loss"},
      {AbortMutation::kHeartbeatStale,
       MotorCommissioningFailure::kHeartbeatLost, "stale heartbeat"},
      {AbortMutation::kHeartbeatState,
       MotorCommissioningFailure::kHeartbeatNotReady, "non-ready heartbeat"},
      {AbortMutation::kSessionZero, MotorCommissioningFailure::kSessionInvalid,
       "zero session"},
      {AbortMutation::kSessionChange,
       MotorCommissioningFailure::kSessionChanged, "changed session"},
      {AbortMutation::kDiagnosticsStale,
       MotorCommissioningFailure::kDiagnosticsLost, "stale diagnostics"},
      {AbortMutation::kDiagnosticsState,
       MotorCommissioningFailure::kDiagnosticsNotActive,
       "non-active diagnostics"},
      {AbortMutation::kDiagnosticsSession,
       MotorCommissioningFailure::kDiagnosticsSessionMismatch,
       "diagnostics session mismatch"},
      {AbortMutation::kMotorStateStale,
       MotorCommissioningFailure::kMotorStateLost, "stale motor state"},
      {AbortMutation::kMotorStateInvalid,
       MotorCommissioningFailure::kMotorStateInvalid, "invalid motor state"},
      {AbortMutation::kMotorStateTarget,
       MotorCommissioningFailure::kMotorStateUnexpectedTarget,
       "unexpected motor target"},
      {AbortMutation::kPublisherConflict,
       MotorCommissioningFailure::kCommandPublisherConflict,
       "command publisher conflict"},
  }};

  for (const TestCase& test_case : cases) {
    auto core = std::make_unique<MotorCommissioningCore>(ValidConfiguration());
    EnterDrive(core.get());
    ApplyAbortMutation(core.get(), test_case.mutation, 550);
    const auto action = core->Tick(550);
    Expect(action.command == MotorCommissioningCommand::kStop,
           std::string(test_case.description) + " immediately emits stop");
    Expect(core->phase() == MotorCommissioningPhase::kPostStop,
           std::string(test_case.description) + " enters post-stop");
    Expect(core->summary().failure == test_case.failure,
           std::string(test_case.description) + " preserves failure cause");
  }
}

void TestInterruptAndPostStopConfirmation() {
  MotorCommissioningCore interrupted_core(ValidConfiguration());
  EnterDrive(&interrupted_core);
  interrupted_core.RequestAbort(550);
  auto action = interrupted_core.Tick(550);
  Record(&interrupted_core, action);
  interrupted_core.RequestAbort(700);
  for (std::int64_t time_ms = 600; time_ms <= 1050; time_ms += 50) {
    ObserveHealthy(&interrupted_core, time_ms);
    action = interrupted_core.Tick(time_ms);
    Record(&interrupted_core, action);
  }
  const auto interrupted_summary = interrupted_core.summary();
  Expect(interrupted_core.complete(), "interrupted run completes after stop");
  Expect(interrupted_summary.failure == MotorCommissioningFailure::kInterrupted,
         "interrupt is the reported outcome");
  Expect(interrupted_summary.post_stop_commands == 10U,
         "repeated interrupt does not extend the bounded stop burst");

  MotorCommissioningCore no_zero_core(ValidConfiguration());
  EnterDrive(&no_zero_core);
  const std::array<float, 4> drive_targets{0.0F, 0.0F, -0.1F, 0.0F};
  for (std::int64_t time_ms = 550; time_ms <= 1000; time_ms += 50) {
    ObserveHealthy(&no_zero_core, time_ms, drive_targets, -10, -0.05F);
    Record(&no_zero_core, no_zero_core.Tick(time_ms));
  }
  for (std::int64_t time_ms = 1050; time_ms <= 1500; time_ms += 50) {
    ObserveHealthy(&no_zero_core, time_ms, drive_targets, -10, -0.01F);
    Record(&no_zero_core, no_zero_core.Tick(time_ms));
  }
  const auto no_zero_summary = no_zero_core.summary();
  Expect(no_zero_core.complete(), "missing zero confirmation still terminates");
  Expect(!no_zero_summary.passed, "missing zero confirmation fails");
  Expect(no_zero_summary.failure ==
             MotorCommissioningFailure::kPostStopNotConfirmed,
         "missing zero confirmation has an explicit failure");
}

void TestCadenceAndWatchdogAbortWithoutRearm() {
  MotorCommissioningConfiguration long_configuration = ValidConfiguration();
  long_configuration.duration_ms = 5000U;
  MotorCommissioningCore cadence_core(long_configuration);
  EnterDrive(&cadence_core);
  ObserveHealthy(&cadence_core, 650);
  const auto cadence_action = cadence_core.Tick(650);
  Expect(cadence_action.command == MotorCommissioningCommand::kStop,
         "a drive publication gap above 100 ms emits stop instead of rearming");
  Expect(cadence_core.summary().failure ==
             MotorCommissioningFailure::kDriveCadenceLost,
         "a host stall is reported as lost drive cadence");

  MotorCommissioningCore state_watchdog_core(ValidConfiguration());
  EnterDrive(&state_watchdog_core);
  ObserveHealthy(&state_watchdog_core, 550);
  auto watchdog_state = StateAt(550, {});
  watchdog_state.watchdog_stop_mask = std::uint8_t{0x04};
  state_watchdog_core.ObserveMotorState(watchdog_state);
  const auto state_watchdog_action = state_watchdog_core.Tick(550);
  Expect(state_watchdog_action.command == MotorCommissioningCommand::kStop,
         "a reported MCU lease stop immediately emits stop");
  Expect(state_watchdog_core.summary().failure ==
             MotorCommissioningFailure::kMotorWatchdogTriggered,
         "the motor-state watchdog mask prevents rearm");

  MotorCommissioningCore counter_watchdog_core(ValidConfiguration());
  EnterDrive(&counter_watchdog_core);
  ObserveHealthy(&counter_watchdog_core, 550);
  auto diagnostics = DiagnosticsAt(550);
  diagnostics.motor_lease_expiries[2] = 1U;
  diagnostics.motor_watchdog_trips = 1U;
  counter_watchdog_core.ObserveDiagnostics(diagnostics);
  const auto counter_watchdog_action = counter_watchdog_core.Tick(550);
  Expect(counter_watchdog_action.command == MotorCommissioningCommand::kStop,
         "a lease counter increment immediately emits stop");
  Expect(counter_watchdog_core.summary().failure ==
             MotorCommissioningFailure::kMotorWatchdogTriggered,
         "lease and watchdog counters are locked at prerequisites");
}

void TestAuthorizationAndResetContinuity() {
  MotorCommissioningCore authorization_core(ValidConfiguration());
  EnterDrive(&authorization_core);
  ObserveHealthy(&authorization_core, 550);
  CommissioningMotionAuthorizationObservation changed_authorization;
  changed_authorization.arrival_time_ms = 550;
  changed_authorization.configuration_generation =
      kConfigurationGeneration + 1U;
  changed_authorization.agent_session_id = kSessionId;
  authorization_core.ObserveMotionAuthorization(changed_authorization);
  const auto authorization_action = authorization_core.Tick(550);
  Expect(authorization_action.command == MotorCommissioningCommand::kStop,
         "a host configuration-generation change immediately emits stop");
  Expect(authorization_core.summary().failure ==
             MotorCommissioningFailure::kMotionAuthorizationLost,
         "authorization is bound to the locked host generation and session");

  MotorCommissioningCore heartbeat_reset_core(ValidConfiguration());
  EnterDrive(&heartbeat_reset_core);
  ObserveHealthy(&heartbeat_reset_core, 550);
  auto reset_heartbeat = HeartbeatAt(550);
  reset_heartbeat.sequence = 0U;
  reset_heartbeat.uptime_ms = 100U;
  heartbeat_reset_core.ObserveHeartbeat(reset_heartbeat);
  const auto heartbeat_reset_action = heartbeat_reset_core.Tick(550);
  Expect(heartbeat_reset_action.command == MotorCommissioningCommand::kStop,
         "same-ID heartbeat reset immediately emits stop");
  Expect(heartbeat_reset_core.summary().failure ==
             MotorCommissioningFailure::kMcuResetDetected,
         "same-ID heartbeat uptime/sequence regression identifies MCU reset");

  MotorCommissioningCore diagnostics_reset_core(ValidConfiguration());
  EnterDrive(&diagnostics_reset_core);
  ObserveHealthy(&diagnostics_reset_core, 550);
  auto reset_diagnostics = DiagnosticsAt(550);
  reset_diagnostics.uptime_ms = 100U;
  diagnostics_reset_core.ObserveDiagnostics(reset_diagnostics);
  const auto diagnostics_reset_action = diagnostics_reset_core.Tick(550);
  Expect(diagnostics_reset_action.command == MotorCommissioningCommand::kStop,
         "same-ID diagnostics reset immediately emits stop");
  Expect(diagnostics_reset_core.summary().failure ==
             MotorCommissioningFailure::kMcuResetDetected,
         "diagnostics uptime regression independently identifies MCU reset");
}

void TestPhysicalMotionGuardsAndResponseRequirement() {
  const std::array<float, 4> drive_targets{0.0F, 0.0F, -0.1F, 0.0F};

  MotorCommissioningCore overspeed_core(ValidConfiguration());
  EnterDrive(&overspeed_core);
  ObserveHealthy(&overspeed_core, 550, drive_targets, -5, -3.1F);
  Expect(overspeed_core.Tick(550).command == MotorCommissioningCommand::kStop,
         "selected-motor overspeed immediately emits stop");
  Expect(overspeed_core.summary().failure ==
             MotorCommissioningFailure::kMotorOverspeed,
         "selected-motor overspeed is explicit");

  MotorCommissioningCore one_tick_noise_core(ValidConfiguration());
  EnterDrive(&one_tick_noise_core);
  ObserveHealthy(&one_tick_noise_core, 550, drive_targets, 1, 0.087F);
  Expect(one_tick_noise_core.Tick(550).command ==
             MotorCommissioningCommand::kDrive,
         "one opposite tick does not classify motor direction");
  Expect(
      one_tick_noise_core.summary().failure == MotorCommissioningFailure::kNone,
      "one opposite tick remains an inconclusive sample");

  MotorCommissioningCore direction_core(ValidConfiguration());
  EnterDrive(&direction_core);
  ObserveHealthy(&direction_core, 550, drive_targets, 5, 0.05F);
  Expect(direction_core.Tick(550).command == MotorCommissioningCommand::kStop,
         "opposite measured and encoder direction immediately emits stop");
  Expect(direction_core.summary().failure ==
             MotorCommissioningFailure::kMotorWrongDirection,
         "wrong selected-motor direction is explicit");

  MotorCommissioningCore unselected_core(ValidConfiguration());
  EnterDrive(&unselected_core);
  ObserveHealthy(&unselected_core, 550, drive_targets, -5, -0.05F);
  auto unselected_state = StateAt(550, drive_targets, -5, -0.05F);
  unselected_state.measured_rps[0] = 0.03F;
  unselected_state.encoder_count[0] = 3;
  unselected_core.ObserveMotorState(unselected_state);
  Expect(unselected_core.Tick(550).command == MotorCommissioningCommand::kStop,
         "motion on an unselected channel immediately emits stop");
  Expect(unselected_core.summary().failure ==
             MotorCommissioningFailure::kUnselectedMotorMotion,
         "wrong-channel motion is explicit");

  MotorCommissioningCore no_response_core(ValidConfiguration());
  EnterDrive(&no_response_core);
  for (std::int64_t time_ms = 550; time_ms <= 950; time_ms += 50) {
    ObserveHealthy(&no_response_core, time_ms, drive_targets);
    Record(&no_response_core, no_response_core.Tick(time_ms));
  }
  ObserveHealthy(&no_response_core, 1000, drive_targets);
  Record(&no_response_core, no_response_core.Tick(1000));
  FinishPostStopWithZero(&no_response_core, 1000);
  Expect(!no_response_core.summary().passed,
         "setpoint admission without encoder/measured response cannot pass");
  Expect(no_response_core.summary().failure ==
             MotorCommissioningFailure::kPhysicalResponseNotObserved,
         "missing physical response has an explicit failure");
}

void TestPostStopZeroMustRemainLatest() {
  MotorCommissioningCore core(ValidConfiguration());
  EnterDrive(&core);
  const std::array<float, 4> drive_targets{0.0F, 0.0F, -0.1F, 0.0F};
  for (std::int64_t time_ms = 550; time_ms <= 950; time_ms += 50) {
    ObserveHealthy(&core, time_ms, drive_targets, -10, -0.05F);
    Record(&core, core.Tick(time_ms));
  }
  ObserveHealthy(&core, 1000, drive_targets, -10, -0.05F);
  Record(&core, core.Tick(1000));
  ObserveHealthy(&core, 1050, {}, -10);
  Record(&core, core.Tick(1050));
  ObserveHealthy(&core, 1100, drive_targets, -10, -0.01F);
  const auto regressed_action = core.Tick(1100);
  Expect(regressed_action.command == MotorCommissioningCommand::kStop,
         "a target returning after zero keeps emitting stop");
  Expect(core.summary().failure ==
             MotorCommissioningFailure::kMotorStateUnexpectedTarget,
         "post-stop zero confirmation cannot be satisfied by an older sample");
  Expect(!core.summary().zero_confirmed,
         "latest nonzero state clears post-stop zero confirmation");
}

void TestPostStopRequiresStationaryMotor() {
  MotorCommissioningCore core(ValidConfiguration());
  EnterDrive(&core);
  const std::array<float, 4> drive_targets{0.0F, 0.0F, -0.1F, 0.0F};
  for (std::int64_t time_ms = 550; time_ms <= 950; time_ms += 50) {
    ObserveHealthy(&core, time_ms, drive_targets, -10, -0.05F);
    Record(&core, core.Tick(time_ms));
  }
  ObserveHealthy(&core, 1000, drive_targets, -10, -0.05F);
  Record(&core, core.Tick(1000));
  for (std::int64_t time_ms = 1050; time_ms <= 1500; time_ms += 50) {
    ObserveHealthy(&core, time_ms, {}, -10, -0.01F);
    Record(&core, core.Tick(time_ms));
  }

  const auto summary = core.summary();
  Expect(core.complete(),
         "moving post-stop run terminates after the stop burst");
  Expect(!summary.passed,
         "zero targets cannot pass while measured motor motion remains");
  Expect(summary.failure == MotorCommissioningFailure::kPostStopNotConfirmed,
         "post-stop motion has an explicit stop-confirmation failure");
  Expect(!summary.zero_confirmed,
         "zero confirmation includes the final stationary measurement");
}

void TestPrerequisiteRefusal() {
  MotorCommissioningCore conflict_core(ValidConfiguration());
  conflict_core.ObserveCommandPublisherConflict(true);
  const auto conflict_action = conflict_core.Tick(0);
  Expect(conflict_action.command == MotorCommissioningCommand::kStop,
         "publisher conflict refuses motion and emits stop");
  Expect(conflict_core.summary().failure ==
             MotorCommissioningFailure::kCommandPublisherConflict,
         "publisher conflict is explicit before prerequisites");

  MotorCommissioningCore timeout_core(ValidConfiguration());
  const auto timeout_action = timeout_core.Tick(5000);
  Expect(timeout_action.command == MotorCommissioningCommand::kStop,
         "prerequisite timeout starts a stop burst");
  Expect(timeout_core.summary().failure ==
             MotorCommissioningFailure::kPrerequisiteTimeout,
         "prerequisite timeout is explicit");
}

}  // namespace

int main() {
  TestConfigurationValidation();
  TestFailureNames();
  TestPassingRun();
  TestLockedImageAndRejectionFailures();
  TestRuntimeAbortCategories();
  TestInterruptAndPostStopConfirmation();
  TestCadenceAndWatchdogAbortWithoutRearm();
  TestAuthorizationAndResetContinuity();
  TestPhysicalMotionGuardsAndResponseRequirement();
  TestPostStopZeroMustRemainLatest();
  TestPostStopRequiresStationaryMotor();
  TestPrerequisiteRefusal();
  if (g_failures == 0) {
    std::cout << "motor commissioning core tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
