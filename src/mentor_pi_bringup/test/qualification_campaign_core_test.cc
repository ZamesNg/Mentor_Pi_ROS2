// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_campaign_core.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {

using mentor_pi_bringup::CampaignCommand;
using mentor_pi_bringup::CampaignConfiguration;
using mentor_pi_bringup::CampaignDiagnosticsObservation;
using mentor_pi_bringup::CampaignEvidenceMetadata;
using mentor_pi_bringup::CampaignFailure;
using mentor_pi_bringup::CampaignFailureBit;
using mentor_pi_bringup::CampaignHeartbeatObservation;
using mentor_pi_bringup::CampaignMetricStatus;
using mentor_pi_bringup::CampaignMode;
using mentor_pi_bringup::CampaignProfileForMode;
using mentor_pi_bringup::CampaignScheduler;
using mentor_pi_bringup::CampaignService;
using mentor_pi_bringup::CampaignSummary;
using mentor_pi_bringup::CampaignTelemetry;
using mentor_pi_bringup::QualificationCampaignCore;
using mentor_pi_bringup::ScheduledSlotCount;
using mentor_pi_bringup::WriteCampaignEvidence;

constexpr std::int64_t kMillisecond = INT64_C(1000000);
constexpr std::int64_t kSecond = INT64_C(1000000000);
constexpr std::uint32_t kSessionId = 42U;
constexpr std::uint8_t kResetPowerOn = 0U;
constexpr std::uint8_t kResetPin = 1U;
constexpr std::uint8_t kResetSoftware = 2U;
constexpr std::uint8_t kResetIndependentWatchdog = 3U;
constexpr std::uint8_t kResetWindowWatchdog = 4U;
constexpr std::uint8_t kResetBrownout = 5U;
constexpr std::uint8_t kResetLowPower = 6U;
constexpr std::uint8_t kResetUnknown = 255U;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

bool HasFailure(const CampaignSummary& summary, CampaignFailure failure) {
  return (summary.failure_mask & CampaignFailureBit(failure)) != 0U;
}

CampaignHeartbeatObservation HeartbeatAt(std::int64_t time_ns,
                                         std::uint32_t session_id,
                                         std::uint32_t sequence,
                                         std::uint32_t uptime_ms) {
  CampaignHeartbeatObservation heartbeat;
  heartbeat.arrival_time_ns = time_ns;
  heartbeat.session_id = session_id;
  heartbeat.sequence = sequence;
  heartbeat.uptime_ms = uptime_ms;
  heartbeat.state = 1U;
  return heartbeat;
}

CampaignDiagnosticsObservation DiagnosticsAt(
    std::int64_t time_ns, std::uint32_t session_id, std::uint32_t uptime_ms,
    std::uint64_t wire_bytes, std::uint32_t consumptions,
    std::uint32_t age_over_20_ms, std::uint32_t maximum_age_us) {
  CampaignDiagnosticsObservation diagnostics;
  diagnostics.arrival_time_ns = time_ns;
  diagnostics.session_generation = session_id;
  diagnostics.uptime_ms = uptime_ms;
  diagnostics.transport_rx_bytes = wire_bytes;
  diagnostics.motor_command_consumptions = consumptions;
  diagnostics.motor_command_age_over_20_ms = age_over_20_ms;
  diagnostics.motor_command_max_age_us = maximum_age_us;
  diagnostics.session_state = 3U;
  return diagnostics;
}

void ObserveTwoConnectedSamples(QualificationCampaignCore* core,
                                CampaignTelemetry stream,
                                std::int64_t start_time_ns,
                                std::int64_t period_ns) {
  core->ObserveTelemetry(stream, start_time_ns, true);
  core->ObserveTelemetry(stream, start_time_ns + period_ns, true);
}

void ObserveRequiredConnectedSamples(QualificationCampaignCore* core,
                                     std::int64_t start_time_ns) {
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kHeartbeat, start_time_ns,
                             500 * kMillisecond);
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kDiagnostics,
                             start_time_ns, kSecond);
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kMotorState,
                             start_time_ns, 20 * kMillisecond);
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kPwmServoState,
                             start_time_ns, 50 * kMillisecond);
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kImu, start_time_ns,
                             20 * kMillisecond);
  ObserveTwoConnectedSamples(core, CampaignTelemetry::kBattery, start_time_ns,
                             kSecond);
}

void ObserveRequiredConnectedWindow(QualificationCampaignCore* core,
                                    std::int64_t start_time_ns,
                                    std::int64_t duration_ns) {
  for (std::int64_t elapsed_ns = 0; elapsed_ns <= duration_ns;
       elapsed_ns += kMillisecond) {
    const std::int64_t now_ns = start_time_ns + elapsed_ns;
    if (elapsed_ns % (500 * kMillisecond) == 0) {
      core->ObserveTelemetry(CampaignTelemetry::kHeartbeat, now_ns, true);
    }
    if (elapsed_ns % kSecond == 0) {
      core->ObserveTelemetry(CampaignTelemetry::kDiagnostics, now_ns, true);
      core->ObserveTelemetry(CampaignTelemetry::kBattery, now_ns, true);
    }
    if (elapsed_ns % (20 * kMillisecond) == 0) {
      core->ObserveTelemetry(CampaignTelemetry::kMotorState, now_ns, true);
      core->ObserveTelemetry(CampaignTelemetry::kImu, now_ns, true);
    }
    if (elapsed_ns % (50 * kMillisecond) == 0) {
      core->ObserveTelemetry(CampaignTelemetry::kPwmServoState, now_ns, true);
    }
  }
}

void TestCanonicalProfiles() {
  const auto load = CampaignProfileForMode(CampaignMode::kLoad500);
  Expect(ScheduledSlotCount(load.canonical_duration_ns,
                            load.command_rates[0U]) == UINT64_C(1800000),
         "load profile schedules exactly 1,800,000 motor commands");
  Expect(ScheduledSlotCount(load.canonical_duration_ns,
                            load.command_rates[4U]) == 360U,
         "load profile schedules one buzzer command per ten seconds");
  Expect(ScheduledSlotCount(load.canonical_duration_ns,
                            load.command_rates[6U]) == 720U,
         "load profile schedules OLED at 0.2 Hz");

  const auto soak = CampaignProfileForMode(CampaignMode::kSoak);
  Expect(ScheduledSlotCount(soak.canonical_duration_ns,
                            soak.command_rates[0U]) == UINT64_C(4320000),
         "soak profile schedules motor commands at 50 Hz for 24 hours");
  Expect(ScheduledSlotCount(soak.canonical_duration_ns,
                            soak.command_rates[4U]) == 1440U,
         "soak profile schedules exactly one buzzer command per minute");

  CampaignScheduler scheduler(load, 20 * kMillisecond);
  Expect(scheduler.Start(0), "scheduler starts once with a valid clock");
  const auto first = scheduler.Poll(0);
  Expect((first.command_mask & UINT8_C(1)) != 0U,
         "motor has a slot at the evidence-window boundary");
  const auto late = scheduler.Poll(10 * kMillisecond);
  Expect((late.missed_command_mask & UINT8_C(1)) != 0U,
         "scheduler records skipped motor slots instead of bursting backlog");
  Expect(scheduler.skipped_commands()[0U] == 4U,
         "ten-millisecond poll records the four skipped 500 Hz slots");
}

CampaignSummary RunPassingShortCampaign() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kLoad500);
  configuration.duration_ns = 2 * kSecond;
  QualificationCampaignCore core(configuration);
  core.ObserveHeartbeat(HeartbeatAt(0, kSessionId, 0U, 0U));
  core.ObserveDiagnostics(DiagnosticsAt(0, kSessionId, 0U, 0U, 0U, 0U, 0U));
  core.ObserveEndpointGraph(true, 0);
  core.ObserveHeartbeat(HeartbeatAt(kSecond, kSessionId, 1U, 1000U));
  Expect(core.BeginEvidence(kSecond),
         "clean diagnostics and complete graph open the evidence window");

  for (std::int64_t elapsed = 0; elapsed < 2 * kSecond;
       elapsed += kMillisecond) {
    const std::int64_t now = kSecond + elapsed;
    const auto work = core.Poll(now);
    for (std::uint8_t index = 0U;
         index < static_cast<std::uint8_t>(CampaignCommand::kCount); ++index) {
      if ((work.command_mask & (UINT8_C(1) << index)) == 0U) {
        continue;
      }
      if (index == static_cast<std::uint8_t>(CampaignCommand::kMotor)) {
        const std::array<float, 4U> targets{};
        Expect(core.RecordMotorCommand(UINT8_C(0x0F), targets),
               "zero-only motor command is accepted by the campaign core");
      } else {
        core.RecordCommand(static_cast<CampaignCommand>(index));
      }
    }
    if (work.service_round) {
      for (std::uint8_t index = 0U;
           index < static_cast<std::uint8_t>(CampaignService::kCount);
           ++index) {
        core.RecordServiceResult(static_cast<CampaignService>(index), 0U, true,
                                 false);
      }
    }

    if (elapsed % (500 * kMillisecond) == 0) {
      const auto sequence =
          static_cast<std::uint32_t>(elapsed / (500 * kMillisecond)) + 1U;
      const auto uptime = static_cast<std::uint32_t>(now / kMillisecond);
      if (elapsed > 0) {
        core.ObserveHeartbeat(HeartbeatAt(now, kSessionId, sequence, uptime));
      }
      core.ObserveTelemetry(CampaignTelemetry::kHeartbeat, now, true);
    }
    if (elapsed % kSecond == 0) {
      core.ObserveTelemetry(CampaignTelemetry::kDiagnostics, now, true);
      core.ObserveTelemetry(CampaignTelemetry::kBattery, now, true);
    }
    if (elapsed % (20 * kMillisecond) == 0) {
      core.ObserveTelemetry(CampaignTelemetry::kMotorState, now, true);
      core.ObserveTelemetry(CampaignTelemetry::kImu, now, true);
    }
    if (elapsed % (50 * kMillisecond) == 0) {
      core.ObserveTelemetry(CampaignTelemetry::kPwmServoState, now, true);
    }
  }

  core.ObserveDiagnostics(
      DiagnosticsAt(2 * kSecond, kSessionId, 2000U, 1200U, 500U, 2U, 9000U));
  core.ObserveDiagnostics(
      DiagnosticsAt(3 * kSecond, kSessionId, 3000U, 2400U, 1000U, 5U, 10000U));
  return core.Finish(3 * kSecond);
}

QualificationCampaignCore StartedCore(CampaignConfiguration configuration);

void TestPassingShortCampaignAndMotorSafety() {
  const CampaignSummary summary = RunPassingShortCampaign();
  Expect(summary.execution_passed,
         "short noncanonical campaign can pass its observable checks mask=" +
             std::to_string(summary.failure_mask));
  Expect(!summary.release_qualified,
         "campaign core can never claim release qualification");
  Expect(!summary.canonical_profile,
         "duration override is visibly noncanonical");
  Expect(summary.command_publications[0U] == 1000U,
         "two-second test publishes exactly 1,000 motor slots");
  Expect(summary.motor_age_p99 == CampaignMetricStatus::kPass,
         "strict-over-20ms ratio supplies a passing nearest-rank p99 check");
  Expect(summary.independent_wire_capture == CampaignMetricStatus::kNotObserved,
         "independent capture remains explicitly unobserved");

  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kLoad500);
  configuration.duration_ns = kSecond;
  QualificationCampaignCore core(configuration);
  core.ObserveHeartbeat(HeartbeatAt(0, kSessionId, 0U, 0U));
  core.ObserveDiagnostics(DiagnosticsAt(0, kSessionId, 0U, 0U, 0U, 0U, 0U));
  core.ObserveEndpointGraph(true, 0);
  Expect(core.BeginEvidence(0), "motor-safety fixture starts");
  std::array<float, 4U> targets{};
  targets[2U] = 0.001F;
  Expect(!core.RecordMotorCommand(UINT8_C(0x0F), targets),
         "any nonzero target is rejected before publication accounting");
  const CampaignSummary unsafe = core.Finish(kSecond);
  Expect(HasFailure(unsafe, CampaignFailure::kNonzeroMotorAttempt),
         "nonzero attempt is a permanent evidence failure");
}

void TestTelemetryDeadlineFailsBeforeFinish() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kLoad500);
  configuration.duration_ns = kSecond;
  configuration.require_valid_imu = false;
  auto core = StartedCore(configuration);
  core.ObserveTelemetry(CampaignTelemetry::kHeartbeat, kMillisecond, true);
  core.ObserveTelemetry(CampaignTelemetry::kDiagnostics, kMillisecond, true);
  core.ObserveTelemetry(CampaignTelemetry::kMotorState, kMillisecond, true);
  core.ObserveTelemetry(CampaignTelemetry::kPwmServoState, kMillisecond, true);
  core.ObserveTelemetry(CampaignTelemetry::kBattery, kMillisecond, true);

  for (std::int64_t now = 0; now <= 41 * kMillisecond; now += kMillisecond) {
    static_cast<void>(core.Poll(now));
  }
  Expect(!core.failed(),
         "the live telemetry deadline allows exactly two motor periods");
  static_cast<void>(core.Poll(42 * kMillisecond));
  Expect(core.failed(),
         "a missing next telemetry sample latches failure before Finish");
}

void TestReconnectObservationIsBounded() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kReconnectAgent);
  configuration.duration_ns = 0;
  QualificationCampaignCore core(configuration);
  core.ObserveHeartbeat(HeartbeatAt(0, 1U, 1U, 1000U));
  core.ObserveDiagnostics(DiagnosticsAt(0, 1U, 1000U, 0U, 0U, 0U, 0U));
  core.ObserveEndpointGraph(true, 0);
  Expect(core.BeginEvidence(0), "reconnect campaign starts from clean state");
  ObserveRequiredConnectedSamples(&core, kMillisecond);

  std::int64_t now = 0;
  std::uint32_t uptime = 1000U;
  std::uint32_t sequence = 1U;
  for (std::uint32_t cycle = 1U; cycle <= 100U; ++cycle) {
    now += 2 * kSecond;
    core.ObserveEndpointGraph(false, now);
    uptime += 2000U;
    ++sequence;
    core.ObserveHeartbeat(
        HeartbeatAt(now + kMillisecond, cycle + 1U, sequence, uptime));
    core.ObserveEndpointGraph(true, now + (100 * kMillisecond));
    core.ObserveDiagnostics(DiagnosticsAt(now + (101 * kMillisecond),
                                          cycle + 1U, uptime, 0U, 0U, 0U, 0U));
  }
  const std::int64_t final_samples_start = now + (102 * kMillisecond);
  ObserveRequiredConnectedWindow(&core, final_samples_start, kSecond);
  const std::int64_t finish_time = final_samples_start + kSecond;
  Expect(core.CampaignComplete(finish_time),
         "operator campaign completes after 100 observed recovered sessions");
  const CampaignSummary summary = core.Finish(finish_time);
  Expect(summary.observed_cycles == 100U,
         "all 100 operator-driven cycles are counted");
  Expect(summary.stored_transition_count == 100U,
         "fixed transition ledger retains exactly its bounded campaign size");
  Expect(summary.transitions[99U].endpoint_recovered,
         "last observed session has host-graph recovery evidence");
  Expect(!HasFailure(summary, CampaignFailure::kSessionInvalid),
         "Agent restart accepts a fast forward session without an outage");
  Expect(
      summary.physical_endpoint_recovery == CampaignMetricStatus::kNotObserved,
      "host graph recovery is not mislabeled physical recovery");
}

void TestReconnectCompletionWaitsForFreshTelemetry() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kReconnectAgent);
  configuration.profile.expected_cycles = 1U;
  configuration.duration_ns = 0;
  auto core = StartedCore(configuration);
  core.ObserveEndpointGraph(false, kMillisecond);
  core.ObserveHeartbeat(HeartbeatAt(2 * kMillisecond, 2U, 2U, 1001U));
  core.ObserveEndpointGraph(true, 3 * kMillisecond);
  core.ObserveDiagnostics(
      DiagnosticsAt(4 * kMillisecond, 2U, 1001U, 0U, 0U, 0U, 0U));

  Expect(!core.CampaignComplete(4 * kMillisecond),
         "graph recovery alone cannot complete a reconnect cycle");
  const std::int64_t samples_start = 5 * kMillisecond;
  core.ObserveTelemetry(CampaignTelemetry::kHeartbeat, samples_start, true);
  core.ObserveTelemetry(CampaignTelemetry::kDiagnostics, samples_start, true);
  core.ObserveTelemetry(CampaignTelemetry::kMotorState, samples_start, true);
  core.ObserveTelemetry(CampaignTelemetry::kPwmServoState, samples_start, true);
  core.ObserveTelemetry(CampaignTelemetry::kImu, samples_start, true);
  core.ObserveTelemetry(CampaignTelemetry::kBattery, samples_start, true);
  Expect(!core.CampaignComplete(samples_start),
         "one fresh sample per stream is not enough to prove settling");
  ObserveRequiredConnectedWindow(&core, samples_start + kMillisecond, kSecond);
  const std::int64_t finish_time = samples_start + kMillisecond + kSecond;
  Expect(core.CampaignComplete(finish_time),
         "two fresh samples per required stream complete settling");
  const CampaignSummary summary = core.Finish(finish_time);
  Expect(!HasFailure(summary, CampaignFailure::kSessionInvalid),
         "fast Agent session rotation needs no synthetic heartbeat outage");
}

QualificationCampaignCore StartedCore(CampaignConfiguration configuration) {
  QualificationCampaignCore core(configuration);
  core.ObserveHeartbeat(HeartbeatAt(0, 1U, 1U, 1000U));
  core.ObserveDiagnostics(DiagnosticsAt(0, 1U, 1000U, 0U, 0U, 0U, 0U));
  core.ObserveEndpointGraph(true, 0);
  Expect(core.BeginEvidence(0), "core fixture enters its evidence window");
  return core;
}

void TestCanonicalAttestationAndFailureEdges() {
  CampaignConfiguration canonical_configuration;
  canonical_configuration.profile =
      CampaignProfileForMode(CampaignMode::kLoad500);
  canonical_configuration.duration_ns =
      canonical_configuration.profile.canonical_duration_ns;
  canonical_configuration.require_button_stimulus = true;
  canonical_configuration.require_valid_imu = true;
  auto canonical = StartedCore(canonical_configuration);
  const CampaignSummary canonical_summary = canonical.Finish(0);
  Expect(canonical_summary.canonical_profile,
         "canonical attestation compares the complete locked profile");

  CampaignConfiguration mutated_configuration = canonical_configuration;
  mutated_configuration.profile.command_rates[1U].numerator = 11U;
  auto mutated = StartedCore(mutated_configuration);
  const CampaignSummary mutated_summary = mutated.Finish(0);
  Expect(!mutated_summary.canonical_profile,
         "a single mutated command rate invalidates canonical attestation");

  CampaignConfiguration tolerant_configuration = canonical_configuration;
  tolerant_configuration.minimum_telemetry_gap_ns = kSecond;
  auto tolerant = StartedCore(tolerant_configuration);
  const CampaignSummary tolerant_summary = tolerant.Finish(0);
  Expect(!tolerant_summary.canonical_profile,
         "a test telemetry-gap override invalidates canonical attestation");

  CampaignConfiguration reconnect_configuration;
  reconnect_configuration.profile =
      CampaignProfileForMode(CampaignMode::kReconnectAgent);
  reconnect_configuration.duration_ns = 0;
  auto reconnect = StartedCore(reconnect_configuration);
  const CampaignSummary reconnect_summary = reconnect.Finish(0);
  Expect(reconnect_summary.canonical_profile,
         "canonical reconnect profile does not require periodic buttons");

  CampaignConfiguration button_configuration = reconnect_configuration;
  button_configuration.require_button_stimulus = true;
  auto missing_button = StartedCore(button_configuration);
  const CampaignSummary missing_button_summary = missing_button.Finish(0);
  Expect(
      missing_button_summary.button_stimulus == CampaignMetricStatus::kFail &&
          HasFailure(missing_button_summary, CampaignFailure::kTelemetryRate),
      "opt-in reconnect button coverage fails when no event is observed");
  auto observed_button = StartedCore(button_configuration);
  observed_button.ObserveTelemetry(CampaignTelemetry::kButtonEvents,
                                   kMillisecond, true);
  const CampaignSummary observed_button_summary =
      observed_button.Finish(kMillisecond);
  Expect(observed_button_summary.button_stimulus == CampaignMetricStatus::kPass,
         "one valid event satisfies opt-in reconnect button coverage");

  CampaignConfiguration short_configuration;
  short_configuration.profile = CampaignProfileForMode(CampaignMode::kLoad500);
  short_configuration.duration_ns = kSecond;
  auto nonmonotonic = StartedCore(short_configuration);
  nonmonotonic.ObserveHeartbeat(HeartbeatAt(0, 1U, 2U, 1001U));
  nonmonotonic.ObserveDiagnostics(DiagnosticsAt(0, 1U, 1001U, 1U, 0U, 0U, 0U));
  const CampaignSummary nonmonotonic_summary = nonmonotonic.Finish(kSecond);
  Expect(HasFailure(nonmonotonic_summary, CampaignFailure::kSessionInvalid),
         "non-monotonic heartbeat arrival time is rejected");
  Expect(
      HasFailure(nonmonotonic_summary, CampaignFailure::kDiagnosticRegression),
      "non-monotonic diagnostics arrival time is rejected");

  auto impossible_service = StartedCore(short_configuration);
  impossible_service.RecordServiceResult(CampaignService::kMotorModel, 0U,
                                         false, false);
  impossible_service.RecordServiceSkipped(CampaignService::kBusConfigure);
  const CampaignSummary service_summary = impossible_service.Finish(kSecond);
  Expect(HasFailure(service_summary, CampaignFailure::kInvalidConfiguration),
         "an OK result without a sent request is impossible and rejected");
  Expect(service_summary.service_skips[3U] == 1U,
         "service skips remain visible in the machine evidence summary");
}

void TestOverlappingAndUsbTransitionsStayIncomplete() {
  CampaignConfiguration reconnect_configuration;
  reconnect_configuration.profile =
      CampaignProfileForMode(CampaignMode::kReconnectAgent);
  reconnect_configuration.duration_ns = 0;
  auto reconnect = StartedCore(reconnect_configuration);
  static_cast<void>(reconnect.Poll(2 * kSecond));
  reconnect.ObserveHeartbeat(HeartbeatAt(2 * kSecond, 2U, 2U, 3000U));
  static_cast<void>(reconnect.Poll(4 * kSecond));
  reconnect.ObserveHeartbeat(HeartbeatAt(4 * kSecond, 3U, 3U, 5000U));
  reconnect.ObserveEndpointGraph(true, (4 * kSecond) + (100 * kMillisecond));
  const CampaignSummary overlap =
      reconnect.Finish((4 * kSecond) + (100 * kMillisecond));
  Expect(HasFailure(overlap, CampaignFailure::kEndpointRecoveryTimeout),
         "a new transition cannot erase an unrecovered previous transition");
  Expect(!overlap.transitions[0U].endpoint_recovered,
         "the older unrecovered transition remains explicit in evidence");

  CampaignConfiguration usb_configuration;
  usb_configuration.profile =
      CampaignProfileForMode(CampaignMode::kReconnectUsb);
  usb_configuration.duration_ns = 0;
  auto usb = StartedCore(usb_configuration);
  static_cast<void>(usb.Poll(11 * kSecond));
  usb.ObserveHeartbeat(HeartbeatAt(11 * kSecond, 2U, 2U, 12000U));
  usb.ObserveEndpointGraph(true, (11 * kSecond) + (100 * kMillisecond));
  const CampaignSummary usb_summary =
      usb.Finish((11 * kSecond) + (100 * kMillisecond));
  Expect(usb_summary.usb_outage_rotation == CampaignMetricStatus::kNotObserved,
         "heartbeat gaps never synthesize a pass for physical USB timing");
}

CampaignSummary RunUsbConnectedBoundary(std::int64_t last_heartbeat_ns) {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kReconnectUsb);
  configuration.duration_ns = 0;
  auto core = StartedCore(configuration);
  const std::uint32_t uptime = static_cast<std::uint32_t>(
      (last_heartbeat_ns / kMillisecond) + INT64_C(1000));
  core.ObserveHeartbeat(HeartbeatAt(last_heartbeat_ns, 1U, 2U, uptime));
  const std::int64_t outage_observation = last_heartbeat_ns + kSecond;
  static_cast<void>(core.Poll(outage_observation));
  core.ObserveHeartbeat(
      HeartbeatAt(outage_observation + kMillisecond, 2U, 3U, uptime + 1001U));
  core.ObserveEndpointGraph(true, outage_observation + (100 * kMillisecond));
  return core.Finish(outage_observation + (100 * kMillisecond));
}

void TestConservativeUsbConnectedBoundaryAndGraphTime() {
  const CampaignSummary too_short =
      RunUsbConnectedBoundary((10 * kSecond) - kMillisecond);
  Expect(HasFailure(too_short, CampaignFailure::kSessionInvalid),
         "9.999 seconds at the last heartbeat fails the connected interval");
  const CampaignSummary exact = RunUsbConnectedBoundary(10 * kSecond);
  Expect(!HasFailure(exact, CampaignFailure::kSessionInvalid),
         "10.000 seconds at the last heartbeat meets the observable boundary");

  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kReconnectAgent);
  configuration.duration_ns = 0;
  auto graph = StartedCore(configuration);
  graph.ObserveEndpointGraph(false, 2 * kSecond);
  graph.ObserveEndpointGraph(true, kSecond);
  const CampaignSummary backward = graph.Finish(2 * kSecond);
  Expect(HasFailure(backward, CampaignFailure::kInvalidConfiguration),
         "backward endpoint-graph observation time is rejected");
}

void TestReconnectRatesAndDiagnosticIntegrity() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kReconnectAgent);
  configuration.duration_ns = 0;
  auto connected = StartedCore(configuration);
  ObserveRequiredConnectedSamples(&connected, kMillisecond);
  connected.ObserveEndpointGraph(false, 2 * kSecond);
  static_cast<void>(connected.Poll(10 * kSecond));
  connected.ObserveHeartbeat(HeartbeatAt(10 * kSecond, 2U, 2U, 11000U));
  connected.ObserveEndpointGraph(true, (10 * kSecond) + (100 * kMillisecond));
  connected.ObserveDiagnostics(DiagnosticsAt(
      (10 * kSecond) + (101 * kMillisecond), 2U, 11000U, 0U, 0U, 0U, 0U));
  const std::int64_t final_samples_start =
      (10 * kSecond) + (102 * kMillisecond);
  ObserveRequiredConnectedWindow(&connected, final_samples_start, kSecond);
  const std::int64_t finish_time = final_samples_start + kSecond;
  const CampaignSummary connected_summary = connected.Finish(finish_time);
  Expect(
      !HasFailure(connected_summary, CampaignFailure::kTelemetryRate),
      "legitimate outage time does not depress connected-window rates mask=" +
          std::to_string(connected_summary.failure_mask));
  Expect(connected_summary.telemetry_rates_hz[2U] < 10.0,
         "total-wall telemetry rate remains informational during reconnect");
  Expect(
      connected_summary.connected_telemetry_gaps == CampaignMetricStatus::kPass,
      "each required stream has a bounded connected-window gap");

  auto generation_error = StartedCore(configuration);
  CampaignDiagnosticsObservation changed_generation =
      DiagnosticsAt(kSecond, 2U, 2000U, 100U, 1U, 0U, 1000U);
  changed_generation.hard_error_total = 1U;
  generation_error.ObserveDiagnostics(changed_generation);
  const CampaignSummary generation_summary = generation_error.Finish(kSecond);
  Expect(HasFailure(generation_summary, CampaignFailure::kDiagnosticError),
         "session-generation change cannot hide a new hard error");

  auto unexpected_reset = StartedCore(configuration);
  unexpected_reset.ObserveDiagnostics(
      DiagnosticsAt(kSecond, 2U, 1U, 0U, 0U, 0U, 0U));
  const CampaignSummary reset_summary = unexpected_reset.Finish(kSecond);
  Expect(HasFailure(reset_summary, CampaignFailure::kUnexpectedReset),
         "Agent reconnect mode cannot hide MCU uptime regression");

  auto counter_regression = StartedCore(configuration);
  CampaignDiagnosticsObservation first_counter =
      DiagnosticsAt(kSecond, 1U, 2000U, 100U, 1U, 0U, 1000U);
  first_counter.monotonic_counters[37U] = 9U;
  counter_regression.ObserveDiagnostics(first_counter);
  CampaignDiagnosticsObservation second_counter =
      DiagnosticsAt(2 * kSecond, 1U, 3000U, 200U, 2U, 0U, 1000U);
  second_counter.monotonic_counters[37U] = 8U;
  counter_regression.ObserveDiagnostics(second_counter);
  const CampaignSummary counter_summary =
      counter_regression.Finish(2 * kSecond);
  Expect(HasFailure(counter_summary, CampaignFailure::kDiagnosticRegression),
         "every individual monotonic diagnostics counter is enforced");
}

void TestResetBindingAndSoakGaps() {
  CampaignConfiguration reset_configuration;
  reset_configuration.profile = CampaignProfileForMode(CampaignMode::kResetMcu);
  reset_configuration.duration_ns = 0;
  auto reset = StartedCore(reset_configuration);
  reset.ObserveHeartbeat(HeartbeatAt(kSecond, 2U, 2U, 1U));
  reset.ObserveEndpointGraph(true, kSecond + (100 * kMillisecond));
  CampaignDiagnosticsObservation pin_reset =
      DiagnosticsAt(kSecond + (200 * kMillisecond), 2U, 1U, 0U, 0U, 0U, 0U);
  pin_reset.reset_reason = kResetPin;
  reset.ObserveDiagnostics(pin_reset);
  reset.ObserveHeartbeat(HeartbeatAt(2 * kSecond, 3U, 3U, 2U));
  reset.ObserveEndpointGraph(true, (2 * kSecond) + (100 * kMillisecond));
  const CampaignSummary reset_summary = reset.Finish(3 * kSecond);
  Expect(reset_summary.transitions[0U].mcu_uptime_regression_observed,
         "first reset is bound to its exact session transition");
  Expect(reset_summary.transitions[0U].reset_reason == kResetPin,
         "the accepted operator reset cause is recorded on its transition");
  Expect(!reset_summary.transitions[1U].mcu_uptime_regression_observed,
         "second transition cannot borrow the first reset observation");
  Expect(reset_summary.transitions[1U].reset_reason == kResetUnknown,
         "a transition without reset proof records no accepted reset cause");
  Expect(HasFailure(reset_summary, CampaignFailure::kResetCycleMissing),
         "every reset campaign transition requires its own uptime regression");

  CampaignConfiguration soak_configuration;
  soak_configuration.profile = CampaignProfileForMode(CampaignMode::kSoak);
  soak_configuration.duration_ns = 2 * kSecond;
  auto soak = StartedCore(soak_configuration);
  soak.ObserveTelemetry(CampaignTelemetry::kMotorState, kMillisecond, true);
  soak.ObserveTelemetry(CampaignTelemetry::kMotorState, 100 * kMillisecond,
                        true);
  const CampaignSummary soak_summary = soak.Finish(2 * kSecond);
  Expect(HasFailure(soak_summary, CampaignFailure::kTelemetryRate),
         "soak detects a connected stream gap longer than two periods");
  Expect(soak_summary.complete_one_second_wire_windows ==
             CampaignMetricStatus::kNotObserved,
         "diagnostics intervals never claim aligned one-second windows");
}

void TestResetCausePolicy() {
  CampaignConfiguration configuration;
  configuration.profile = CampaignProfileForMode(CampaignMode::kResetMcu);
  configuration.duration_ns = 0;

  constexpr std::array<std::uint8_t, 4U> kAcceptedReasons{
      kResetPowerOn, kResetPin, kResetSoftware, kResetBrownout};
  for (const std::uint8_t reason : kAcceptedReasons) {
    QualificationCampaignCore core(configuration);
    core.ObserveHeartbeat(HeartbeatAt(0, 1U, 1U, 1000U));
    CampaignDiagnosticsObservation diagnostics =
        DiagnosticsAt(0, 1U, 1000U, 0U, 0U, 0U, 0U);
    diagnostics.reset_reason = reason;
    core.ObserveDiagnostics(diagnostics);
    core.ObserveEndpointGraph(true, 0);
    Expect(core.BeginEvidence(0),
           "an explicitly accepted operator reset cause permits evidence");
  }

  constexpr std::array<std::uint8_t, 4U> kRejectedReasons{
      kResetIndependentWatchdog, kResetWindowWatchdog, kResetLowPower,
      kResetUnknown};
  for (const std::uint8_t reason : kRejectedReasons) {
    QualificationCampaignCore core(configuration);
    core.ObserveHeartbeat(HeartbeatAt(0, 1U, 1U, 1000U));
    CampaignDiagnosticsObservation diagnostics =
        DiagnosticsAt(0, 1U, 1000U, 0U, 0U, 0U, 0U);
    diagnostics.reset_reason = reason;
    core.ObserveDiagnostics(diagnostics);
    core.ObserveEndpointGraph(true, 0);
    Expect(!core.BeginEvidence(0),
           "watchdog, low-power, and unknown causes fail before evidence");
    const CampaignSummary summary = core.Finish(0);
    Expect(HasFailure(summary, CampaignFailure::kUnexpectedReset),
           "every non-operator reset cause is an unexpected-reset failure");
  }

  auto rejected_transition = StartedCore(configuration);
  rejected_transition.ObserveHeartbeat(HeartbeatAt(kSecond, 2U, 2U, 1U));
  rejected_transition.ObserveEndpointGraph(true,
                                           kSecond + (100 * kMillisecond));
  CampaignDiagnosticsObservation watchdog_reset =
      DiagnosticsAt(kSecond + (200 * kMillisecond), 2U, 1U, 0U, 0U, 0U, 0U);
  watchdog_reset.reset_reason = kResetIndependentWatchdog;
  rejected_transition.ObserveDiagnostics(watchdog_reset);
  const CampaignSummary rejected_summary =
      rejected_transition.Finish(2 * kSecond);
  Expect(rejected_summary.observed_mcu_resets == 0U,
         "a watchdog reset never counts as an operator reset cycle");
  Expect(!rejected_summary.transitions[0U].mcu_uptime_regression_observed &&
             rejected_summary.transitions[0U].reset_reason == kResetUnknown,
         "a rejected reset cause is not bound to transition evidence");
  Expect(HasFailure(rejected_summary, CampaignFailure::kUnexpectedReset) &&
             HasFailure(rejected_summary, CampaignFailure::kResetCycleMissing),
         "a rejected reset both fails safety and leaves the cycle unproven");
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

void TestEvidenceFilesAreRevisionBoundAndExplicit() {
  CampaignSummary summary = RunPassingShortCampaign();
  CampaignEvidenceMetadata metadata;
  metadata.run_id = "unit-test-run";
  metadata.source_revision = "source-r3";
  metadata.firmware_sha256 =
      "963f2834a08b9e86dbe736e58cfee83a2378983cdcaf2f1bc1a3ea0257136e8f";
  metadata.host_revision = "host-r3";
  metadata.ros_distribution = "humble";
  metadata.board_serial = "RRCLITE-UNIT-BOARD";
  metadata.fixture_revision = "fixture-unit-r1";
  metadata.campaign_mode = "load500";
  metadata.start_time_utc = "2026-08-06T00:00:00Z";
  metadata.finish_time_utc = "2026-08-06T00:00:02Z";

  const auto nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("rrclite-campaign-evidence-" + std::to_string(nonce));
  std::string error;
  Expect(WriteCampaignEvidence(directory.string(), metadata, summary, &error),
         "valid metadata writes JSON, CSV, transition, and JUnit evidence: " +
             error);
  const std::string json = ReadFile(directory / "summary.json");
  const std::string junit = ReadFile(directory / "junit.xml");
  Expect(json.find(metadata.firmware_sha256) != std::string::npos,
         "summary binds the exact firmware digest");
  Expect(json.find(R"("release_qualification": "INCOMPLETE")") !=
             std::string::npos,
         "summary cannot be mistaken for release evidence");
  Expect(junit.find("requires an independent serial instrument") !=
             std::string::npos,
         "JUnit marks independent capture as skipped, not passed");

  CampaignEvidenceMetadata unsafe_metadata = metadata;
  unsafe_metadata.run_id = "../unsafe";
  const std::filesystem::path unsafe_directory =
      directory.parent_path() / ("rrclite-unsafe-" + std::to_string(nonce));
  error.clear();
  Expect(!WriteCampaignEvidence(unsafe_directory.string(), unsafe_metadata,
                                summary, &error),
         "run_id cannot introduce a path component");
  Expect(!std::filesystem::exists(unsafe_directory),
         "invalid metadata creates no partial evidence directory");

  CampaignSummary nonfinite_summary = summary;
  nonfinite_summary.telemetry_rates_hz[0U] =
      std::numeric_limits<double>::infinity();
  const std::filesystem::path nonfinite_directory =
      directory.parent_path() / ("rrclite-nonfinite-" + std::to_string(nonce));
  error.clear();
  Expect(!WriteCampaignEvidence(nonfinite_directory.string(), metadata,
                                nonfinite_summary, &error),
         "nonfinite numeric evidence is rejected before JSON emission");
  Expect(!std::filesystem::exists(nonfinite_directory),
         "nonfinite evidence creates no partial directory");

  error.clear();
  Expect(!WriteCampaignEvidence(directory.string(), metadata, summary, &error),
         "an existing evidence directory is immutable and never overwritten");
  std::error_code remove_error;
  std::filesystem::permissions(directory, std::filesystem::perms::owner_all,
                               std::filesystem::perm_options::add,
                               remove_error);
  Expect(!remove_error,
         "test restores owner permissions on the immutable bundle");
  remove_error.clear();
  std::filesystem::remove_all(directory, remove_error);
  Expect(!remove_error, "test evidence directory is removed cleanly");
}

}  // namespace

int main() {
  try {
    TestCanonicalProfiles();
    TestPassingShortCampaignAndMotorSafety();
    TestTelemetryDeadlineFailsBeforeFinish();
    TestReconnectObservationIsBounded();
    TestReconnectCompletionWaitsForFreshTelemetry();
    TestCanonicalAttestationAndFailureEdges();
    TestOverlappingAndUsbTransitionsStayIncomplete();
    TestConservativeUsbConnectedBoundaryAndGraphTime();
    TestReconnectRatesAndDiagnosticIntegrity();
    TestResetBindingAndSoakGaps();
    TestResetCausePolicy();
    TestEvidenceFilesAreRevisionBoundAndExplicit();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "FAIL: unexpected non-standard exception\n";
    return 1;
  }
  if (g_failures == 0) {
    std::cout << "qualification campaign core tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
