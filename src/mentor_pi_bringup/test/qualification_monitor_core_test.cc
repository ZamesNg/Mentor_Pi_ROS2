// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_monitor_core.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

namespace {

using mentor_pi_bringup::DiagnosticsObservation;
using mentor_pi_bringup::HeartbeatObservation;
using mentor_pi_bringup::ImuObservation;
using mentor_pi_bringup::IsValidQualificationBatteryState;
using mentor_pi_bringup::QualificationConfiguration;
using mentor_pi_bringup::QualificationFailure;
using mentor_pi_bringup::QualificationFailureBit;
using mentor_pi_bringup::QualificationMonitorCore;
using mentor_pi_bringup::QualificationStream;
using mentor_pi_bringup::QualificationSummary;
using mentor_pi_bringup::QualificationTimestamp;

constexpr std::int64_t kNanosecondsPerMillisecond = INT64_C(1000000);
constexpr std::int64_t kNanosecondsPerSecond = INT64_C(1000000000);
constexpr std::int64_t kTestDuration = 60 * kNanosecondsPerSecond;
constexpr std::uint32_t kSessionId = 42U;
constexpr std::size_t kImuPeripheralIndex = 1U;
constexpr std::size_t kImuErrorCounterIndex = 34U;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

bool HasFailure(std::uint64_t mask, QualificationFailure failure) {
  return (mask & QualificationFailureBit(failure)) != 0U;
}

QualificationTimestamp StampAt(std::int64_t time_ns) {
  QualificationTimestamp stamp;
  stamp.seconds =
      100 + static_cast<std::int32_t>(time_ns / kNanosecondsPerSecond);
  stamp.nanoseconds =
      static_cast<std::uint32_t>(time_ns % kNanosecondsPerSecond);
  return stamp;
}

void ObserveAllPublishers(QualificationMonitorCore* core) {
  for (std::uint8_t index = 0;
       index < static_cast<std::uint8_t>(QualificationStream::kCount);
       ++index) {
    core->ObservePublisher(static_cast<QualificationStream>(index), true, 0);
  }
}

DiagnosticsObservation DiagnosticsAt(std::int64_t time_ns) {
  const std::uint64_t seconds =
      static_cast<std::uint64_t>(time_ns / kNanosecondsPerSecond);
  DiagnosticsObservation observation;
  observation.stamp = StampAt(time_ns);
  observation.arrival_time_ns = time_ns;
  observation.uptime_ms =
      static_cast<std::uint32_t>(time_ns / kNanosecondsPerMillisecond);
  observation.session_generation = kSessionId;
  observation.transport_rx_bytes = seconds * 1000U;
  observation.transport_tx_bytes = seconds * 200U;
  observation.monotonic_counters[0] = observation.transport_rx_bytes;
  observation.monotonic_counters[1] = observation.transport_tx_bytes;
  observation.session_state = 3U;
  return observation;
}

ImuObservation ValidImuAt(std::int64_t time_ns) {
  ImuObservation observation;
  observation.stamp = StampAt(time_ns);
  observation.arrival_time_ns = time_ns;
  observation.valid = true;
  observation.vectors_finite = true;
  return observation;
}

void PopulatePassingRun(QualificationMonitorCore* core) {
  ObserveAllPublishers(core);

  std::uint32_t sequence = 0U;
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 500 * kNanosecondsPerMillisecond) {
    HeartbeatObservation heartbeat;
    heartbeat.stamp = StampAt(time_ns);
    heartbeat.arrival_time_ns = time_ns;
    heartbeat.sequence = sequence;
    ++sequence;
    heartbeat.uptime_ms =
        static_cast<std::uint32_t>(time_ns / kNanosecondsPerMillisecond);
    heartbeat.agent_session_id = kSessionId;
    heartbeat.state = 1U;
    heartbeat.time_synchronized = true;
    core->ObserveHeartbeat(heartbeat);
  }

  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += kNanosecondsPerSecond) {
    core->ObserveDiagnostics(DiagnosticsAt(time_ns));
    core->ObserveTelemetry(QualificationStream::kBattery, StampAt(time_ns),
                           time_ns, true);
  }
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 20 * kNanosecondsPerMillisecond) {
    core->ObserveTelemetry(QualificationStream::kMotorState, StampAt(time_ns),
                           time_ns, true);
    core->ObserveImu(ValidImuAt(time_ns));
  }
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 50 * kNanosecondsPerMillisecond) {
    core->ObserveTelemetry(QualificationStream::kPwmServoState,
                           StampAt(time_ns), time_ns, true);
  }
}

QualificationConfiguration MotorAgeEvidenceConfiguration() {
  QualificationConfiguration configuration;
  configuration.motor_command_age_evidence_required = true;
  return configuration;
}

QualificationSummary RunMotorAgeEvidence(std::uint32_t host_publications,
                                         std::uint32_t command_messages,
                                         std::uint32_t consumptions,
                                         std::uint32_t overwrites,
                                         std::uint32_t age_over_20_ms,
                                         std::uint32_t maximum_age_us) {
  QualificationMonitorCore core(MotorAgeEvidenceConfiguration());
  Expect(!core.MotorCommandAgeBaselineReady(),
         "motor commands stay gated before the first diagnostics baseline");
  PopulatePassingRun(&core);
  Expect(core.MotorCommandAgeBaselineReady(),
         "a clean first diagnostics sample opens the zero-command gate");
  core.ObserveMotorCommandPublisherCount(1U, kTestDuration);
  for (std::uint32_t index = 0U; index < host_publications; ++index) {
    core.RecordZeroMotorCommand();
  }

  constexpr std::int64_t kEvidenceFinishTime =
      kTestDuration + kNanosecondsPerSecond;
  DiagnosticsObservation final = DiagnosticsAt(kEvidenceFinishTime);
  final.command_messages = command_messages;
  final.motor_command_consumptions = consumptions;
  final.motor_mailbox_overwrites = overwrites;
  final.motor_command_age_over_20_ms = age_over_20_ms;
  final.motor_command_max_age_us = maximum_age_us;
  core.ObserveDiagnostics(final);
  return core.Finish(kEvidenceFinishTime);
}

struct CharacterizationOptions {
  bool time_synchronized = true;
  bool heartbeat_imu_healthy = false;
  bool imu_valid = false;
  bool imu_vectors_finite = true;
  bool imu_nonzero_stamp = false;
  std::size_t imu_sample_count = 1U;
  std::uint8_t heartbeat_state = 2U;
  std::uint8_t last_error_code = 6U;
  std::uint8_t last_error_source = 9U;
  std::uint16_t last_error_detail = 1U;
  std::uint32_t imu_error_count = 1U;
  std::uint32_t imu_timeout_count = 0U;
  std::uint32_t non_imu_error_count = 0U;
  std::uint32_t non_imu_timeout_count = 0U;
};

QualificationConfiguration CharacterizationConfiguration() {
  QualificationConfiguration configuration;
  configuration.imu_characterization_mode = true;
  return configuration;
}

DiagnosticsObservation CharacterizationDiagnosticsAt(
    std::int64_t time_ns, const CharacterizationOptions& options) {
  DiagnosticsObservation observation = DiagnosticsAt(time_ns);
  observation.peripheral_errors[kImuPeripheralIndex] =
      options.imu_error_count + options.imu_timeout_count;
  observation.peripheral_timeouts[kImuPeripheralIndex] =
      options.imu_timeout_count;
  observation.peripheral_errors[2U] =
      options.non_imu_error_count + options.non_imu_timeout_count;
  observation.peripheral_timeouts[2U] = options.non_imu_timeout_count;
  observation.diagnostic_fault_total =
      options.imu_error_count + 2U * options.imu_timeout_count +
      options.non_imu_error_count + 2U * options.non_imu_timeout_count;
  observation.monotonic_counters[kImuErrorCounterIndex] =
      options.imu_error_count + options.imu_timeout_count;
  observation.last_error_code = options.last_error_code;
  observation.last_error_source = options.last_error_source;
  observation.last_error_detail = options.last_error_detail;
  return observation;
}

void PopulateCharacterizationRun(QualificationMonitorCore* core,
                                 const CharacterizationOptions& options = {}) {
  ObserveAllPublishers(core);

  std::uint32_t sequence = 0U;
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 500 * kNanosecondsPerMillisecond) {
    HeartbeatObservation heartbeat;
    heartbeat.stamp = StampAt(time_ns);
    heartbeat.arrival_time_ns = time_ns;
    heartbeat.sequence = sequence;
    ++sequence;
    heartbeat.uptime_ms =
        static_cast<std::uint32_t>(time_ns / kNanosecondsPerMillisecond);
    heartbeat.agent_session_id = kSessionId;
    heartbeat.state = options.heartbeat_state;
    heartbeat.time_synchronized = options.time_synchronized;
    heartbeat.imu_healthy = options.heartbeat_imu_healthy;
    core->ObserveHeartbeat(heartbeat);
  }

  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += kNanosecondsPerSecond) {
    core->ObserveDiagnostics(CharacterizationDiagnosticsAt(time_ns, options));
    core->ObserveTelemetry(QualificationStream::kBattery, StampAt(time_ns),
                           time_ns, true);
  }
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 20 * kNanosecondsPerMillisecond) {
    core->ObserveTelemetry(QualificationStream::kMotorState, StampAt(time_ns),
                           time_ns, true);
  }
  for (std::int64_t time_ns = 0; time_ns <= kTestDuration;
       time_ns += 50 * kNanosecondsPerMillisecond) {
    core->ObserveTelemetry(QualificationStream::kPwmServoState,
                           StampAt(time_ns), time_ns, true);
  }
  for (std::size_t index = 0U; index < options.imu_sample_count; ++index) {
    ImuObservation imu;
    if (options.imu_nonzero_stamp) {
      imu.stamp = StampAt(0);
    }
    imu.arrival_time_ns = static_cast<std::int64_t>(index);
    imu.valid = options.imu_valid;
    imu.vectors_finite = options.imu_vectors_finite;
    core->ObserveImu(imu);
  }
}

void TestPassingImuCharacterizationRun() {
  QualificationMonitorCore core(CharacterizationConfiguration());
  PopulateCharacterizationRun(&core);
  const auto summary = core.Finish(kTestDuration);

  Expect(summary.passed, "exact unverified-IMU characterization run passes");
  Expect(summary.failure_mask == 0U,
         "passing characterization has no failure bits");
  Expect(summary.streams[static_cast<std::size_t>(QualificationStream::kImu)]
                 .sample_count == 1U,
         "characterization accepts the single invalid zero-stamp IMU sample");
  Expect(
      (summary.invalid_telemetry_mask &
       (UINT8_C(1) << static_cast<std::uint8_t>(QualificationStream::kImu))) ==
          0U,
      "expected unverified IMU sample is not reported as generic invalid "
      "telemetry");

  CharacterizationOptions late_join_options;
  late_join_options.imu_sample_count = 0U;
  QualificationMonitorCore late_join_core(CharacterizationConfiguration());
  PopulateCharacterizationRun(&late_join_core, late_join_options);
  const auto late_join_summary = late_join_core.Finish(kTestDuration);
  Expect(late_join_summary.passed,
         "late-joining characterization accepts no volatile IMU sample");

  QualificationMonitorCore strict_core(QualificationConfiguration{});
  PopulateCharacterizationRun(&strict_core);
  const auto strict_summary = strict_core.Finish(kTestDuration);
  Expect(!strict_summary.passed,
         "default strict mode does not accept characterization evidence");
  Expect(HasFailure(strict_summary.failure_mask,
                    QualificationFailure::kHeartbeatNotReady),
         "strict mode still requires a READY heartbeat");
  Expect(HasFailure(strict_summary.failure_mask,
                    QualificationFailure::kInvalidTelemetry),
         "strict mode still requires valid IMU telemetry");
}

void ExpectCharacterizationMismatch(const CharacterizationOptions& options,
                                    const std::string& description) {
  QualificationMonitorCore core(CharacterizationConfiguration());
  PopulateCharacterizationRun(&core, options);
  const auto summary = core.Finish(kTestDuration);
  Expect(!summary.passed, description + " fails characterization");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kImuCharacterizationMismatch),
         description + " reports the characterization mismatch bit");
}

void TestImuCharacterizationRejectsAdjacentStates() {
  CharacterizationOptions options;
  options.heartbeat_imu_healthy = true;
  ExpectCharacterizationMismatch(options, "IMU_HEALTHY heartbeat flag");

  options = {};
  options.heartbeat_state = 1U;
  ExpectCharacterizationMismatch(options, "READY heartbeat");

  options = {};
  options.heartbeat_state = 3U;
  ExpectCharacterizationMismatch(options, "FAULT heartbeat");

  options = {};
  options.time_synchronized = false;
  ExpectCharacterizationMismatch(options, "unsynchronized heartbeat");

  options = {};
  options.imu_valid = true;
  ExpectCharacterizationMismatch(options, "valid IMU sample");

  options = {};
  options.imu_vectors_finite = false;
  ExpectCharacterizationMismatch(options, "non-finite invalid IMU sample");

  options = {};
  options.imu_sample_count = 2U;
  ExpectCharacterizationMismatch(options, "duplicate initial IMU sample");

  options = {};
  options.imu_nonzero_stamp = true;
  ExpectCharacterizationMismatch(options, "nonzero IMU timestamp");

  options = {};
  options.imu_timeout_count = 1U;
  ExpectCharacterizationMismatch(options, "IMU timeout");

  options = {};
  options.non_imu_error_count = 1U;
  ExpectCharacterizationMismatch(options, "non-IMU peripheral error");

  options = {};
  options.non_imu_timeout_count = 1U;
  ExpectCharacterizationMismatch(options, "non-IMU peripheral timeout");

  options = {};
  options.last_error_code = 5U;
  ExpectCharacterizationMismatch(options, "wrong last error code");

  options = {};
  options.last_error_source = 8U;
  ExpectCharacterizationMismatch(options, "wrong last error source");

  options = {};
  options.last_error_detail = 2U;
  ExpectCharacterizationMismatch(options, "wrong last error detail");

  options = {};
  options.imu_error_count = 0U;
  ExpectCharacterizationMismatch(options, "missing IMU error");

  options = {};
  options.imu_error_count = 2U;
  ExpectCharacterizationMismatch(options, "duplicate IMU error");
}

void TestPassingProductionRateRun() {
  QualificationMonitorCore core(QualificationConfiguration{});
  PopulatePassingRun(&core);
  core.RecordZeroMotorCommand();
  const auto summary = core.Finish(kTestDuration);

  Expect(summary.passed, "contract-rate run passes");
  Expect(summary.failure_mask == 0U, "passing run has no failure bits");
  Expect(summary.agent_session_id == kSessionId,
         "passing run reports the session ID");
  Expect(summary.streams[static_cast<std::size_t>(
                             QualificationStream::kMotorState)]
                 .observed_rate_hz == 50.0,
         "motor rate is measured across the whole qualification run");
  Expect(summary.streams[static_cast<std::size_t>(
                             QualificationStream::kButtonEvents)]
                 .sample_count == 0U,
         "event-driven button stream does not require an event");
  Expect(summary.zero_motor_commands_published == 1U,
         "zero-command accounting is bounded and explicit");
  Expect(summary.transport_bytes_per_second == 1200.0,
         "diagnostics produce the aggregate transport rate");
  Expect(summary.maximum_transport_interval_bytes_per_second == 1200.0,
         "diagnostics produce the maximum interval transport rate");
}

void TestBatteryContractBoundaries() {
  Expect(IsValidQualificationBatteryState(1U, 5000U, true),
         "positive voltage and minimum threshold are valid");
  Expect(IsValidQualificationBatteryState(20000U, 20000U, true),
         "20,000 mV voltage and threshold boundaries are valid");
  Expect(!IsValidQualificationBatteryState(0U, 6300U, true),
         "zero voltage is invalid");
  Expect(!IsValidQualificationBatteryState(20001U, 6300U, true),
         "voltage above the public 20,000 mV ceiling is invalid");
  Expect(!IsValidQualificationBatteryState(7000U, 4999U, true),
         "threshold below the public range is invalid");
  Expect(!IsValidQualificationBatteryState(7000U, 20001U, true),
         "threshold above the public range is invalid");
  Expect(!IsValidQualificationBatteryState(7000U, 6300U, false),
         "an explicitly invalid battery sample fails strict preflight");
}

void TestTransportIntervalCannotHideInWholeRunAverage() {
  QualificationMonitorCore core(QualificationConfiguration{});
  DiagnosticsObservation first = DiagnosticsAt(0);
  first.transport_rx_bytes = 0U;
  first.transport_tx_bytes = 0U;
  first.monotonic_counters[0] = 0U;
  first.monotonic_counters[1] = 0U;
  core.ObserveDiagnostics(first);

  DiagnosticsObservation burst = DiagnosticsAt(kNanosecondsPerSecond);
  burst.transport_rx_bytes = 69999U;
  burst.transport_tx_bytes = 1U;
  burst.monotonic_counters[0] = burst.transport_rx_bytes;
  burst.monotonic_counters[1] = burst.transport_tx_bytes;
  core.ObserveDiagnostics(burst);

  DiagnosticsObservation idle = DiagnosticsAt(kTestDuration);
  idle.transport_rx_bytes = burst.transport_rx_bytes;
  idle.transport_tx_bytes = burst.transport_tx_bytes;
  idle.monotonic_counters[0] = idle.transport_rx_bytes;
  idle.monotonic_counters[1] = idle.transport_tx_bytes;
  core.ObserveDiagnostics(idle);

  const auto summary = core.Finish(kTestDuration);
  Expect(summary.transport_bytes_per_second < 2000.0,
         "later idle time makes the whole-run transport average look safe");
  Expect(summary.maximum_transport_interval_bytes_per_second == 70000.0,
         "the over-limit interval is retained independently of the average");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kTransportRateExceeded),
         "a 70,000 B/s interval fails the strict less-than ceiling");
}

void TestMotorCommandAgeEvidenceBoundaries() {
  const QualificationSummary exact_one_percent =
      RunMotorAgeEvidence(100U, 100U, 100U, 0U, 1U, 99999U);
  Expect(exact_one_percent.passed,
         "one strict-over-20-ms sample out of 100 passes nearest-rank p99");
  Expect(exact_one_percent.motor_command_consumptions == 100U,
         "motor command consumption delta is reported");
  Expect(exact_one_percent.motor_command_age_over_20_ms == 1U,
         "strict-over-20-ms delta is reported");
  Expect(exact_one_percent.motor_command_max_age_us == 99999U,
         "99,999 us remains below the absolute command-age ceiling");
  Expect(exact_one_percent.zero_motor_commands_in_evidence_window == 100U,
         "host publications use matching diagnostics boundaries");

  const QualificationSummary p99_failure =
      RunMotorAgeEvidence(100U, 100U, 100U, 0U, 2U, 99999U);
  Expect(HasFailure(p99_failure.failure_mask,
                    QualificationFailure::kMotorCommandAgeP99Exceeded),
         "two strict-over-20-ms samples out of 100 fail nearest-rank p99");

  const QualificationSummary maximum_failure =
      RunMotorAgeEvidence(100U, 100U, 100U, 0U, 0U, 100000U);
  Expect(HasFailure(maximum_failure.failure_mask,
                    QualificationFailure::kMotorCommandAgeMaximumExceeded),
         "the strict 100,000 us absolute-age boundary fails");

  const QualificationSummary no_samples =
      RunMotorAgeEvidence(1U, 1U, 0U, 0U, 0U, 0U);
  Expect(HasFailure(no_samples.failure_mask,
                    QualificationFailure::kMotorCommandAgeEvidenceMissing),
         "zero command-age samples cannot pass evidence mode");
}

void TestMotorCommandAcceptanceCorrelationBoundaries() {
  const QualificationSummary coalesced =
      RunMotorAgeEvidence(100U, 100U, 60U, 39U, 0U, 20000U);
  Expect(coalesced.passed,
         "consumptions plus mailbox overwrites account for coalescing");
  Expect(coalesced.motor_commands_accounted == 99U,
         "coalesced command accounting is explicit");

  const QualificationSummary lower_boundary =
      RunMotorAgeEvidence(100U, 100U, 94U, 0U, 0U, 20000U);
  Expect(lower_boundary.passed,
         "94 observed plus one final pending command meets the 95% boundary");

  const QualificationSummary excessive_loss =
      RunMotorAgeEvidence(100U, 100U, 93U, 0U, 0U, 20000U);
  Expect(HasFailure(excessive_loss.failure_mask,
                    QualificationFailure::kMotorCommandAcceptanceOutOfRange),
         "more than the bounded best-effort loss fails correlation");

  const QualificationSummary prebaseline_pending =
      RunMotorAgeEvidence(100U, 100U, 100U, 1U, 0U, 20000U);
  Expect(prebaseline_pending.passed,
         "one pre-baseline pending generation is within the upper boundary");

  const QualificationSummary inflated =
      RunMotorAgeEvidence(100U, 100U, 100U, 2U, 0U, 20000U);
  Expect(HasFailure(inflated.failure_mask,
                    QualificationFailure::kMotorCommandAcceptanceOutOfRange),
         "firmware evidence inflated by more than one command fails");

  const QualificationSummary incoherent_generic_count =
      RunMotorAgeEvidence(100U, 98U, 100U, 0U, 0U, 20000U);
  Expect(HasFailure(incoherent_generic_count.failure_mask,
                    QualificationFailure::kMotorCommandAcceptanceOutOfRange),
         "motor accounting cannot outrun the generic callback count by two");
}

void TestMotorCommandEvidenceBaselinePublisherAndRegression() {
  QualificationMonitorCore contaminated(MotorAgeEvidenceConfiguration());
  DiagnosticsObservation contaminated_baseline = DiagnosticsAt(0);
  contaminated_baseline.motor_command_consumptions = 1U;
  contaminated_baseline.motor_command_max_age_us = 1U;
  contaminated.ObserveDiagnostics(contaminated_baseline);
  Expect(!contaminated.MotorCommandAgeBaselineReady(),
         "a contaminated diagnostics baseline never opens the send gate");
  const QualificationSummary contaminated_summary =
      contaminated.Finish(kTestDuration);
  Expect(HasFailure(contaminated_summary.failure_mask,
                    QualificationFailure::kMotorCommandAgeBaselineContaminated),
         "nonzero first command-age evidence is rejected as contaminated");
  Expect(contaminated_summary.zero_motor_commands_published == 0U,
         "the core records no pre-baseline zero command");

  QualificationMonitorCore publisher_core(MotorAgeEvidenceConfiguration());
  PopulatePassingRun(&publisher_core);
  publisher_core.ObserveMotorCommandPublisherCount(1U, kTestDuration / 2);
  publisher_core.ObserveMotorCommandPublisherCount(2U, kTestDuration / 2);
  publisher_core.ObserveMotorCommandPublisherCount(1U, kTestDuration);
  const QualificationSummary publisher_summary =
      publisher_core.Finish(kTestDuration);
  Expect(HasFailure(publisher_summary.failure_mask,
                    QualificationFailure::kMotorCommandPublisherInvalid),
         "a transient second motor-command publisher is retained as failure");

  QualificationMonitorCore regression(MotorAgeEvidenceConfiguration());
  regression.ObserveMotorCommandPublisherCount(1U, kTestDuration);
  regression.ObserveDiagnostics(DiagnosticsAt(0));
  DiagnosticsObservation advanced = DiagnosticsAt(kNanosecondsPerSecond);
  advanced.command_messages = 10U;
  advanced.motor_command_consumptions = 10U;
  advanced.motor_command_age_over_20_ms = 1U;
  advanced.motor_command_max_age_us = 30000U;
  regression.ObserveDiagnostics(advanced);
  DiagnosticsObservation regressed = DiagnosticsAt(2 * kNanosecondsPerSecond);
  regressed.session_generation = kSessionId + 1U;
  regressed.command_messages = 9U;
  regressed.motor_command_consumptions = 9U;
  regressed.motor_command_age_over_20_ms = 0U;
  regressed.motor_command_max_age_us = 20000U;
  regression.ObserveDiagnostics(regressed);
  const QualificationSummary regression_summary =
      regression.Finish(kTestDuration);
  Expect(HasFailure(regression_summary.failure_mask,
                    QualificationFailure::kMotorCommandAgeCounterRegression),
         "motor command-age counter regression has a dedicated failure");
  Expect(HasFailure(regression_summary.failure_mask,
                    QualificationFailure::kSessionChanged),
         "session change remains independently visible with age regression");
}

void TestPublisherAndRateFailures() {
  QualificationMonitorCore core(QualificationConfiguration{});
  ObserveAllPublishers(&core);
  core.ObservePublisher(QualificationStream::kImu, false,
                        kNanosecondsPerSecond);
  core.ObserveTelemetry(QualificationStream::kMotorState, StampAt(0), 0, true);
  core.ObserveTelemetry(QualificationStream::kMotorState,
                        StampAt(20 * kNanosecondsPerMillisecond),
                        20 * kNanosecondsPerMillisecond, true);
  const auto summary = core.Finish(kTestDuration);

  Expect(HasFailure(summary.failure_mask, QualificationFailure::kPublisherLost),
         "publisher disappearance is retained");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kPublisherMissing),
      "publisher absent at finish fails discovery");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kRateOutOfRange),
      "missing periodic telemetry fails its rate");
  Expect(summary.streams[static_cast<std::size_t>(
                             QualificationStream::kMotorState)]
                 .observed_rate_hz < 1.0,
         "two adjacent samples cannot impersonate a full-duration 50 Hz run");
  Expect(
      (summary.lost_publisher_mask & (UINT8_C(1) << static_cast<std::uint8_t>(
                                          QualificationStream::kImu))) != 0U,
      "lost-publisher mask identifies IMU");
}

void TestSessionStampAndProgressFailures() {
  QualificationMonitorCore core(QualificationConfiguration{});
  ObserveAllPublishers(&core);

  HeartbeatObservation first;
  first.stamp = StampAt(kNanosecondsPerSecond);
  first.arrival_time_ns = kNanosecondsPerSecond;
  first.sequence = 10U;
  first.uptime_ms = 1000U;
  first.agent_session_id = kSessionId;
  first.state = 1U;
  first.time_synchronized = true;
  core.ObserveHeartbeat(first);

  HeartbeatObservation second = first;
  second.stamp = QualificationTimestamp{};
  second.arrival_time_ns += 500 * kNanosecondsPerMillisecond;
  second.agent_session_id = kSessionId + 1U;
  second.time_synchronized = false;
  core.ObserveHeartbeat(second);

  const auto summary = core.Finish(kTestDuration);
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kSessionChanged),
      "session ID change fails continuity");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kSequenceRegression),
         "duplicate heartbeat sequence fails progress");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kUptimeRegression),
      "duplicate uptime fails progress");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kTimeSynchronizationLost),
         "clearing synchronized flag fails continuity");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kTimestampInvalid),
      "zero timestamp after synchronization is invalid");
}

void TestDiagnosticChangesAndRegression() {
  QualificationMonitorCore core(QualificationConfiguration{});
  ObserveAllPublishers(&core);

  HeartbeatObservation heartbeat;
  heartbeat.stamp = StampAt(0);
  heartbeat.arrival_time_ns = 0;
  heartbeat.sequence = 1U;
  heartbeat.uptime_ms = 1U;
  heartbeat.agent_session_id = kSessionId;
  heartbeat.state = 1U;
  heartbeat.time_synchronized = true;
  core.ObserveHeartbeat(heartbeat);

  DiagnosticsObservation first = DiagnosticsAt(0);
  first.monotonic_counters[10] = 9U;
  core.ObserveDiagnostics(first);
  DiagnosticsObservation second = DiagnosticsAt(kNanosecondsPerSecond);
  second.monotonic_counters[10] = 8U;
  second.transport_error_total = 1U;
  second.diagnostic_fault_total = 1U;
  second.post_seal_allocation_attempts = 1U;
  second.session_state = 4U;
  second.last_reset_reason = 2U;
  core.ObserveDiagnostics(second);

  const auto summary = core.Finish(kTestDuration);
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kDiagnosticCounterRegression),
         "monotonic diagnostics may not regress");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kTransportErrorChanged),
         "new transport errors fail qualification");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kDiagnosticFaultChanged),
         "new diagnostic faults fail qualification");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kResetReasonChanged),
         "reset-reason change fails qualification");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kPostSealAllocation),
         "post-seal allocation attempts always fail");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kSessionNotActive),
      "non-active diagnostic session fails qualification");
}

void TestInvalidTelemetryAndSerialWrap() {
  QualificationMonitorCore core(QualificationConfiguration{});
  ObserveAllPublishers(&core);
  HeartbeatObservation first;
  first.stamp = StampAt(0);
  first.arrival_time_ns = 0;
  first.sequence = std::numeric_limits<std::uint32_t>::max();
  first.uptime_ms = std::numeric_limits<std::uint32_t>::max();
  first.agent_session_id = kSessionId;
  first.state = 1U;
  first.time_synchronized = true;
  core.ObserveHeartbeat(first);
  HeartbeatObservation second = first;
  second.stamp = StampAt(500 * kNanosecondsPerMillisecond);
  second.arrival_time_ns = 500 * kNanosecondsPerMillisecond;
  second.sequence = 0U;
  second.uptime_ms = 0U;
  core.ObserveHeartbeat(second);
  core.ObserveTelemetry(QualificationStream::kMotorState, StampAt(0), 0, false);

  const auto summary = core.Finish(kTestDuration);
  Expect(!HasFailure(summary.failure_mask,
                     QualificationFailure::kSequenceRegression),
         "uint32 sequence wrap is forward progress");
  Expect(!HasFailure(summary.failure_mask,
                     QualificationFailure::kUptimeRegression),
         "uint32 uptime wrap is forward progress");
  Expect(
      HasFailure(summary.failure_mask, QualificationFailure::kInvalidTelemetry),
      "adapter-reported invalid telemetry fails qualification");
}

void TestTransportLimitTimestampRegressionAndReadyLoss() {
  QualificationMonitorCore core(QualificationConfiguration{});
  ObserveAllPublishers(&core);

  HeartbeatObservation heartbeat;
  heartbeat.stamp = StampAt(0);
  heartbeat.arrival_time_ns = 0;
  heartbeat.sequence = 1U;
  heartbeat.uptime_ms = 1U;
  heartbeat.agent_session_id = kSessionId;
  heartbeat.state = 1U;
  heartbeat.time_synchronized = true;
  core.ObserveHeartbeat(heartbeat);
  heartbeat.stamp = StampAt(500 * kNanosecondsPerMillisecond);
  heartbeat.arrival_time_ns = 500 * kNanosecondsPerMillisecond;
  heartbeat.sequence = 2U;
  heartbeat.uptime_ms = 501U;
  heartbeat.state = 2U;
  core.ObserveHeartbeat(heartbeat);

  DiagnosticsObservation first = DiagnosticsAt(0);
  core.ObserveDiagnostics(first);
  DiagnosticsObservation second = DiagnosticsAt(kNanosecondsPerSecond);
  second.transport_rx_bytes = 70000U;
  second.transport_tx_bytes = 0U;
  second.monotonic_counters[0] = second.transport_rx_bytes;
  second.monotonic_counters[1] = second.transport_tx_bytes;
  core.ObserveDiagnostics(second);

  core.ObserveTelemetry(QualificationStream::kMotorState,
                        StampAt(kNanosecondsPerSecond), 0, true);
  core.ObserveTelemetry(QualificationStream::kMotorState, StampAt(0),
                        20 * kNanosecondsPerMillisecond, true);

  const auto summary = core.Finish(kTestDuration);
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kTransportRateExceeded),
         "the strict 70 kB/s transport boundary fails qualification");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kTimestampRegression),
         "nonzero per-stream timestamp regression fails qualification");
  Expect(HasFailure(summary.failure_mask,
                    QualificationFailure::kHeartbeatNotReady),
         "READY-to-DEGRADED transition fails qualification");
}

}  // namespace

int main() {
  TestPassingProductionRateRun();
  TestBatteryContractBoundaries();
  TestTransportIntervalCannotHideInWholeRunAverage();
  TestMotorCommandAgeEvidenceBoundaries();
  TestMotorCommandAcceptanceCorrelationBoundaries();
  TestMotorCommandEvidenceBaselinePublisherAndRegression();
  TestPassingImuCharacterizationRun();
  TestImuCharacterizationRejectsAdjacentStates();
  TestPublisherAndRateFailures();
  TestSessionStampAndProgressFailures();
  TestDiagnosticChangesAndRegression();
  TestInvalidTelemetryAndSerialWrap();
  TestTransportLimitTimestampRegressionAndReadyLoss();
  if (g_failures != 0) {
    std::cerr << g_failures << " qualification monitor test(s) failed\n";
    return 1;
  }
  std::cout << "qualification monitor core tests passed\n";
  return 0;
}
