// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/supervisor_core.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace {

// Session IDs, protocol details, and timeline boundaries are intentionally
// literal test vectors. Naming every point would make the state-machine
// sequence harder to audit against HOST-002/HOST-003.
// NOLINTBEGIN(readability-magic-numbers)

using namespace std::chrono_literals;
using mentor_pi_bringup::ApplyOperation;
using mentor_pi_bringup::ApplyOperationName;
using mentor_pi_bringup::DeploymentConfiguration;
using mentor_pi_bringup::HeartbeatSample;
using mentor_pi_bringup::HeartbeatState;
using mentor_pi_bringup::ResponseDisposition;
using mentor_pi_bringup::ResultCode;
using mentor_pi_bringup::ResultCodeName;
using mentor_pi_bringup::ServiceCall;
using mentor_pi_bringup::SupervisorCore;
using mentor_pi_bringup::SupervisorPhase;
using mentor_pi_bringup::SupervisorPhaseName;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

SupervisorCore::TimePoint At(std::chrono::milliseconds milliseconds) {
  return SupervisorCore::TimePoint{} + milliseconds;
}

// NOLINTNEXTLINE
HeartbeatSample Heartbeat(std::uint32_t session_id, std::uint32_t uptime_ms,
                          HeartbeatState state = HeartbeatState::kReady) {
  HeartbeatSample heartbeat;
  heartbeat.agent_session_id = session_id;
  heartbeat.uptime_ms = uptime_ms;
  heartbeat.state = state;
  return heartbeat;
}

ServiceCall RequireCall(SupervisorCore* core, SupervisorCore::TimePoint now,
                        ApplyOperation operation, const std::string& context) {
  const std::optional<ServiceCall> call = core->Tick(now);
  Expect(call.has_value(), context + ": expected a service call");
  if (!call.has_value()) {
    return ServiceCall{};
  }
  Expect(call->token.operation == operation,
         context + ": unexpected operation order");
  return *call;
}

void TestOrderedApplication() {
  DeploymentConfiguration configuration;
  configuration.pwm_servo_offsets_us = {-100, -1, 1, 100};
  configuration.battery_low_threshold_mv = 20000;
  SupervisorCore core(configuration);
  core.OnHeartbeat(Heartbeat(7, 100), At(0ms));
  Expect(core.status().configuration_generation == 1,
         "first heartbeat creates generation one");
  Expect(!core.status().motion_enabled,
         "motion starts disabled before services complete");

  auto call =
      RequireCall(&core, At(0ms), ApplyOperation::kMotorModel, "motor model");
  Expect(call.configuration == configuration,
         "service call carries immutable validated configuration");
  core.OnServiceResult(call.token, ResultCode::kOk, 0, At(1ms));
  Expect(!core.status().motion_enabled,
         "motor model alone cannot enable motion");

  call = RequireCall(&core, At(1ms), ApplyOperation::kMotorAdrc, "motor LADRC");
  core.OnServiceResult(call.token, ResultCode::kOk, 0, At(2ms));
  Expect(!core.status().motion_enabled,
         "motor LADRC alone cannot enable motion");

  call = RequireCall(&core, At(2ms), ApplyOperation::kPwmServoOffsets,
                     "PWM offsets");
  core.OnServiceResult(call.token, ResultCode::kOk, 0, At(3ms));
  call = RequireCall(&core, At(3ms), ApplyOperation::kBatteryThreshold,
                     "battery threshold");
  core.OnServiceResult(call.token, ResultCode::kOk, 0, At(4ms));
  Expect(core.status().motion_enabled,
         "all four ordered OK responses enable motion");
  Expect(core.status().phase == SupervisorPhase::kReady,
         "successful configuration reports READY");
}

void TestRetryPolicyAndTimeout() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnHeartbeat(Heartbeat(3, 10), At(0ms));
  auto call = RequireCall(&core, At(0ms), ApplyOperation::kMotorModel,
                          "retry attempt 1");
  core.OnServiceResult(call.token, ResultCode::kBusy, 0, At(10ms));
  Expect(!core.Tick(At(109ms)).has_value(),
         "BUSY retry must wait the full 100 ms backoff");
  call = RequireCall(&core, At(110ms), ApplyOperation::kMotorModel,
                     "retry attempt 2");
  Expect(call.token.attempt == 2, "second attempt number");

  core.OnServiceResult(call.token, ResultCode::kTimeout, 0, At(120ms));
  Expect(!core.Tick(At(319ms)).has_value(),
         "returned TIMEOUT retry waits 200 ms");
  call = RequireCall(&core, At(320ms), ApplyOperation::kMotorModel,
                     "retry attempt 3");

  Expect(!core.Tick(At(419ms)).has_value(),
         "client response timeout is inclusive at 100 ms");
  Expect(!core.Tick(At(420ms)).has_value(),
         "client timeout starts the 400 ms backoff");
  Expect(!core.Tick(At(819ms)).has_value(),
         "fourth attempt cannot start before 400 ms backoff");
  call = RequireCall(&core, At(820ms), ApplyOperation::kMotorModel,
                     "retry attempt 4");
  Expect(call.token.attempt == 4, "fourth attempt number");
  core.OnServiceResult(call.token, ResultCode::kBusy, 0, At(821ms));
  Expect(core.status().phase == SupervisorPhase::kRejected,
         "fourth retryable failure exhausts generation");
  Expect(!core.status().motion_enabled,
         "retry exhaustion leaves motion disabled");
  Expect(!core.Tick(At(10s)).has_value(),
         "rejected generation does not retry without a new session");
}

void TestPermanentFailure() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnHeartbeat(Heartbeat(5, 10), At(0ms));
  const auto call = RequireCall(&core, At(0ms), ApplyOperation::kMotorModel,
                                "permanent failure");
  core.OnServiceResult(call.token, ResultCode::kOutOfRange, 12, At(1ms));
  Expect(core.status().phase == SupervisorPhase::kRejected,
         "non-retryable result is immediately permanent");
  Expect(core.status().last_detail == 12,
         "permanent failure detail is retained");
  Expect(!core.Tick(At(1h)).has_value(),
         "permanent result is attempted only once");
}

void TestStaleResponseAndSessionChange() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnHeartbeat(Heartbeat(10, 100), At(0ms));
  const auto old_call = RequireCall(&core, At(0ms), ApplyOperation::kMotorModel,
                                    "old session call");
  core.OnHeartbeat(Heartbeat(11, 1), At(10ms));
  Expect(core.status().configuration_generation == 2,
         "Agent session ID change creates a generation");
  Expect(core.OnServiceResult(old_call.token, ResultCode::kOk, 0, At(11ms)) ==
             ResponseDisposition::kStale,
         "old-session future must be ignored");
  Expect(!core.status().motion_enabled, "stale OK cannot open motion gate");
  const auto new_call = RequireCall(
      &core, At(11ms), ApplyOperation::kMotorModel, "new session call");
  Expect(new_call.token.configuration_generation == 2 &&
             new_call.token.agent_session_id == 11,
         "new future carries generation and Agent session tags");
}

void TestUptimeSerialArithmetic() {
  SupervisorCore core(DeploymentConfiguration{});
  constexpr std::uint32_t kNearWrap =
      std::numeric_limits<std::uint32_t>::max() - 5U;
  core.OnHeartbeat(Heartbeat(1, kNearWrap), At(0ms));
  const std::uint64_t initial_generation =
      core.status().configuration_generation;
  core.OnHeartbeat(Heartbeat(1, 3), At(1ms));
  Expect(core.status().configuration_generation == initial_generation,
         "normal uint32 uptime wrap is forward progress");
  core.OnHeartbeat(Heartbeat(1, 2), At(2ms));
  Expect(core.status().configuration_generation == initial_generation + 1U,
         "wrap-aware uptime regression creates a generation");

  const std::uint64_t regression_generation =
      core.status().configuration_generation;
  core.OnHeartbeat(Heartbeat(1, UINT32_C(0x80000002)), At(3ms));
  Expect(core.status().configuration_generation == regression_generation + 1U,
         "half-range delta is classified as regression");
}

void TestPresenceAndReadyGate() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnControllerPresence(true, At(0ms));
  Expect(core.status().configuration_generation == 1,
         "graph discovery starts one generation");
  core.OnHeartbeat(Heartbeat(4, 10, HeartbeatState::kBooting), At(1ms));
  Expect(core.status().configuration_generation == 1,
         "first heartbeat after graph discovery does not duplicate generation");
  Expect(!core.Tick(At(1ms)).has_value(),
         "BOOTING heartbeat cannot start configuration");
  core.OnHeartbeat(Heartbeat(4, 11, HeartbeatState::kDegraded), At(2ms));
  Expect(core.Tick(At(2ms)).has_value(),
         "DEGRADED heartbeat permits configuration");

  core.OnControllerPresence(false, At(3ms));
  Expect(!core.status().motion_enabled &&
             core.status().phase == SupervisorPhase::kDisconnected,
         "disappearance closes the host motion gate");
  core.OnControllerPresence(true, At(4ms));
  Expect(core.status().configuration_generation == 2,
         "graph reappearance starts exactly one new generation");
  core.OnHeartbeat(Heartbeat(4, 12), At(5ms));
  Expect(core.status().configuration_generation == 2,
         "heartbeat baseline after reappearance does not double-count");
}

void TestLateAtDeadline() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnHeartbeat(Heartbeat(8, 1), At(0ms));
  const auto call =
      RequireCall(&core, At(0ms), ApplyOperation::kMotorModel, "deadline call");
  Expect(core.OnServiceResult(call.token, ResultCode::kOk, 0, At(100ms)) ==
             ResponseDisposition::kStale,
         "response at inclusive 100 ms deadline is stale");
  Expect(!core.status().motion_enabled, "late OK cannot enable motion");
  Expect(!core.Tick(At(199ms)).has_value(),
         "late response timeout applies 100 ms retry backoff");
  const auto retry = RequireCall(&core, At(200ms), ApplyOperation::kMotorModel,
                                 "post-timeout retry");
  Expect(retry.token.request_serial != call.token.request_serial,
         "each attempt has an additional stale-response correlation token");
}

void TestReservedSessionIdStopsApplication() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnHeartbeat(Heartbeat(9, 1), At(0ms));
  const auto call = RequireCall(&core, At(0ms), ApplyOperation::kMotorModel,
                                "valid session before zero ID");
  core.OnHeartbeat(Heartbeat(0, 2), At(1ms));
  Expect(!core.status().motion_enabled,
         "reserved zero Agent session ID closes motion gate");
  Expect(core.OnServiceResult(call.token, ResultCode::kOk, 0, At(2ms)) ==
             ResponseDisposition::kStale,
         "zero-ID heartbeat invalidates an outstanding request");
  core.OnHeartbeat(Heartbeat(9, 3), At(3ms));
  Expect(core.Tick(At(3ms)).has_value(),
         "a later valid ready heartbeat restarts ordered application");
}

void TestStableDiagnosticNames() {
  const std::array<std::pair<ApplyOperation, std::string>, 4> operations{{
      {ApplyOperation::kMotorModel, "motors/set_model"},
      {ApplyOperation::kMotorAdrc, "motors/set_adrc"},
      {ApplyOperation::kPwmServoOffsets, "pwm_servos/set_offsets"},
      {ApplyOperation::kBatteryThreshold, "battery/set_low_threshold"},
  }};
  for (const auto& operation : operations) {
    Expect(ApplyOperationName(operation.first) == operation.second,
           "every apply operation has a stable diagnostic name");
  }
  Expect(std::string{ApplyOperationName(static_cast<ApplyOperation>(255U))} ==
             "unknown operation",
         "unknown apply operation has a defensive diagnostic name");

  const std::array<std::pair<SupervisorPhase, std::string>, 9> phases{{
      {SupervisorPhase::kDisconnected, "DISCONNECTED"},
      {SupervisorPhase::kWaitingForHeartbeat, "WAITING_FOR_HEARTBEAT"},
      {SupervisorPhase::kWaitingForReady, "WAITING_FOR_READY"},
      {SupervisorPhase::kApplyingMotorModel, "APPLYING_MOTOR_MODEL"},
      {SupervisorPhase::kApplyingMotorAdrc, "APPLYING_MOTOR_ADRC"},
      {SupervisorPhase::kApplyingPwmServoOffsets, "APPLYING_PWM_SERVO_OFFSETS"},
      {SupervisorPhase::kApplyingBatteryThreshold,
       "APPLYING_BATTERY_THRESHOLD"},
      {SupervisorPhase::kReady, "READY"},
      {SupervisorPhase::kRejected, "REJECTED"},
  }};
  for (const auto& phase : phases) {
    Expect(SupervisorPhaseName(phase.first) == phase.second,
           "every supervisor phase has a stable diagnostic name");
  }
  Expect(std::string{SupervisorPhaseName(static_cast<SupervisorPhase>(255U))} ==
             "UNKNOWN",
         "unknown supervisor phase has a defensive diagnostic name");

  const std::array<std::pair<ResultCode, std::string>, 8> results{{
      {ResultCode::kOk, "OK"},
      {ResultCode::kInvalidArgument, "INVALID_ARGUMENT"},
      {ResultCode::kOutOfRange, "OUT_OF_RANGE"},
      {ResultCode::kBusy, "BUSY"},
      {ResultCode::kTimeout, "TIMEOUT"},
      {ResultCode::kIoError, "IO_ERROR"},
      {ResultCode::kUnsupported, "UNSUPPORTED"},
      {ResultCode::kPartial, "PARTIAL"},
  }};
  for (const auto& result : results) {
    Expect(ResultCodeName(result.first) == result.second,
           "every result code has a stable diagnostic name");
  }
  Expect(std::string{ResultCodeName(static_cast<ResultCode>(255U))} ==
             "UNKNOWN_RESULT",
         "unknown result code has a defensive diagnostic name");
}

void TestDuplicatePresenceAndTokenMismatchAreNoOps() {
  SupervisorCore core(DeploymentConfiguration{});
  core.OnControllerPresence(false, At(0ms));
  Expect(core.status().configuration_generation == 0,
         "duplicate absent notification is a no-op");
  core.OnControllerPresence(true, At(1ms));
  core.OnControllerPresence(true, At(2ms));
  Expect(core.status().configuration_generation == 1,
         "duplicate present notification does not create another generation");
  core.OnHeartbeat(Heartbeat(2, 1), At(3ms));
  const auto call = RequireCall(&core, At(3ms), ApplyOperation::kMotorModel,
                                "token mismatch baseline");

  auto mismatched = call.token;
  ++mismatched.request_serial;
  Expect(core.OnServiceResult(mismatched, ResultCode::kOk, 0, At(4ms)) ==
             ResponseDisposition::kStale,
         "request serial mismatch is stale");
  Expect(core.OnServiceResult(call.token, ResultCode::kOk, 0, At(4ms)) ==
             ResponseDisposition::kAccepted,
         "original matching token remains valid after stale response");
}

// NOLINTEND(readability-magic-numbers)

}  // namespace

// NOLINTNEXTLINE
int main() {
  TestOrderedApplication();
  TestRetryPolicyAndTimeout();
  TestPermanentFailure();
  TestStaleResponseAndSessionChange();
  TestUptimeSerialArithmetic();
  TestPresenceAndReadyGate();
  TestLateAtDeadline();
  TestReservedSessionIdStopsApplication();
  TestStableDiagnosticNames();
  TestDuplicatePresenceAndTokenMismatchAreNoOps();
  if (g_failures != 0) {
    std::cerr << g_failures << " supervisor core tests failed\n";
    return 1;
  }
  std::cout << "supervisor core tests passed\n";
  return 0;
}
