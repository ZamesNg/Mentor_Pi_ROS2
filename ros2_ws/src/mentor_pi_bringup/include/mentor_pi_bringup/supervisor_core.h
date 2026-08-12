// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__SUPERVISOR_CORE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__SUPERVISOR_CORE_H_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "mentor_pi_bringup/configuration.h"

namespace mentor_pi_bringup {

enum class HeartbeatState : std::uint8_t {
  kBooting = 0,
  kReady = 1,
  kDegraded = 2,
  kFault = 3,
};

struct HeartbeatSample {
  std::uint32_t uptime_ms = 0;
  std::uint32_t agent_session_id = 0;
  HeartbeatState state = HeartbeatState::kBooting;
};

enum class ApplyOperation : std::uint8_t {
  kMotorModel = 0,
  kMotorAdrc = 1,
  kPwmServoOffsets = 2,
  kBatteryThreshold = 3,
};

enum class ResultCode : std::uint8_t {
  kOk = 0,
  kInvalidArgument = 1,
  kOutOfRange = 2,
  kBusy = 3,
  kTimeout = 4,
  kIoError = 5,
  kUnsupported = 6,
  kPartial = 7,
};

enum class SupervisorPhase : std::uint8_t {
  kDisconnected,
  kWaitingForHeartbeat,
  kWaitingForReady,
  kApplyingMotorModel,
  kApplyingMotorAdrc,
  kApplyingPwmServoOffsets,
  kApplyingBatteryThreshold,
  kReady,
  kRejected,
};

struct RequestToken {
  std::uint64_t configuration_generation = 0;
  std::uint32_t agent_session_id = 0;
  std::uint64_t request_serial = 0;
  ApplyOperation operation = ApplyOperation::kMotorModel;
  std::uint8_t attempt = 0;

  bool operator==(const RequestToken& other) const;
};

struct ServiceCall {
  RequestToken token{};
  DeploymentConfiguration configuration{};
  std::chrono::steady_clock::time_point deadline;
};

enum class ResponseDisposition : std::uint8_t {
  kAccepted,
  kStale,
};

struct SupervisorStatus {
  SupervisorPhase phase = SupervisorPhase::kDisconnected;
  bool motion_enabled = false;
  bool controller_present = false;
  std::uint64_t configuration_generation = 0;
  std::uint32_t agent_session_id = 0;
  ApplyOperation operation = ApplyOperation::kMotorModel;
  std::uint8_t attempt = 0;
  ResultCode last_result = ResultCode::kOk;
  std::uint16_t last_detail = 0;
  std::string description = "controller not discovered";
};

// Pure C++ state machine for HOST-002/HOST-003. The ROS adapter owns discovery,
// timers, and clients; this type owns all generation, ordering, timeout, and
// stale-response decisions.
class SupervisorCore {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit SupervisorCore(DeploymentConfiguration configuration);

  void OnControllerPresence(bool present, TimePoint now);
  void OnHeartbeat(const HeartbeatSample& heartbeat, TimePoint now);

  // Returns at most one new service attempt. Calling Tick also expires the
  // current attempt at its inclusive 100 ms deadline.
  std::optional<ServiceCall> Tick(TimePoint now);

  ResponseDisposition OnServiceResult(const RequestToken& token,
                                      ResultCode result, std::uint16_t detail,
                                      TimePoint now);

  bool IsCurrentToken(const RequestToken& token) const;
  const SupervisorStatus& status() const { return status_; }
  const DeploymentConfiguration& configuration() const {
    return configuration_;
  }

 private:
  static constexpr std::chrono::milliseconds kResponseTimeout{100};
  static constexpr std::uint8_t kMaximumAttempts = 4;

  void BeginGeneration(const char* reason, TimePoint now);
  void SetSessionBaseline(const HeartbeatSample& heartbeat);
  void StartApplying(TimePoint now);
  void SetOperation(ApplyOperation operation, TimePoint now);
  void ExpireRequestIfDue(TimePoint now);
  void RetryOrReject(ResultCode result, std::uint16_t detail, TimePoint now,
                     const char* reason);
  void CompleteOperation(TimePoint now);
  void Reject(ResultCode result, std::uint16_t detail,
              const std::string& reason);
  void InvalidateRequest();
  static bool IsReadyState(HeartbeatState state);
  bool UptimeRegressed(std::uint32_t new_uptime_ms) const;

  DeploymentConfiguration configuration_;
  SupervisorStatus status_;
  bool has_session_baseline_ = false;
  bool ready_heartbeat_seen_ = false;
  bool configuration_started_ = false;
  bool blocked_for_generation_ = false;
  std::uint32_t last_uptime_ms_ = 0;
  std::uint8_t attempts_ = 0;
  std::uint64_t request_serial_ = 0;
  TimePoint next_attempt_time_;
  std::optional<ServiceCall> in_flight_;
};

const char* ApplyOperationName(ApplyOperation operation);
const char* SupervisorPhaseName(SupervisorPhase phase);
const char* ResultCodeName(ResultCode result);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__SUPERVISOR_CORE_H_
