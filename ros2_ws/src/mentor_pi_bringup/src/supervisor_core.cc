// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/supervisor_core.h"

#include <array>
#include <limits>
#include <sstream>

namespace mentor_pi_bringup {
namespace {

constexpr std::array<std::chrono::milliseconds, 3> kRetryBackoff{
    std::chrono::milliseconds{100}, std::chrono::milliseconds{200},
    std::chrono::milliseconds{400}};

SupervisorPhase PhaseForOperation(ApplyOperation operation) {
  switch (operation) {
    case ApplyOperation::kMotorModel:
      return SupervisorPhase::kApplyingMotorModel;
    case ApplyOperation::kMotorAdrc:
      return SupervisorPhase::kApplyingMotorAdrc;
    case ApplyOperation::kPwmServoOffsets:
      return SupervisorPhase::kApplyingPwmServoOffsets;
    case ApplyOperation::kBatteryThreshold:
      return SupervisorPhase::kApplyingBatteryThreshold;
  }
  return SupervisorPhase::kRejected;
}

bool IsRetryable(ResultCode result) {
  return result == ResultCode::kBusy || result == ResultCode::kTimeout;
}

}  // namespace

bool RequestToken::operator==(const RequestToken& other) const {
  return configuration_generation == other.configuration_generation &&
         agent_session_id == other.agent_session_id &&
         request_serial == other.request_serial &&
         operation == other.operation && attempt == other.attempt;
}

SupervisorCore::SupervisorCore(DeploymentConfiguration configuration)
    : configuration_(configuration) {}

void SupervisorCore::OnControllerPresence(bool present, TimePoint now) {
  if (present == status_.controller_present) {
    return;
  }

  if (!present) {
    status_.controller_present = false;
    status_.motion_enabled = false;
    status_.phase = SupervisorPhase::kDisconnected;
    status_.description = "controller heartbeat/graph disappeared";
    has_session_baseline_ = false;
    ready_heartbeat_seen_ = false;
    configuration_started_ = false;
    InvalidateRequest();
    return;
  }

  status_.controller_present = true;
  BeginGeneration(status_.configuration_generation == 0
                      ? "first controller discovery"
                      : "controller graph/heartbeat reappeared",
                  now);
}

void SupervisorCore::OnHeartbeat(const HeartbeatSample& heartbeat,
                                 TimePoint now) {
  if (!status_.controller_present) {
    status_.controller_present = true;
    BeginGeneration(status_.configuration_generation == 0
                        ? "first heartbeat discovery"
                        : "heartbeat reappeared",
                    now);
  }

  if (heartbeat.agent_session_id == 0) {
    status_.motion_enabled = false;
    if (!blocked_for_generation_) {
      status_.phase = SupervisorPhase::kWaitingForHeartbeat;
      status_.description = "heartbeat has reserved agent_session_id zero";
    }
    ready_heartbeat_seen_ = false;
    configuration_started_ = false;
    InvalidateRequest();
    return;
  }

  if (!has_session_baseline_) {
    SetSessionBaseline(heartbeat);
  } else if (heartbeat.agent_session_id != status_.agent_session_id) {
    BeginGeneration("Agent session ID changed", now);
    SetSessionBaseline(heartbeat);
  } else if (UptimeRegressed(heartbeat.uptime_ms)) {
    BeginGeneration("MCU uptime regressed", now);
    SetSessionBaseline(heartbeat);
  } else {
    last_uptime_ms_ = heartbeat.uptime_ms;
  }

  if (!IsReadyState(heartbeat.state)) {
    if (!blocked_for_generation_) {
      status_.phase = SupervisorPhase::kWaitingForReady;
      status_.description = "waiting for READY or DEGRADED heartbeat";
    }
    status_.motion_enabled = false;
    ready_heartbeat_seen_ = false;
    configuration_started_ = false;
    InvalidateRequest();
    return;
  }

  ready_heartbeat_seen_ = true;
  if (!configuration_started_ && !blocked_for_generation_) {
    StartApplying(now);
  }
}

std::optional<ServiceCall> SupervisorCore::Tick(TimePoint now) {
  ExpireRequestIfDue(now);
  if (!status_.controller_present || !has_session_baseline_ ||
      !ready_heartbeat_seen_ || blocked_for_generation_ ||
      status_.motion_enabled || in_flight_.has_value() ||
      !configuration_started_ || now < next_attempt_time_) {
    return std::nullopt;
  }

  ++attempts_;
  ++request_serial_;
  RequestToken token;
  token.configuration_generation = status_.configuration_generation;
  token.agent_session_id = status_.agent_session_id;
  token.request_serial = request_serial_;
  token.operation = status_.operation;
  token.attempt = attempts_;

  ServiceCall call;
  call.token = token;
  call.configuration = configuration_;
  call.deadline = now + kResponseTimeout;
  in_flight_ = call;

  status_.attempt = attempts_;
  status_.phase = PhaseForOperation(status_.operation);
  std::ostringstream description;
  description << "attempt " << static_cast<unsigned int>(attempts_) << "/"
              << static_cast<unsigned int>(kMaximumAttempts) << " for "
              << ApplyOperationName(status_.operation);
  status_.description = description.str();
  return call;
}

ResponseDisposition SupervisorCore::OnServiceResult(const RequestToken& token,
                                                    ResultCode result,
                                                    std::uint16_t detail,
                                                    TimePoint now) {
  ExpireRequestIfDue(now);
  if (!IsCurrentToken(token)) {
    return ResponseDisposition::kStale;
  }

  in_flight_.reset();
  status_.last_result = result;
  status_.last_detail = detail;
  if (result == ResultCode::kOk) {
    CompleteOperation(now);
  } else if (IsRetryable(result)) {
    RetryOrReject(result, detail, now, ResultCodeName(result));
  } else {
    std::ostringstream description;
    description << ApplyOperationName(token.operation) << " returned "
                << ResultCodeName(result) << " (detail " << detail << ")";
    Reject(result, detail, description.str());
  }
  return ResponseDisposition::kAccepted;
}

bool SupervisorCore::IsCurrentToken(const RequestToken& token) const {
  return in_flight_.has_value() && in_flight_->token == token &&
         token.configuration_generation == status_.configuration_generation &&
         token.agent_session_id == status_.agent_session_id;
}

void SupervisorCore::BeginGeneration(const char* reason, TimePoint now) {
  if (status_.configuration_generation ==
      std::numeric_limits<std::uint64_t>::max()) {
    status_.configuration_generation = 1;
  } else {
    ++status_.configuration_generation;
  }
  status_.motion_enabled = false;
  status_.agent_session_id = 0;
  status_.phase = SupervisorPhase::kWaitingForHeartbeat;
  status_.operation = ApplyOperation::kMotorModel;
  status_.attempt = 0;
  status_.last_result = ResultCode::kOk;
  status_.last_detail = 0;
  status_.description = reason;
  has_session_baseline_ = false;
  ready_heartbeat_seen_ = false;
  configuration_started_ = false;
  blocked_for_generation_ = false;
  attempts_ = 0;
  next_attempt_time_ = now;
  InvalidateRequest();
}

void SupervisorCore::SetSessionBaseline(const HeartbeatSample& heartbeat) {
  has_session_baseline_ = true;
  status_.agent_session_id = heartbeat.agent_session_id;
  last_uptime_ms_ = heartbeat.uptime_ms;
}

void SupervisorCore::StartApplying(TimePoint now) {
  configuration_started_ = true;
  SetOperation(ApplyOperation::kMotorModel, now);
}

void SupervisorCore::SetOperation(ApplyOperation operation, TimePoint now) {
  status_.operation = operation;
  status_.phase = PhaseForOperation(operation);
  status_.attempt = 0;
  status_.description = std::string("pending ") + ApplyOperationName(operation);
  attempts_ = 0;
  next_attempt_time_ = now;
  InvalidateRequest();
}

void SupervisorCore::ExpireRequestIfDue(TimePoint now) {
  if (!in_flight_.has_value() || now < in_flight_->deadline) {
    return;
  }
  const ApplyOperation operation = in_flight_->token.operation;
  in_flight_.reset();
  status_.operation = operation;
  RetryOrReject(ResultCode::kTimeout, 0, now, "client timeout");
}

void SupervisorCore::RetryOrReject(ResultCode result, std::uint16_t detail,
                                   TimePoint now, const char* reason) {
  status_.last_result = result;
  status_.last_detail = detail;
  if (attempts_ >= kMaximumAttempts) {
    std::ostringstream description;
    description << ApplyOperationName(status_.operation)
                << " exhausted four attempts after " << reason;
    Reject(result, detail, description.str());
    return;
  }

  const std::size_t backoff_index = static_cast<std::size_t>(attempts_ - 1U);
  next_attempt_time_ = now + kRetryBackoff[backoff_index];
  status_.phase = PhaseForOperation(status_.operation);
  std::ostringstream description;
  description << ApplyOperationName(status_.operation) << " " << reason
              << "; retry after " << kRetryBackoff[backoff_index].count()
              << " ms";
  status_.description = description.str();
}

void SupervisorCore::CompleteOperation(TimePoint now) {
  switch (status_.operation) {
    case ApplyOperation::kMotorModel:
      SetOperation(ApplyOperation::kMotorAdrc, now);
      return;
    case ApplyOperation::kMotorAdrc:
      SetOperation(ApplyOperation::kPwmServoOffsets, now);
      return;
    case ApplyOperation::kPwmServoOffsets:
      SetOperation(ApplyOperation::kBatteryThreshold, now);
      return;
    case ApplyOperation::kBatteryThreshold:
      status_.motion_enabled = true;
      status_.phase = SupervisorPhase::kReady;
      status_.attempt = attempts_;
      status_.description = "deployment configuration applied";
      attempts_ = 0;
      return;
  }
}

void SupervisorCore::Reject(ResultCode result, std::uint16_t detail,
                            const std::string& reason) {
  status_.motion_enabled = false;
  status_.phase = SupervisorPhase::kRejected;
  status_.last_result = result;
  status_.last_detail = detail;
  status_.description = reason;
  blocked_for_generation_ = true;
  InvalidateRequest();
}

void SupervisorCore::InvalidateRequest() { in_flight_.reset(); }

bool SupervisorCore::IsReadyState(HeartbeatState state) {
  return state == HeartbeatState::kReady || state == HeartbeatState::kDegraded;
}

bool SupervisorCore::UptimeRegressed(std::uint32_t new_uptime_ms) const {
  const std::uint32_t delta = new_uptime_ms - last_uptime_ms_;
  return delta >= UINT32_C(0x80000000);
}

const char* ApplyOperationName(ApplyOperation operation) {
  switch (operation) {
    case ApplyOperation::kMotorModel:
      return "motors/set_model";
    case ApplyOperation::kMotorAdrc:
      return "motors/set_adrc";
    case ApplyOperation::kPwmServoOffsets:
      return "pwm_servos/set_offsets";
    case ApplyOperation::kBatteryThreshold:
      return "battery/set_low_threshold";
  }
  return "unknown operation";
}

const char* SupervisorPhaseName(SupervisorPhase phase) {
  switch (phase) {
    case SupervisorPhase::kDisconnected:
      return "DISCONNECTED";
    case SupervisorPhase::kWaitingForHeartbeat:
      return "WAITING_FOR_HEARTBEAT";
    case SupervisorPhase::kWaitingForReady:
      return "WAITING_FOR_READY";
    case SupervisorPhase::kApplyingMotorModel:
      return "APPLYING_MOTOR_MODEL";
    case SupervisorPhase::kApplyingMotorAdrc:
      return "APPLYING_MOTOR_ADRC";
    case SupervisorPhase::kApplyingPwmServoOffsets:
      return "APPLYING_PWM_SERVO_OFFSETS";
    case SupervisorPhase::kApplyingBatteryThreshold:
      return "APPLYING_BATTERY_THRESHOLD";
    case SupervisorPhase::kReady:
      return "READY";
    case SupervisorPhase::kRejected:
      return "REJECTED";
  }
  return "UNKNOWN";
}

const char* ResultCodeName(ResultCode result) {
  switch (result) {
    case ResultCode::kOk:
      return "OK";
    case ResultCode::kInvalidArgument:
      return "INVALID_ARGUMENT";
    case ResultCode::kOutOfRange:
      return "OUT_OF_RANGE";
    case ResultCode::kBusy:
      return "BUSY";
    case ResultCode::kTimeout:
      return "TIMEOUT";
    case ResultCode::kIoError:
      return "IO_ERROR";
    case ResultCode::kUnsupported:
      return "UNSUPPORTED";
    case ResultCode::kPartial:
      return "PARTIAL";
  }
  return "UNKNOWN_RESULT";
}

}  // namespace mentor_pi_bringup
