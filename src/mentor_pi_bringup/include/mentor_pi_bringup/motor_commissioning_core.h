// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_CORE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_CORE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace mentor_pi_bringup {

inline constexpr std::string_view kMotorCommissioningAcknowledgement =
    "MOTORS_RAISED_CURRENT_LIMITED";
inline constexpr std::int64_t kMotorCommissioningPeriodMs = 50;

struct MotorCommissioningConfiguration {
  std::string acknowledgement;
  std::uint8_t motor_id = 0U;
  float target_rps = 0.0F;
  std::uint32_t duration_ms = 0U;
  std::int64_t start_time_ms = 0;
};

enum class MotorCommissioningPhase : std::uint8_t {
  kAwaitingPrerequisites = 0,
  kPreStop,
  kDrive,
  kPostStop,
  kSucceeded,
  kFailed,
};

enum class MotorCommissioningFailure : std::uint8_t {
  kNone = 0,
  kInvalidAcknowledgement,
  kInvalidMotorId,
  kInvalidTarget,
  kInvalidDuration,
  kPrerequisiteTimeout,
  kMotionAuthorizationLost,
  kHeartbeatLost,
  kHeartbeatNotReady,
  kSessionInvalid,
  kSessionChanged,
  kMcuResetDetected,
  kDiagnosticsLost,
  kDiagnosticsNotActive,
  kDiagnosticsSessionMismatch,
  kMotorStateLost,
  kMotorStateInvalid,
  kMotorStateUnexpectedTarget,
  kMotorWatchdogTriggered,
  kDriveCadenceLost,
  kMotorOverspeed,
  kMotorWrongDirection,
  kUnselectedMotorMotion,
  kCommandPublisherConflict,
  kCommandRejected,
  kPreStopNotConfirmed,
  kTargetNotObserved,
  kPhysicalResponseNotObserved,
  kPostStopNotConfirmed,
  kInterrupted,
};

const char* MotorCommissioningFailureName(MotorCommissioningFailure failure);

MotorCommissioningFailure ValidateMotorCommissioningConfiguration(
    const MotorCommissioningConfiguration& configuration);

struct CommissioningMotionAuthorizationObservation {
  std::int64_t arrival_time_ms = 0;
  std::uint32_t configuration_generation = 0U;
  std::uint32_t agent_session_id = 0U;
};

struct CommissioningHeartbeatObservation {
  std::int64_t arrival_time_ms = 0;
  std::uint32_t sequence = 0U;
  std::uint32_t uptime_ms = 0U;
  std::uint32_t agent_session_id = 0U;
  std::uint8_t state = 0U;
};

struct CommissioningDiagnosticsObservation {
  std::int64_t arrival_time_ms = 0;
  std::uint32_t uptime_ms = 0U;
  std::uint32_t session_generation = 0U;
  std::uint32_t command_rejections = 0U;
  std::array<std::uint32_t, 4> motor_command_rejections{};
  std::array<std::uint32_t, 4> motor_lease_expiries{};
  std::uint32_t motor_watchdog_trips = 0U;
  std::uint8_t session_state = 0U;
};

struct CommissioningMotorStateObservation {
  std::int64_t arrival_time_ms = 0;
  std::array<float, 4> target_rps{};
  std::array<float, 4> measured_rps{};
  std::array<std::int64_t, 4> encoder_count{};
  std::uint8_t watchdog_stop_mask = 0U;
};

enum class MotorCommissioningCommand : std::uint8_t {
  kNone = 0,
  kStop,
  kDrive,
};

struct MotorCommissioningAction {
  MotorCommissioningCommand command = MotorCommissioningCommand::kNone;
};

struct MotorCommissioningSummary {
  bool passed = false;
  MotorCommissioningFailure failure = MotorCommissioningFailure::kNone;
  MotorCommissioningPhase phase =
      MotorCommissioningPhase::kAwaitingPrerequisites;
  std::uint32_t agent_session_id = 0U;
  std::uint8_t motor_id = 0U;
  float target_rps = 0.0F;
  std::uint32_t duration_ms = 0U;
  bool target_observed = false;
  bool physical_response_observed = false;
  bool zero_confirmed = false;
  std::int64_t encoder_delta = 0;
  float peak_absolute_measured_rps = 0.0F;
  float final_measured_rps = 0.0F;
  std::uint32_t pre_stop_commands = 0U;
  std::uint32_t drive_commands = 0U;
  std::uint32_t post_stop_commands = 0U;
};

class MotorCommissioningCore final {
 public:
  explicit MotorCommissioningCore(
      MotorCommissioningConfiguration configuration);

  void ObserveMotionAuthorizationPublisher(bool valid);
  void ObserveMotionAuthorization(
      const CommissioningMotionAuthorizationObservation& observation);
  void ObserveHeartbeat(const CommissioningHeartbeatObservation& observation);
  void ObserveDiagnostics(
      const CommissioningDiagnosticsObservation& observation);
  void ObserveMotorState(const CommissioningMotorStateObservation& observation);
  void ObserveCommandPublisherConflict(bool conflict);

  MotorCommissioningAction Tick(std::int64_t now_ms);
  void RecordCommandPublished(MotorCommissioningCommand command,
                              std::int64_t publish_time_ms);
  void RequestAbort(std::int64_t now_ms);

  bool complete() const;
  MotorCommissioningPhase phase() const;
  MotorCommissioningSummary summary() const;

 private:
  static constexpr std::int64_t kPrerequisiteTimeoutMs = 5000;
  static constexpr std::int64_t kHeartbeatFreshnessMs = 1000;
  static constexpr std::int64_t kDiagnosticsFreshnessMs = 1500;
  static constexpr std::int64_t kMotorStateFreshnessMs = 200;
  static constexpr std::int64_t kStopPhaseDurationMs = 500;
  static constexpr std::uint32_t kStopCommandCount = 10U;
  static constexpr std::int64_t kMaximumDriveCommandGapMs = 100;
  static constexpr float kMinimumTargetRps = 0.01F;
  static constexpr float kMaximumMeasuredRps = 0.50F;
  static constexpr float kMinimumResponseRps = 0.002F;
  static constexpr float kUnselectedMotionRps = 0.02F;
  static constexpr std::int64_t kMinimumEncoderResponseTicks = 2;

  bool PrerequisitesSatisfied(std::int64_t now_ms) const;
  MotorCommissioningFailure RuntimeFailure(std::int64_t now_ms) const;
  MotorCommissioningFailure ValidateMotorStateForPhase() const;
  MotorCommissioningFailure ValidatePhysicalMotion() const;
  void LockPrerequisites(std::int64_t now_ms);
  void BeginPostStop(MotorCommissioningFailure failure, std::int64_t now_ms);
  void FinishPostStop();
  void RememberFailure(MotorCommissioningFailure failure);
  static bool IsFresh(std::int64_t arrival_time_ms, std::int64_t now_ms,
                      std::int64_t maximum_age_ms);
  bool MotorTargetsAreZero() const;
  bool MotorsAreStationary() const;
  bool MotorStateIsFinite() const;
  bool SelectedTargetMatches(float expected) const;
  bool SelectedResponseDirectionMatches() const;
  float ResponseThresholdRps() const;
  std::int64_t EncoderDelta(std::size_t index) const;
  static bool SerialNumberRegressed(std::uint32_t current,
                                    std::uint32_t previous);
  std::size_t SelectedMotorIndex() const;

  MotorCommissioningConfiguration configuration_;
  MotorCommissioningPhase phase_ =
      MotorCommissioningPhase::kAwaitingPrerequisites;
  MotorCommissioningFailure failure_ = MotorCommissioningFailure::kNone;
  std::int64_t phase_start_time_ms_ = 0;

  bool motion_authorization_publisher_valid_ = false;
  bool motion_authorization_seen_ = false;
  CommissioningMotionAuthorizationObservation motion_authorization_{};

  bool heartbeat_seen_ = false;
  CommissioningHeartbeatObservation heartbeat_{};
  bool heartbeat_discontinuity_ = false;
  bool diagnostics_seen_ = false;
  CommissioningDiagnosticsObservation diagnostics_{};
  bool diagnostics_discontinuity_ = false;
  bool motor_state_seen_ = false;
  CommissioningMotorStateObservation motor_state_{};
  bool command_publisher_conflict_ = false;

  std::uint32_t locked_session_id_ = 0U;
  std::uint32_t locked_configuration_generation_ = 0U;
  std::uint32_t baseline_command_rejections_ = 0U;
  std::uint32_t baseline_motor_rejections_ = 0U;
  std::array<std::uint32_t, 4> baseline_motor_lease_expiries_{};
  std::uint32_t baseline_motor_watchdog_trips_ = 0U;
  bool encoder_baseline_seen_ = false;
  std::array<std::int64_t, 4> encoder_baseline_{};
  std::array<std::int64_t, 4> latest_encoder_{};
  float peak_absolute_measured_rps_ = 0.0F;
  float final_measured_rps_ = 0.0F;

  bool target_observed_ = false;
  bool physical_response_observed_ = false;
  bool pre_stop_zero_confirmed_ = false;
  bool post_stop_zero_confirmed_ = false;
  bool post_stop_zero_seen_ = false;
  bool abort_requested_ = false;
  std::int64_t last_drive_publish_time_ms_ = 0;
  std::uint32_t pre_stop_commands_ = 0U;
  std::uint32_t drive_commands_ = 0U;
  std::uint32_t post_stop_commands_ = 0U;
};

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_CORE_H_
