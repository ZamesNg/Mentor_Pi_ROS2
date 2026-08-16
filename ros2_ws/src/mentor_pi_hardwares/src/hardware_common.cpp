#include "mentor_pi_hardwares/hardware_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mentor_pi_interfaces/motor_profile_contract.hpp"

namespace mentor_pi::hardware {
namespace {

double EffectiveAngle(double angle_rad,
                      const SteeringCalibration& calibration) {
  return calibration.inverted ? -angle_rad : angle_rad;
}

}  // namespace

double RpsToRadiansPerSecond(double rps) { return rps * kTwoPi; }

double EncoderCountToRadians(std::int64_t count,
                             std::uint32_t ticks_per_revolution) {
  if (ticks_per_revolution == 0U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(count) * kTwoPi /
         static_cast<double>(ticks_per_revolution);
}

std::optional<float> RadiansPerSecondToRps(double radians_per_second,
                                           double maximum_rps) {
  if (!std::isfinite(radians_per_second) || !std::isfinite(maximum_rps) ||
      maximum_rps <= 0.0) {
    return std::nullopt;
  }
  const double rps = radians_per_second / kTwoPi;
  if (std::fabs(rps) > maximum_rps ||
      std::fabs(rps) > static_cast<double>(std::numeric_limits<float>::max())) {
    return std::nullopt;
  }
  return static_cast<float>(rps);
}

std::optional<double> MotorMaximumRps(std::uint8_t model) {
  const auto* contract = mentor_pi_interfaces::FindMotorProfileContract(model);
  if (contract == nullptr) {
    return std::nullopt;
  }
  return static_cast<double>(contract->max_rps);
}

std::optional<std::uint32_t> MotorTicksPerRevolution(std::uint8_t model) {
  const auto* contract = mentor_pi_interfaces::FindMotorProfileContract(model);
  if (contract == nullptr) {
    return std::nullopt;
  }
  return contract->ticks_per_revolution;
}

const char* RecoveryReasonName(RecoveryReason reason) {
  switch (reason) {
    case RecoveryReason::kInitial:
      return "initial inhibition";
    case RecoveryReason::kFeedbackMissing:
      return "required feedback missing";
    case RecoveryReason::kFeedbackInvalid:
      return "required feedback invalid";
    case RecoveryReason::kFeedbackStale:
      return "required feedback stale";
    case RecoveryReason::kHeartbeatMissing:
      return "heartbeat missing";
    case RecoveryReason::kHeartbeatNotReady:
      return "heartbeat not ready";
    case RecoveryReason::kHeartbeatStale:
      return "heartbeat stale";
    case RecoveryReason::kAuthorizationPublisherInvalid:
      return "authorization publisher invalid";
    case RecoveryReason::kAuthorizationInvalid:
      return "authorization invalid";
    case RecoveryReason::kSessionChanged:
      return "Agent session changed";
    case RecoveryReason::kUptimeRegressed:
      return "MCU uptime regressed";
  }
  return "unknown";
}

bool ReconnectGate::Configure(std::uint8_t required_feedback_mask,
                              std::chrono::milliseconds motor_timeout,
                              std::chrono::milliseconds imu_timeout,
                              std::chrono::milliseconds pwm_timeout) {
  constexpr std::uint8_t kKnownFeedbackMask =
      FeedbackMask(FeedbackStream::kMotor) |
      FeedbackMask(FeedbackStream::kImu) | FeedbackMask(FeedbackStream::kPwm);
  if (required_feedback_mask == 0U ||
      (required_feedback_mask & ~kKnownFeedbackMask) != 0U ||
      motor_timeout.count() <= 0 || imu_timeout.count() <= 0 ||
      pwm_timeout.count() <= 0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  required_feedback_mask_ = required_feedback_mask;
  feedback_[FeedbackIndex(FeedbackStream::kMotor)].timeout = motor_timeout;
  feedback_[FeedbackIndex(FeedbackStream::kImu)].timeout = imu_timeout;
  feedback_[FeedbackIndex(FeedbackStream::kPwm)].timeout = pwm_timeout;
  ResetLocked(Clock::now());
  return true;
}

void ReconnectGate::Reset(TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  ResetLocked(now);
}

void ReconnectGate::ObserveHeartbeat(std::uint32_t session_id,
                                     std::uint32_t uptime_ms, bool ready,
                                     TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool session_changed =
      has_heartbeat_ && session_id != heartbeat_session_id_;
  const bool uptime_regressed =
      has_heartbeat_ && !session_changed &&
      (uptime_ms - heartbeat_uptime_ms_) >= UINT32_C(0x80000000);

  heartbeat_session_id_ = session_id;
  heartbeat_uptime_ms_ = uptime_ms;
  heartbeat_ready_ = ready && session_id != 0U;
  has_heartbeat_ = true;
  last_heartbeat_ = now;
  ++heartbeat_sequence_;

  if (session_changed) {
    EnterRecoveryLocked(RecoveryReason::kSessionChanged, true, true);
  } else if (uptime_regressed) {
    EnterRecoveryLocked(RecoveryReason::kUptimeRegressed, true, true);
  } else if (!heartbeat_ready_) {
    EnterRecoveryLocked(RecoveryReason::kHeartbeatNotReady, false);
  }
}

void ReconnectGate::ObserveAuthorization(std::uint64_t authorization,
                                         TimePoint) {
  std::lock_guard<std::mutex> lock(mutex_);
  authorization_ = authorization;
  const std::uint32_t generation =
      static_cast<std::uint32_t>(authorization_ >> 32U);
  const std::uint32_t session = static_cast<std::uint32_t>(authorization_);
  if (generation == 0U || session == 0U ||
      (has_heartbeat_ && session != heartbeat_session_id_)) {
    EnterRecoveryLocked(RecoveryReason::kAuthorizationInvalid, false);
  }
}

void ReconnectGate::ObserveFeedback(FeedbackStream stream, bool valid,
                                    TimePoint now) {
  const std::size_t index = FeedbackIndex(stream);
  if (index >= feedback_.size()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto& observation = feedback_[index];
  observation.seen = true;
  observation.valid = valid;
  observation.received_at = now;
  if (valid) {
    ++observation.valid_sequence;
  } else if ((required_feedback_mask_ & FeedbackMask(stream)) != 0U) {
    EnterRecoveryLocked(RecoveryReason::kFeedbackInvalid, false);
  }
}

ReconnectStatus ReconnectGate::Evaluate(bool authorization_publisher_valid,
                                        TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!authorization_publisher_valid) {
    EnterRecoveryLocked(RecoveryReason::kAuthorizationPublisherInvalid, false);
  } else if (!has_heartbeat_) {
    EnterRecoveryLocked(RecoveryReason::kHeartbeatMissing, false);
  } else if (!heartbeat_ready_) {
    EnterRecoveryLocked(RecoveryReason::kHeartbeatNotReady, false);
  } else if (now - last_heartbeat_ > kHeartbeatTimeout) {
    EnterRecoveryLocked(RecoveryReason::kHeartbeatStale, false);
  } else {
    const std::uint32_t generation =
        static_cast<std::uint32_t>(authorization_ >> 32U);
    const std::uint32_t session = static_cast<std::uint32_t>(authorization_);
    if (generation == 0U || session == 0U || session != heartbeat_session_id_) {
      EnterRecoveryLocked(RecoveryReason::kAuthorizationInvalid, false);
    } else {
      for (FeedbackStream stream :
           {FeedbackStream::kMotor, FeedbackStream::kImu,
            FeedbackStream::kPwm}) {
        if ((required_feedback_mask_ & FeedbackMask(stream)) == 0U) {
          continue;
        }
        const auto& observation = feedback_[FeedbackIndex(stream)];
        if (!observation.seen) {
          EnterRecoveryLocked(RecoveryReason::kFeedbackMissing, false);
          break;
        }
        if (!observation.valid) {
          EnterRecoveryLocked(RecoveryReason::kFeedbackInvalid, false);
          break;
        }
        if (now - observation.received_at > observation.timeout) {
          EnterRecoveryLocked(RecoveryReason::kFeedbackStale, false);
          break;
        }
      }
    }
  }

  if (!recovering_) {
    return StatusLocked(false);
  }

  const bool report_transition = transition_pending_;
  transition_pending_ = false;
  if (!recovery_cycle_observed_) {
    recovery_cycle_observed_ = true;
    return StatusLocked(report_transition);
  }

  const std::uint32_t generation =
      static_cast<std::uint32_t>(authorization_ >> 32U);
  const std::uint32_t session = static_cast<std::uint32_t>(authorization_);
  bool prerequisites_ready =
      authorization_publisher_valid && has_heartbeat_ && heartbeat_ready_ &&
      now - last_heartbeat_ <= kHeartbeatTimeout && generation != 0U &&
      session != 0U && session == heartbeat_session_id_ &&
      heartbeat_sequence_ > recovery_heartbeat_sequence_;
  if (require_new_generation_ && generation == last_connected_generation_) {
    prerequisites_ready = false;
  }
  for (FeedbackStream stream :
       {FeedbackStream::kMotor, FeedbackStream::kImu, FeedbackStream::kPwm}) {
    if ((required_feedback_mask_ & FeedbackMask(stream)) == 0U) {
      continue;
    }
    const std::size_t index = FeedbackIndex(stream);
    const auto& observation = feedback_[index];
    prerequisites_ready =
        prerequisites_ready && observation.seen && observation.valid &&
        now - observation.received_at <= observation.timeout &&
        observation.valid_sequence > recovery_feedback_sequence_[index];
  }
  if (!prerequisites_ready) {
    return StatusLocked(report_transition);
  }

  recovering_ = false;
  require_new_generation_ = false;
  last_connected_generation_ = generation;
  return StatusLocked(true);
}

std::size_t ReconnectGate::FeedbackIndex(FeedbackStream stream) {
  switch (stream) {
    case FeedbackStream::kMotor:
      return 0U;
    case FeedbackStream::kImu:
      return 1U;
    case FeedbackStream::kPwm:
      return 2U;
  }
  return 3U;
}

void ReconnectGate::ResetLocked(TimePoint now) {
  for (auto& observation : feedback_) {
    observation.seen = false;
    observation.valid = false;
    observation.received_at = now;
    observation.valid_sequence = 0U;
  }
  recovery_feedback_sequence_.fill(0U);
  heartbeat_sequence_ = 0U;
  recovery_heartbeat_sequence_ = 0U;
  authorization_ = 0U;
  last_heartbeat_ = now;
  heartbeat_session_id_ = 0U;
  heartbeat_uptime_ms_ = 0U;
  last_connected_generation_ = 0U;
  recovery_reason_ = RecoveryReason::kInitial;
  has_heartbeat_ = false;
  heartbeat_ready_ = false;
  recovering_ = true;
  recovery_cycle_observed_ = false;
  require_new_generation_ = false;
  transition_pending_ = true;
}

void ReconnectGate::EnterRecoveryLocked(RecoveryReason reason,
                                        bool require_new_generation,
                                        bool force_new_snapshot) {
  if (!recovering_ || force_new_snapshot) {
    for (std::size_t index = 0U; index < feedback_.size(); ++index) {
      recovery_feedback_sequence_[index] = feedback_[index].valid_sequence;
    }
    recovery_heartbeat_sequence_ = heartbeat_sequence_;
    recovering_ = true;
    recovery_cycle_observed_ = false;
    transition_pending_ = true;
    recovery_reason_ = reason;
  } else if (recovery_reason_ == RecoveryReason::kInitial) {
    recovery_reason_ = reason;
  }
  require_new_generation_ = require_new_generation_ || require_new_generation;
}

ReconnectStatus ReconnectGate::StatusLocked(bool transition) const {
  return ReconnectStatus{
      !recovering_,         transition,
      recovery_reason_,     heartbeat_session_id_,
      heartbeat_uptime_ms_, static_cast<std::uint32_t>(authorization_ >> 32U)};
}

bool FirstOrderLowPass::Configure(double cutoff_hz) {
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0) {
    return false;
  }
  cutoff_hz_ = cutoff_hz;
  Reset();
  return true;
}

void FirstOrderLowPass::Reset() {
  output_ = 0.0;
  initialized_ = false;
}

std::optional<double> FirstOrderLowPass::Update(double measurement,
                                                double period_seconds) {
  if (!std::isfinite(measurement) || !std::isfinite(period_seconds) ||
      period_seconds <= 0.0) {
    Reset();
    return std::nullopt;
  }
  if (!initialized_) {
    output_ = measurement;
    initialized_ = true;
    return output_;
  }
  const double alpha = -std::expm1(-kTwoPi * cutoff_hz_ * period_seconds);
  output_ += alpha * (measurement - output_);
  if (!std::isfinite(output_)) {
    Reset();
    return std::nullopt;
  }
  return output_;
}

bool FirstOrderLadrc::Configure(double controller_bandwidth_rad_s,
                                double observer_bandwidth_rad_s) {
  if (!std::isfinite(controller_bandwidth_rad_s) ||
      !std::isfinite(observer_bandwidth_rad_s) ||
      controller_bandwidth_rad_s <= 0.0 ||
      observer_bandwidth_rad_s < controller_bandwidth_rad_s) {
    return false;
  }
  controller_bandwidth_rad_s_ = controller_bandwidth_rad_s;
  observer_bandwidth_rad_s_ = observer_bandwidth_rad_s;
  Reset();
  return true;
}

void FirstOrderLadrc::Reset() {
  observed_output_ = 0.0;
  observed_disturbance_ = 0.0;
  initialized_ = false;
}

std::optional<double> FirstOrderLadrc::Update(double reference,
                                              double measurement,
                                              double applied_control,
                                              double input_gain,
                                              double period_seconds) {
  if (!std::isfinite(reference) || !std::isfinite(measurement) ||
      !std::isfinite(applied_control) || !std::isfinite(input_gain) ||
      !std::isfinite(period_seconds) || input_gain == 0.0 ||
      period_seconds <= 0.0 ||
      observer_bandwidth_rad_s_ * period_seconds > 0.5) {
    return std::nullopt;
  }
  if (!initialized_) {
    observed_output_ = measurement;
    observed_disturbance_ = 0.0;
    initialized_ = true;
  }
  const double observer_error = observed_output_ - measurement;
  observed_output_ +=
      period_seconds * (observed_disturbance_ + input_gain * applied_control -
                        2.0 * observer_bandwidth_rad_s_ * observer_error);
  observed_disturbance_ +=
      period_seconds *
      (-observer_bandwidth_rad_s_ * observer_bandwidth_rad_s_ * observer_error);
  const double control =
      (controller_bandwidth_rad_s_ * (reference - observed_output_) -
       observed_disturbance_) /
      input_gain;
  if (!std::isfinite(observed_output_) ||
      !std::isfinite(observed_disturbance_) || !std::isfinite(control)) {
    Reset();
    return std::nullopt;
  }
  return control;
}

bool IsValidSteeringCalibration(const SteeringCalibration& calibration) {
  return calibration.servo_index < 4U && calibration.minimum_pulse_us >= 500U &&
         calibration.maximum_pulse_us <= 2500U &&
         calibration.minimum_pulse_us < calibration.center_pulse_us &&
         calibration.center_pulse_us < calibration.maximum_pulse_us &&
         std::isfinite(calibration.minimum_angle_rad) &&
         std::isfinite(calibration.maximum_angle_rad) &&
         calibration.minimum_angle_rad < 0.0 &&
         calibration.maximum_angle_rad > 0.0;
}

std::optional<std::uint16_t> SteeringAngleToPulse(
    double angle_rad, const SteeringCalibration& calibration) {
  if (!std::isfinite(angle_rad) || !IsValidSteeringCalibration(calibration)) {
    return std::nullopt;
  }
  const double effective =
      std::clamp(EffectiveAngle(angle_rad, calibration),
                 calibration.minimum_angle_rad, calibration.maximum_angle_rad);
  double pulse = static_cast<double>(calibration.center_pulse_us);
  if (effective >= 0.0) {
    const double fraction = effective / calibration.maximum_angle_rad;
    pulse += fraction * static_cast<double>(calibration.maximum_pulse_us -
                                            calibration.center_pulse_us);
  } else {
    const double fraction = effective / -calibration.minimum_angle_rad;
    pulse += fraction * static_cast<double>(calibration.center_pulse_us -
                                            calibration.minimum_pulse_us);
  }
  return static_cast<std::uint16_t>(std::lround(pulse));
}

std::optional<double> SteeringPulseToAngle(
    std::uint16_t pulse_us, const SteeringCalibration& calibration) {
  if (!IsValidSteeringCalibration(calibration) ||
      pulse_us < calibration.minimum_pulse_us ||
      pulse_us > calibration.maximum_pulse_us) {
    return std::nullopt;
  }
  double effective = 0.0;
  if (pulse_us >= calibration.center_pulse_us) {
    const double fraction =
        static_cast<double>(pulse_us - calibration.center_pulse_us) /
        static_cast<double>(calibration.maximum_pulse_us -
                            calibration.center_pulse_us);
    effective = fraction * calibration.maximum_angle_rad;
  } else {
    const double fraction =
        static_cast<double>(calibration.center_pulse_us - pulse_us) /
        static_cast<double>(calibration.center_pulse_us -
                            calibration.minimum_pulse_us);
    effective = fraction * calibration.minimum_angle_rad;
  }
  return calibration.inverted ? -effective : effective;
}

}  // namespace mentor_pi::hardware
