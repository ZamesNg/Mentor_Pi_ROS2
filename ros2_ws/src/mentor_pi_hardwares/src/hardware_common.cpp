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
