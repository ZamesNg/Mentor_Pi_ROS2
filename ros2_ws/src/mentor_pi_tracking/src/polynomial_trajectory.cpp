// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/polynomial_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mentor_pi::tracking {
namespace {

constexpr double kContinuityTolerance = 1.0e-3;
constexpr std::size_t kMaximumSegments = 64U;

bool IsFinite(const std::array<double, 6>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool IsEvaluable(const std::array<double, 6>& values, double duration) {
  long double magnitude_bound = 0.0L;
  long double derivative_bound = 0.0L;
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (values[index] == 0.0) {
      continue;
    }
    const long double coefficient =
        std::abs(static_cast<long double>(values[index]));
    magnitude_bound +=
        coefficient *
        std::pow(static_cast<long double>(duration), static_cast<int>(index));
    if (index > 0U) {
      derivative_bound += static_cast<long double>(index) * coefficient *
                          std::pow(static_cast<long double>(duration),
                                   static_cast<int>(index - 1U));
    }
  }
  return std::isfinite(magnitude_bound) && std::isfinite(derivative_bound) &&
         magnitude_bound <= std::numeric_limits<double>::max() &&
         derivative_bound <= std::numeric_limits<double>::max();
}

double DurationSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

double WrapAngle(double angle) {
  return std::remainder(angle, 2.0 * std::acos(-1.0));
}

std::optional<PolynomialTrajectory> PolynomialTrajectory::FromMessage(
    const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory& message,
    std::string* error) {
  auto reject = [error](const char* reason) {
    if (error != nullptr) {
      *error = reason;
    }
    return std::optional<PolynomialTrajectory>{};
  };
  if (message.header.frame_id != "map") {
    return reject("trajectory frame must be map");
  }
  if (message.trajectory_id.empty() || message.trajectory_id.size() > 64U) {
    return reject("trajectory_id must contain 1..64 characters");
  }
  if (message.segments.empty() || message.segments.size() > kMaximumSegments) {
    return reject("trajectory must contain 1..64 segments");
  }

  PolynomialTrajectory trajectory;
  trajectory.id_ = message.trajectory_id;
  trajectory.segments_.reserve(message.segments.size());
  for (const auto& input : message.segments) {
    if (input.duration.sec < 0 || input.duration.nanosec >= 1'000'000'000U) {
      return reject("segment duration must use canonical ROS seconds");
    }
    PolynomialSegment segment;
    segment.duration = DurationSeconds(input.duration);
    segment.x = input.x_coefficients;
    segment.y = input.y_coefficients;
    segment.yaw = input.yaw_coefficients;
    if (!std::isfinite(segment.duration) || segment.duration <= 0.0 ||
        !IsFinite(segment.x) || !IsFinite(segment.y) ||
        !IsFinite(segment.yaw) || !IsEvaluable(segment.x, segment.duration) ||
        !IsEvaluable(segment.y, segment.duration) ||
        !IsEvaluable(segment.yaw, segment.duration)) {
      return reject("segment duration and coefficients must be finite");
    }
    if (trajectory.duration_ >
        std::numeric_limits<double>::max() - segment.duration) {
      return reject("trajectory duration overflow");
    }
    trajectory.duration_ += segment.duration;
    trajectory.segments_.push_back(segment);
  }

  for (std::size_t index = 1U; index < trajectory.segments_.size(); ++index) {
    const auto& previous = trajectory.segments_[index - 1U];
    const auto& next = trajectory.segments_[index];
    const bool continuous =
        std::abs(EvaluatePolynomial(previous.x, previous.duration) -
                 next.x[0]) <= kContinuityTolerance &&
        std::abs(EvaluatePolynomial(previous.y, previous.duration) -
                 next.y[0]) <= kContinuityTolerance &&
        std::abs(EvaluatePolynomial(previous.yaw, previous.duration) -
                 next.yaw[0]) <= kContinuityTolerance;
    if (!continuous) {
      return reject("trajectory segments are not position-continuous");
    }
  }
  return trajectory;
}

ReferenceState PolynomialTrajectory::Evaluate(double elapsed_seconds) const {
  const bool terminal_hold = elapsed_seconds >= duration_;
  double remaining = std::clamp(elapsed_seconds, 0.0, duration_);
  const PolynomialSegment* selected = &segments_.back();
  for (const auto& segment : segments_) {
    selected = &segment;
    if (remaining <= segment.duration) {
      break;
    }
    remaining -= segment.duration;
  }
  remaining = std::min(remaining, selected->duration);
  const double vx_world =
      terminal_hold ? 0.0 : EvaluateDerivative(selected->x, remaining);
  const double vy_world =
      terminal_hold ? 0.0 : EvaluateDerivative(selected->y, remaining);
  const double yaw_rate =
      terminal_hold ? 0.0 : EvaluateDerivative(selected->yaw, remaining);
  return ReferenceState{EvaluatePolynomial(selected->x, remaining),
                        EvaluatePolynomial(selected->y, remaining),
                        EvaluatePolynomial(selected->yaw, remaining),
                        vx_world,
                        vy_world,
                        yaw_rate};
}

double PolynomialTrajectory::EvaluatePolynomial(
    const std::array<double, 6>& coefficients, double time) {
  double value = coefficients.back();
  for (std::size_t index = coefficients.size() - 1U; index > 0U; --index) {
    value = value * time + coefficients[index - 1U];
  }
  return value;
}

double PolynomialTrajectory::EvaluateDerivative(
    const std::array<double, 6>& coefficients, double time) {
  double value = 5.0 * coefficients[5];
  for (std::size_t index = 5U; index > 1U; --index) {
    value = value * time +
            static_cast<double>(index - 1U) * coefficients[index - 1U];
  }
  return value;
}

}  // namespace mentor_pi::tracking
