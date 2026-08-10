// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef MENTOR_PI_TRACKING__POLYNOMIAL_TRAJECTORY_HPP_
#define MENTOR_PI_TRACKING__POLYNOMIAL_TRAJECTORY_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"

namespace mentor_pi::tracking {

struct ReferenceState {
  double x{};
  double y{};
  double yaw{};
  double vx_world{};
  double vy_world{};
  double yaw_rate{};
};

struct PolynomialSegment {
  double duration{};
  std::array<double, 6> x{};
  std::array<double, 6> y{};
  std::array<double, 6> yaw{};
};

class PolynomialTrajectory {
 public:
  static std::optional<PolynomialTrajectory> FromMessage(
      const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory& message,
      std::string* error);

  ReferenceState Evaluate(double elapsed_seconds) const;
  double duration() const { return duration_; }
  const std::string& id() const { return id_; }

 private:
  static double EvaluatePolynomial(const std::array<double, 6>& coefficients,
                                   double time);
  static double EvaluateDerivative(const std::array<double, 6>& coefficients,
                                   double time);

  std::string id_;
  std::vector<PolynomialSegment> segments_;
  double duration_{};
};

double WrapAngle(double angle);

}  // namespace mentor_pi::tracking

#endif  // MENTOR_PI_TRACKING__POLYNOMIAL_TRAJECTORY_HPP_
