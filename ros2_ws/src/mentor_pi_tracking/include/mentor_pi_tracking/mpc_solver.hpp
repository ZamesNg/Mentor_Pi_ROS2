// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef MENTOR_PI_TRACKING__MPC_SOLVER_HPP_
#define MENTOR_PI_TRACKING__MPC_SOLVER_HPP_

#include <array>
#include <string>

#include "mentor_pi_tracking/polynomial_trajectory.hpp"

namespace mentor_pi::tracking {

enum class VehicleType { kMecanum, kAckermann };

struct MpcConfiguration {
  VehicleType vehicle{VehicleType::kMecanum};
  int horizon{10};
  double prediction_step{0.1};
  double wheelbase{0.145};
  double mecanum_radius_sum{0.14};
  double max_linear_speed{0.5};
  double max_lateral_speed{0.5};
  double max_yaw_rate{1.5};
  double max_steering_angle{0.5};
};

struct MpcRequest {
  std::array<double, 3> state{};
  const PolynomialTrajectory* trajectory{};
  double elapsed_seconds{};
};

struct MpcCommand {
  bool solved{};
  double linear_x{};
  double linear_y{};
  double angular_z{};
  std::string detail;
};

class MpcSolver {
 public:
  explicit MpcSolver(MpcConfiguration configuration);
  MpcCommand Solve(const MpcRequest& request) const;

 private:
  MpcConfiguration configuration_;
};

MpcCommand FeedbackCommand(const MpcConfiguration& configuration,
                           const MpcRequest& request);
MpcCommand EnforceCommandBounds(const MpcConfiguration& configuration,
                                MpcCommand command);

}  // namespace mentor_pi::tracking

#endif  // MENTOR_PI_TRACKING__MPC_SOLVER_HPP_
