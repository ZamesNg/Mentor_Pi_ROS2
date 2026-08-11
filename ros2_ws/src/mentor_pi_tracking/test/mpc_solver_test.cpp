// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/mpc_solver.hpp"

#include <cmath>
#include <string>

#include "gtest/gtest.h"

namespace mentor_pi::tracking {
namespace {

PolynomialTrajectory StraightTrajectory() {
  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory message;
  message.header.frame_id = "odom";
  message.trajectory_id = "straight";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 5;
  segment.x_coefficients[1] = 0.1;
  message.segments.push_back(segment);
  std::string error;
  auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
  EXPECT_TRUE(trajectory.has_value()) << error;
  return *trajectory;
}

TEST(MpcSolverTest, MecanumFeedbackIsFiniteAndBounded) {
  const PolynomialTrajectory trajectory = StraightTrajectory();
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kMecanum;
  configuration.max_linear_speed = 0.2;
  configuration.max_lateral_speed = 0.2;
  configuration.max_yaw_rate = 0.5;
  const MpcRequest request{{-1.0, 1.0, 0.0}, &trajectory, 0.0};
  const MpcCommand command = FeedbackCommand(configuration, request);
  EXPECT_TRUE(command.solved);
  EXPECT_GE(command.linear_x, -0.2);
  EXPECT_LE(command.linear_x, 0.2);
  EXPECT_GE(command.linear_y, -0.2);
  EXPECT_LE(command.linear_y, 0.2);
  EXPECT_GE(command.angular_z, -0.5);
  EXPECT_LE(command.angular_z, 0.5);
}

TEST(MpcSolverTest, AckermannFeedbackUsesTwistContract) {
  const PolynomialTrajectory trajectory = StraightTrajectory();
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kAckermann;
  configuration.max_linear_speed = 0.3;
  configuration.max_steering_angle = 0.4;
  configuration.wheelbase = 0.145;
  const MpcRequest request{{0.0, 0.1, 0.0}, &trajectory, 0.0};
  const MpcCommand command = FeedbackCommand(configuration, request);
  EXPECT_TRUE(command.solved);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_TRUE(std::isfinite(command.angular_z));
}

TEST(MpcSolverTest, CombinedMecanumCommandRespectsEveryWheelLimit) {
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kMecanum;
  configuration.max_linear_speed = 0.4;
  configuration.mecanum_radius_sum = 0.2;
  const MpcCommand command =
      EnforceCommandBounds(configuration, {true, 0.4, 0.4, 2.0, "combined"});
  const double yaw_component =
      configuration.mecanum_radius_sum * command.angular_z;
  EXPECT_LE(std::abs(command.linear_x - command.linear_y - yaw_component),
            0.4 + 1.0e-12);
  EXPECT_LE(std::abs(command.linear_x + command.linear_y + yaw_component),
            0.4 + 1.0e-12);
  EXPECT_LE(std::abs(command.linear_x + command.linear_y - yaw_component),
            0.4 + 1.0e-12);
  EXPECT_LE(std::abs(command.linear_x - command.linear_y + yaw_component),
            0.4 + 1.0e-12);
}

TEST(MpcSolverTest, AckermannBoundPreservesSteeringGeometry) {
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kAckermann;
  configuration.max_linear_speed = 0.3;
  configuration.max_steering_angle = 0.4;
  configuration.wheelbase = 0.15;
  const MpcCommand command =
      EnforceCommandBounds(configuration, {true, 1.0, 0.5, 10.0, "combined"});
  EXPECT_DOUBLE_EQ(command.linear_x, 0.3);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_LE(std::abs(command.angular_z), 0.3 * std::tan(0.4) / 0.15 + 1.0e-12);
}

TEST(MpcSolverTest, SolvesStationaryMecanumProblemWithAlto) {
  auto message = mentor_pi_tracking_interfaces::msg::PolynomialTrajectory{};
  message.header.frame_id = "odom";
  message.trajectory_id = "stationary";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 2;
  message.segments.push_back(segment);
  std::string error;
  const auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
  ASSERT_TRUE(trajectory.has_value()) << error;
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kMecanum;
  const MpcCommand command =
      MpcSolver(configuration).Solve({{0.0, 0.0, 0.0}, &*trajectory, 0.0});
  EXPECT_TRUE(command.solved) << command.detail;
  EXPECT_NEAR(command.linear_x, 0.0, 1.0e-6);
  EXPECT_NEAR(command.linear_y, 0.0, 1.0e-6);
  EXPECT_NEAR(command.angular_z, 0.0, 1.0e-6);
}

}  // namespace
}  // namespace mentor_pi::tracking
