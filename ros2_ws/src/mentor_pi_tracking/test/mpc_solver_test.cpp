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

PolynomialTrajectory StraightTrajectoryWithYaw(double yaw, double yaw_rate) {
  auto message = mentor_pi_tracking_interfaces::msg::PolynomialTrajectory{};
  message.header.frame_id = "odom";
  message.trajectory_id = "straight-yaw-" + std::to_string(yaw);
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 5;
  segment.x_coefficients[1] = 0.1;
  segment.yaw_coefficients[0] = yaw;
  segment.yaw_coefficients[1] = yaw_rate;
  message.segments.push_back(segment);
  std::string error;
  const auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
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
  configuration.wheelbase = 0.135;
  const MpcRequest request{{0.0, 0.1, 0.0}, &trajectory, 0.0};
  const MpcCommand command = FeedbackCommand(configuration, request);
  EXPECT_TRUE(command.solved);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_TRUE(std::isfinite(command.angular_z));
}

TEST(MpcSolverTest, AckermannFeedbackIgnoresCommonYawPolynomial) {
  const PolynomialTrajectory first = StraightTrajectoryWithYaw(0.0, 0.0);
  const PolynomialTrajectory second = StraightTrajectoryWithYaw(2.0, -4.0);
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kAckermann;
  configuration.max_linear_speed = 0.3;
  configuration.max_steering_angle = 0.4;
  configuration.wheelbase = 0.135;
  configuration.geometry_center_offset = 0.0675;
  const MpcCommand first_command =
      FeedbackCommand(configuration, {{0.0, 0.1, 0.2}, &first, 0.0});
  const MpcCommand second_command =
      FeedbackCommand(configuration, {{0.0, 0.1, 0.2}, &second, 0.0});
  ASSERT_TRUE(first_command.solved);
  ASSERT_TRUE(second_command.solved);
  EXPECT_DOUBLE_EQ(first_command.linear_x, second_command.linear_x);
  EXPECT_DOUBLE_EQ(first_command.linear_y, second_command.linear_y);
  EXPECT_DOUBLE_EQ(first_command.angular_z, second_command.angular_z);
}

TEST(MpcSolverTest, TerminalFeedbackHoldsEndpointForBothVehicles) {
  const PolynomialTrajectory trajectory = StraightTrajectoryWithYaw(0.25, 0.2);

  MpcConfiguration mecanum;
  mecanum.vehicle = VehicleType::kMecanum;
  const MpcCommand mecanum_at_endpoint = FeedbackCommand(
      mecanum, {{0.5, 0.0, 1.25}, &trajectory, trajectory.duration()});
  ASSERT_TRUE(mecanum_at_endpoint.solved);
  EXPECT_NEAR(mecanum_at_endpoint.linear_x, 0.0, 1.0e-12);
  EXPECT_NEAR(mecanum_at_endpoint.linear_y, 0.0, 1.0e-12);
  EXPECT_NEAR(mecanum_at_endpoint.angular_z, 0.0, 1.0e-12);
  const MpcCommand mecanum_behind = FeedbackCommand(
      mecanum, {{0.4, 0.0, 1.25}, &trajectory, trajectory.duration()});
  ASSERT_TRUE(mecanum_behind.solved);
  EXPECT_GT(mecanum_behind.linear_x, 0.0);

  MpcConfiguration ackermann;
  ackermann.vehicle = VehicleType::kAckermann;
  const MpcCommand ackermann_at_endpoint = FeedbackCommand(
      ackermann, {{0.5, 0.0, -2.0}, &trajectory, trajectory.duration()});
  ASSERT_TRUE(ackermann_at_endpoint.solved);
  EXPECT_NEAR(ackermann_at_endpoint.linear_x, 0.0, 1.0e-12);
  EXPECT_NEAR(ackermann_at_endpoint.linear_y, 0.0, 1.0e-12);
  EXPECT_NEAR(ackermann_at_endpoint.angular_z, 0.0, 1.0e-12);
}

TEST(MpcSolverTest, AckermannAltoSolveIgnoresCommonYawPolynomial) {
  const PolynomialTrajectory first = StraightTrajectoryWithYaw(0.0, 0.0);
  const PolynomialTrajectory second = StraightTrajectoryWithYaw(-2.5, 6.0);
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kAckermann;
  configuration.max_linear_speed = 0.3;
  configuration.max_lateral_speed = 0.3;
  configuration.max_yaw_rate = 1.0;
  configuration.max_steering_angle = 0.4;
  configuration.wheelbase = 0.135;
  configuration.geometry_center_offset = 0.0675;
  const MpcCommand first_command =
      MpcSolver(configuration).Solve({{0.0, 0.0, 0.0}, &first, 0.0});
  const MpcCommand second_command =
      MpcSolver(configuration).Solve({{0.0, 0.0, 0.0}, &second, 0.0});
  ASSERT_TRUE(first_command.solved) << first_command.detail;
  ASSERT_TRUE(second_command.solved) << second_command.detail;
  EXPECT_NEAR(first_command.linear_x, second_command.linear_x, 1.0e-12);
  EXPECT_NEAR(first_command.linear_y, second_command.linear_y, 1.0e-12);
  EXPECT_NEAR(first_command.angular_z, second_command.angular_z, 1.0e-12);
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
  EXPECT_LE(std::abs(command.linear_x), 0.3);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_LE(std::abs(command.angular_z),
            std::abs(command.linear_x) * std::tan(0.4) / 0.15 + 1.0e-12);
  EXPECT_LE(std::abs(command.linear_x -
                     0.5 * configuration.wheel_track * command.angular_z),
            0.3 + 1.0e-12);
  EXPECT_LE(std::abs(command.linear_x +
                     0.5 * configuration.wheel_track * command.angular_z),
            0.3 + 1.0e-12);
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

TEST(MpcSolverTest, PredictionBeyondExecutionHorizonIsStationary) {
  const PolynomialTrajectory trajectory = StraightTrajectory();
  MpcConfiguration configuration;
  configuration.vehicle = VehicleType::kMecanum;
  const MpcCommand command =
      MpcSolver(configuration)
          .Solve({{0.5, 0.0, 0.0}, &trajectory, trajectory.duration()});
  ASSERT_TRUE(command.solved) << command.detail;
  EXPECT_NEAR(command.linear_x, 0.0, 1.0e-6);
  EXPECT_NEAR(command.linear_y, 0.0, 1.0e-6);
  EXPECT_NEAR(command.angular_z, 0.0, 1.0e-6);
}

}  // namespace
}  // namespace mentor_pi::tracking
