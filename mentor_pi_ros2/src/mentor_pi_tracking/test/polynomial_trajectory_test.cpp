// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/polynomial_trajectory.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace mentor_pi::tracking {
namespace {

mentor_pi_tracking_interfaces::msg::PolynomialTrajectory ValidMessage() {
  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory message;
  message.header.frame_id = "odom";
  message.trajectory_id = "trajectory-1";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment first;
  first.duration.sec = 2;
  first.x_coefficients[1] = 1.0;
  first.yaw_coefficients[1] = 0.5;
  mentor_pi_tracking_interfaces::msg::PolynomialSegment second;
  second.duration.sec = 1;
  second.x_coefficients[0] = 2.0;
  second.x_coefficients[1] = 2.0;
  second.yaw_coefficients[0] = 1.0;
  message.segments = {first, second};
  return message;
}

TEST(PolynomialTrajectoryTest, EvaluatesSegmentsAndDerivatives) {
  std::string error;
  const auto trajectory =
      PolynomialTrajectory::FromMessage(ValidMessage(), &error);
  ASSERT_TRUE(trajectory.has_value()) << error;
  EXPECT_DOUBLE_EQ(trajectory->duration(), 3.0);

  const ReferenceState first = trajectory->Evaluate(1.0);
  EXPECT_DOUBLE_EQ(first.x, 1.0);
  EXPECT_DOUBLE_EQ(first.vx_world, 1.0);
  EXPECT_DOUBLE_EQ(first.yaw, 0.5);
  EXPECT_DOUBLE_EQ(first.yaw_rate, 0.5);

  const ReferenceState second = trajectory->Evaluate(2.5);
  EXPECT_DOUBLE_EQ(second.x, 3.0);
  EXPECT_DOUBLE_EQ(second.vx_world, 2.0);
}

TEST(PolynomialTrajectoryTest, RejectsWrongFrameNonFiniteAndDiscontinuity) {
  std::string error;
  auto message = ValidMessage();
  message.header.frame_id = "map";
  EXPECT_FALSE(PolynomialTrajectory::FromMessage(message, &error).has_value());

  message = ValidMessage();
  message.segments[0].x_coefficients[2] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PolynomialTrajectory::FromMessage(message, &error).has_value());

  message = ValidMessage();
  message.segments[0].x_coefficients[5] = std::numeric_limits<double>::max();
  EXPECT_FALSE(PolynomialTrajectory::FromMessage(message, &error).has_value());

  message = ValidMessage();
  message.segments[0].duration.nanosec = 1'000'000'000U;
  EXPECT_FALSE(PolynomialTrajectory::FromMessage(message, &error).has_value());

  message = ValidMessage();
  message.segments[1].x_coefficients[0] = 4.0;
  EXPECT_FALSE(PolynomialTrajectory::FromMessage(message, &error).has_value());
}

TEST(PolynomialTrajectoryTest, WrapsAnglesToShortestDifference) {
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(std::abs(WrapAngle(3.0 * pi)), pi, 1.0e-12);
  EXPECT_NEAR(std::abs(WrapAngle(-3.0 * pi)), pi, 1.0e-12);
}

TEST(PolynomialTrajectoryTest, AnalyticDerivativeMatchesFiniteDifference) {
  auto message = mentor_pi_tracking_interfaces::msg::PolynomialTrajectory{};
  message.header.frame_id = "odom";
  message.trajectory_id = "derivative";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 2;
  segment.x_coefficients = {{0.5, -1.0, 2.0, -0.25, 0.1, -0.02}};
  message.segments.push_back(segment);
  std::string error;
  const auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
  ASSERT_TRUE(trajectory.has_value()) << error;
  constexpr double kTime = 0.7;
  constexpr double kStep = 1.0e-6;
  const double numerical = (trajectory->Evaluate(kTime + kStep).x -
                            trajectory->Evaluate(kTime - kStep).x) /
                           (2.0 * kStep);
  EXPECT_NEAR(trajectory->Evaluate(kTime).vx_world, numerical, 1.0e-8);
}

}  // namespace
}  // namespace mentor_pi::tracking
