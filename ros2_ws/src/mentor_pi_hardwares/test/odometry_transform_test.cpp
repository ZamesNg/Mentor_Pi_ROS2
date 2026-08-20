#include "mentor_pi_hardwares/odometry_transform.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace mentor_pi::hardware {
namespace {

constexpr double kOffset = 0.0675;
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion Orientation(double yaw) {
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(0.5 * yaw);
  orientation.w = std::cos(0.5 * yaw);
  return orientation;
}

double OrientationYaw(const geometry_msgs::msg::Quaternion& orientation) {
  return std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      orientation.w * orientation.w + orientation.x * orientation.x -
          orientation.y * orientation.y - orientation.z * orientation.z);
}

nav_msgs::msg::Odometry SourceOdometry(double yaw) {
  nav_msgs::msg::Odometry input;
  input.header.stamp.sec = 17;
  input.header.stamp.nanosec = 42U;
  input.header.frame_id = "ackermann_1/odom";
  input.child_frame_id = "ackermann_1/rear_axle_footprint";
  input.pose.pose.position.x = 1.0;
  input.pose.pose.position.y = -2.0;
  input.pose.pose.orientation = Orientation(yaw);
  input.twist.twist.linear.x = 0.4;
  input.twist.twist.linear.y = -0.1;
  input.twist.twist.angular.z = 0.6;
  input.pose.covariance[0] = 1.0;
  input.pose.covariance[7] = 2.0;
  input.pose.covariance[35] = 4.0;
  input.twist.covariance[0] = 5.0;
  input.twist.covariance[7] = 6.0;
  input.twist.covariance[35] = 9.0;
  return input;
}

TEST(OdometryTransformTest, ConvertsRearAxlePoseAtZeroYaw) {
  const auto input = SourceOdometry(0.0);
  const auto output =
      ToGeometryCenterOdometry(input, kOffset, "ackermann_1/base_footprint");

  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.child_frame_id, "ackermann_1/base_footprint");
  EXPECT_NEAR(output.pose.pose.position.x, 1.0 + kOffset, 1.0e-12);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, -2.0);
  EXPECT_EQ(output.pose.pose.orientation, input.pose.pose.orientation);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 0.4);
  EXPECT_NEAR(output.twist.twist.linear.y, -0.1 + kOffset * 0.6, 1.0e-12);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, 0.6);

  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 1.0);
  EXPECT_NEAR(output.pose.covariance[7], 2.0 + 4.0 * kOffset * kOffset,
              1.0e-12);
  EXPECT_NEAR(output.pose.covariance[11], 4.0 * kOffset, 1.0e-12);
  EXPECT_NEAR(output.pose.covariance[31], 4.0 * kOffset, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[7], 6.0 + 9.0 * kOffset * kOffset,
              1.0e-12);
  EXPECT_NEAR(output.twist.covariance[11], 9.0 * kOffset, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[31], 9.0 * kOffset, 1.0e-12);
}

TEST(OdometryTransformTest, ConvertsPoseAtQuarterTurnAndNegativeYaw) {
  auto output = ToGeometryCenterOdometry(SourceOdometry(kPi / 2.0), kOffset,
                                         "ackermann_1/base_footprint");
  EXPECT_NEAR(output.pose.pose.position.x, 1.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y, -2.0 + kOffset, 1.0e-12);

  output = ToGeometryCenterOdometry(SourceOdometry(-kPi / 3.0), kOffset,
                                    "ackermann_1/base_footprint");
  EXPECT_NEAR(output.pose.pose.position.x, 1.0 + 0.5 * kOffset, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y,
              -2.0 - std::sqrt(3.0) * 0.5 * kOffset, 1.0e-12);
}

TEST(OdometryTransformTest, MecanumZeroOffsetIsIdentityExceptForFrame) {
  auto input = SourceOdometry(-0.4);
  input.child_frame_id = "mecanum_2/base_footprint";
  const auto output =
      ToGeometryCenterOdometry(input, 0.0, "mecanum_2/base_footprint");
  EXPECT_EQ(output, input);
}

TEST(OdometryTransformTest, PlacesInitialAckermannGeometryCenterExactly) {
  auto input = SourceOdometry(0.0);
  input.pose.pose.position.x = 0.0;
  input.pose.pose.position.y = 0.0;
  const double initial_x = 2.5;
  const double initial_y = -1.25;
  const double initial_yaw = kPi / 2.0;
  const PlanarTransform output_from_source{
      initial_x - kOffset * std::cos(initial_yaw),
      initial_y - kOffset * std::sin(initial_yaw),
      initial_yaw,
  };

  const auto output =
      ToGeometryCenterOdometry(input, kOffset, "ackermann_sim/base_footprint",
                               output_from_source, "ackermann_sim/odom");
  EXPECT_EQ(output.header.frame_id, "ackermann_sim/odom");
  EXPECT_NEAR(output.pose.pose.position.x, initial_x, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y, initial_y, 1.0e-12);
  EXPECT_NEAR(OrientationYaw(output.pose.pose.orientation), initial_yaw,
              1.0e-12);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, input.twist.twist.linear.x);
  EXPECT_NEAR(
      output.twist.twist.linear.y,
      input.twist.twist.linear.y + kOffset * input.twist.twist.angular.z,
      1.0e-12);
}

TEST(OdometryTransformTest, RotatesMecanumPoseAndPoseCovariance) {
  auto input = SourceOdometry(-0.25);
  input.pose.pose.position.x = 1.0;
  input.pose.pose.position.y = 2.0;
  input.pose.covariance.fill(0.0);
  input.pose.covariance[0] = 1.0;
  input.pose.covariance[7] = 4.0;
  input.pose.covariance[35] = 9.0;
  const PlanarTransform output_from_source{3.0, -4.0, -kPi / 2.0};

  const auto output = ToGeometryCenterOdometry(
      input, 0.0, "mecanum_sim/base_footprint", output_from_source);
  EXPECT_NEAR(output.pose.pose.position.x, 5.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y, -5.0, 1.0e-12);
  EXPECT_NEAR(OrientationYaw(output.pose.pose.orientation), -0.25 - kPi / 2.0,
              1.0e-12);
  EXPECT_NEAR(output.pose.covariance[0], 4.0, 1.0e-12);
  EXPECT_NEAR(output.pose.covariance[7], 1.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(output.pose.covariance[35], 9.0);
  EXPECT_EQ(output.twist, input.twist);
}

TEST(OdometryTransformTest, ProducesMapGeometryCenterPoseAndTransform) {
  auto input = SourceOdometry(0.0);
  input.pose.pose.position.x = 0.0;
  input.pose.pose.position.y = 0.0;
  input.pose.pose.orientation.w = 2.0;
  const double initial_x = 2.5;
  const double initial_y = -1.25;
  const double initial_yaw = kPi / 2.0;
  const PlanarTransform output_from_source{
      initial_x - kOffset * std::cos(initial_yaw),
      initial_y - kOffset * std::sin(initial_yaw), initial_yaw};

  const auto pose = GeometryCenterPoseFromOdometry(
      input, kOffset, "ackermann_sim/base_footprint", output_from_source,
      "map");
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_EQ(pose.header.stamp, input.header.stamp);
  EXPECT_NEAR(pose.pose.position.x, initial_x, 1.0e-12);
  EXPECT_NEAR(pose.pose.position.y, initial_y, 1.0e-12);
  EXPECT_NEAR(OrientationYaw(pose.pose.orientation), initial_yaw, 1.0e-12);
  EXPECT_NEAR(std::hypot(pose.pose.orientation.z, pose.pose.orientation.w), 1.0,
              1.0e-12);

  const auto transform =
      GeometryCenterPoseToTransform(pose, "ackermann_sim/base_footprint");
  EXPECT_EQ(transform.header.frame_id, "map");
  EXPECT_EQ(transform.child_frame_id, "ackermann_sim/base_footprint");
  EXPECT_NEAR(transform.transform.translation.x, initial_x, 1.0e-12);
  EXPECT_NEAR(transform.transform.translation.y, initial_y, 1.0e-12);
}

TEST(OdometryTransformTest, ValidatesMapMocapPoseWithoutChangingItsPoint) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "map";
  input.header.stamp.sec = 12;
  input.pose.position.x = 1.0;
  input.pose.position.y = -2.0;
  input.pose.position.z = 0.25;
  input.pose.orientation.w = 2.0;

  const auto output = ValidateGeometryCenterPose(input);
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.pose.position, input.pose.position);
  EXPECT_DOUBLE_EQ(output.pose.orientation.w, 1.0);

  input.header.frame_id = "odom";
  EXPECT_THROW(ValidateGeometryCenterPose(input), std::invalid_argument);
  input.header.frame_id = "map";
  input.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ValidateGeometryCenterPose(input), std::invalid_argument);
}

TEST(OdometryTransformTest, RejectsInvalidConfigurationAndOrientation) {
  const auto valid = SourceOdometry(0.0);
  EXPECT_THROW(ToGeometryCenterOdometry(valid, -0.1, "base_footprint"),
               std::invalid_argument);
  EXPECT_THROW(ToGeometryCenterOdometry(valid, 0.0, ""), std::invalid_argument);
  EXPECT_THROW(
      ToGeometryCenterOdometry(
          valid, 0.0, "base_footprint",
          PlanarTransform{0.0, 0.0, std::numeric_limits<double>::infinity()}),
      std::invalid_argument);

  auto invalid = valid;
  invalid.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ToGeometryCenterOdometry(invalid, 0.0, "base_footprint"),
               std::invalid_argument);
  invalid = valid;
  invalid.pose.pose.orientation.x = 0.0;
  invalid.pose.pose.orientation.y = 0.0;
  invalid.pose.pose.orientation.z = 0.0;
  invalid.pose.pose.orientation.w = 0.0;
  EXPECT_THROW(ToGeometryCenterOdometry(invalid, 0.0, "base_footprint"),
               std::invalid_argument);

  invalid = valid;
  invalid.pose.covariance[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ToGeometryCenterOdometry(invalid, 0.0, "base_footprint"),
               std::invalid_argument);
}

}  // namespace
}  // namespace mentor_pi::hardware
