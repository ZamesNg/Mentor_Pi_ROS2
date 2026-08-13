#include "mentor_pi_hardwares/hardware_common.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace mentor_pi::hardware {
namespace {

TEST(HardwareCommonTest, MapsLogicalWheelsToFirmwareConnectors) {
  EXPECT_EQ(McuMotorIndex(Wheel::kFrontLeft), 0U);
  EXPECT_EQ(McuMotorIndex(Wheel::kRearLeft), 1U);
  EXPECT_EQ(McuMotorIndex(Wheel::kFrontRight), 2U);
  EXPECT_EQ(McuMotorIndex(Wheel::kRearRight), 3U);
  EXPECT_EQ(McuMotorMask(Wheel::kRearLeft), 0x02U);
  EXPECT_EQ(McuMotorMask(Wheel::kRearRight), 0x08U);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kFrontLeft), 1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kRearLeft), 1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kFrontRight), -1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kRearRight), -1);
}

TEST(HardwareCommonTest, ConvertsRosVelocityAndEncoderUnits) {
  EXPECT_DOUBLE_EQ(RpsToRadiansPerSecond(1.0), kTwoPi);
  ASSERT_TRUE(RadiansPerSecondToRps(kTwoPi, 6.0).has_value());
  EXPECT_FLOAT_EQ(*RadiansPerSecondToRps(kTwoPi, 6.0), 1.0F);
  EXPECT_FALSE(RadiansPerSecondToRps(kTwoPi * 6.01, 6.0).has_value());
  EXPECT_FALSE(
      RadiansPerSecondToRps(std::numeric_limits<double>::quiet_NaN(), 6.0)
          .has_value());
  EXPECT_DOUBLE_EQ(EncoderCountToRadians(1040, 1040U), kTwoPi);
  EXPECT_TRUE(std::isnan(EncoderCountToRadians(1, 0U)));
}

TEST(HardwareCommonTest, ExposesLegacyMotorProfileLimits) {
  EXPECT_DOUBLE_EQ(*MotorMaximumRps(0U), 1.5);
  EXPECT_DOUBLE_EQ(*MotorMaximumRps(1U), 3.0);
  EXPECT_DOUBLE_EQ(*MotorMaximumRps(2U), 6.0);
  EXPECT_NEAR(*MotorMaximumRps(3U), 1.1, 1.0e-6);
  EXPECT_EQ(*MotorTicksPerRevolution(2U), 1040U);
  EXPECT_FALSE(MotorMaximumRps(4U).has_value());
}

TEST(HardwareCommonTest, FirstOrderLadrcRejectsInvalidTimingAndResets) {
  FirstOrderLadrc controller;
  EXPECT_FALSE(controller.Configure(0.0, 3.0));
  EXPECT_FALSE(controller.Configure(4.0, 3.0));
  ASSERT_TRUE(controller.Configure(1.0, 3.0));

  const auto first = controller.Update(1.0, 0.0, 0.0, 5.0, 1.0 / 30.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(*first, 0.2, 1.0e-9);
  EXPECT_FALSE(controller.Update(1.0, 0.0, 0.0, 5.0, 0.2).has_value());

  controller.Reset();
  const auto reverse = controller.Update(1.0, 0.0, 0.0, -5.0, 1.0 / 30.0);
  ASSERT_TRUE(reverse.has_value());
  EXPECT_NEAR(*reverse, -0.2, 1.0e-9);
}

TEST(HardwareCommonTest, ConvertsConfiguredSteeringCalibration) {
  SteeringCalibration calibration;
  EXPECT_TRUE(IsValidSteeringCalibration(calibration));
  EXPECT_EQ(*SteeringAngleToPulse(0.0, calibration), 1500U);
  EXPECT_EQ(*SteeringAngleToPulse(0.6, calibration), 500U);
  EXPECT_EQ(*SteeringAngleToPulse(-0.6, calibration), 2500U);
  EXPECT_DOUBLE_EQ(*SteeringPulseToAngle(500U, calibration), 0.6);
  EXPECT_DOUBLE_EQ(*SteeringPulseToAngle(2500U, calibration), -0.6);
  EXPECT_FALSE(
      SteeringAngleToPulse(std::numeric_limits<double>::infinity(), calibration)
          .has_value());

  calibration.center_pulse_us = calibration.minimum_pulse_us;
  EXPECT_FALSE(IsValidSteeringCalibration(calibration));
}

}  // namespace
}  // namespace mentor_pi::hardware
