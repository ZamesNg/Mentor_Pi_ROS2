#include "mentor_pi_hardwares/hardware_common.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace mentor_pi::hardware {
namespace {

TEST(HardwareCommonTest, UsesOneVehicleHardwareNodeName) {
  EXPECT_STREQ(kVehicleHardwareNodeName, "vehicle_hardware");
}

TEST(HardwareCommonTest, MapsLogicalWheelsToFirmwareConnectors) {
  EXPECT_EQ(McuMotorIndex(Wheel::kFrontLeft), 0U);
  EXPECT_EQ(McuMotorIndex(Wheel::kRearLeft), 1U);
  EXPECT_EQ(McuMotorIndex(Wheel::kFrontRight), 2U);
  EXPECT_EQ(McuMotorIndex(Wheel::kRearRight), 3U);
  EXPECT_EQ(McuMotorMask(Wheel::kRearLeft), 0x02U);
  EXPECT_EQ(McuMotorMask(Wheel::kRearRight), 0x08U);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kFrontLeft), -1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kRearLeft), -1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kFrontRight), 1);
  EXPECT_EQ(ChassisDirectionSign(Wheel::kRearRight), 1);
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

TEST(HardwareCommonTest, ReconnectGateStartsInhibitedAndRequiresFreshInputs) {
  using namespace std::chrono_literals;
  ReconnectGate gate;
  ASSERT_TRUE(gate.Configure(
      FeedbackMask(FeedbackStream::kMotor) | FeedbackMask(FeedbackStream::kImu),
      100ms, 100ms, 100ms));
  const auto start = ReconnectGate::TimePoint{};
  gate.Reset(start);
  gate.ObserveHeartbeat(7U, 1000U, true, start + 1ms);
  gate.ObserveAuthorization((UINT64_C(1) << 32U) | UINT64_C(7), start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 1ms);

  const auto inhibited = gate.Evaluate(true, start + 2ms);
  EXPECT_FALSE(inhibited.ready);
  EXPECT_TRUE(inhibited.transition);
  const auto ready = gate.Evaluate(true, start + 3ms);
  EXPECT_TRUE(ready.ready);
  EXPECT_TRUE(ready.transition);
  EXPECT_EQ(ready.session_id, 7U);
  EXPECT_EQ(ready.authorization_generation, 1U);
}

TEST(HardwareCommonTest, ReconnectGateRecoversSameSessionAfterEveryNewStream) {
  using namespace std::chrono_literals;
  ReconnectGate gate;
  ASSERT_TRUE(gate.Configure(
      FeedbackMask(FeedbackStream::kMotor) | FeedbackMask(FeedbackStream::kImu),
      100ms, 100ms, 100ms));
  const auto start = ReconnectGate::TimePoint{};
  gate.Reset(start);
  gate.ObserveHeartbeat(9U, 1000U, true, start + 1ms);
  gate.ObserveAuthorization((UINT64_C(3) << 32U) | UINT64_C(9), start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 1ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 2ms).ready);
  EXPECT_TRUE(gate.Evaluate(true, start + 3ms).ready);

  gate.ObserveHeartbeat(9U, 1150U, true, start + 150ms);
  const auto stale = gate.Evaluate(true, start + 151ms);
  EXPECT_FALSE(stale.ready);
  EXPECT_TRUE(stale.transition);
  EXPECT_EQ(stale.reason, RecoveryReason::kFeedbackStale);

  gate.ObserveHeartbeat(9U, 1160U, true, start + 160ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 160ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 161ms).ready);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 162ms);
  const auto recovered = gate.Evaluate(true, start + 163ms);
  EXPECT_TRUE(recovered.ready);
  EXPECT_TRUE(recovered.transition);
  EXPECT_EQ(recovered.authorization_generation, 3U);
}

TEST(HardwareCommonTest, ReconnectGateRequiresNewGenerationAfterRestart) {
  using namespace std::chrono_literals;
  ReconnectGate gate;
  ASSERT_TRUE(gate.Configure(
      FeedbackMask(FeedbackStream::kMotor) | FeedbackMask(FeedbackStream::kImu),
      100ms, 100ms, 100ms));
  const auto start = ReconnectGate::TimePoint{};
  gate.Reset(start);
  gate.ObserveHeartbeat(4U, 1000U, true, start + 1ms);
  gate.ObserveAuthorization((UINT64_C(8) << 32U) | UINT64_C(4), start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 1ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 2ms).ready);
  EXPECT_TRUE(gate.Evaluate(true, start + 3ms).ready);

  gate.ObserveHeartbeat(5U, 20U, true, start + 10ms);
  gate.ObserveAuthorization((UINT64_C(8) << 32U) | UINT64_C(5), start + 10ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 10ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 10ms);
  const auto changed = gate.Evaluate(true, start + 11ms);
  EXPECT_FALSE(changed.ready);
  EXPECT_EQ(changed.reason, RecoveryReason::kSessionChanged);
  gate.ObserveHeartbeat(5U, 30U, true, start + 12ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 12ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 12ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 13ms).ready);
  gate.ObserveAuthorization((UINT64_C(9) << 32U) | UINT64_C(5), start + 14ms);
  EXPECT_TRUE(gate.Evaluate(true, start + 15ms).ready);

  gate.ObserveHeartbeat(5U, 100U, true, start + 20ms);
  gate.ObserveHeartbeat(5U, 10U, true, start + 21ms);
  gate.ObserveAuthorization((UINT64_C(9) << 32U) | UINT64_C(5), start + 21ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 21ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 21ms);
  const auto regressed = gate.Evaluate(true, start + 22ms);
  EXPECT_FALSE(regressed.ready);
  EXPECT_EQ(regressed.reason, RecoveryReason::kUptimeRegressed);
  gate.ObserveHeartbeat(5U, 20U, true, start + 23ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 23ms);
  gate.ObserveFeedback(FeedbackStream::kImu, true, start + 23ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 24ms).ready);
  gate.ObserveAuthorization((UINT64_C(10) << 32U) | UINT64_C(5), start + 25ms);
  EXPECT_TRUE(gate.Evaluate(true, start + 26ms).ready);
}

TEST(HardwareCommonTest, ReconnectGateRejectsStaleHeartbeatAndAuthorization) {
  using namespace std::chrono_literals;
  ReconnectGate gate;
  ASSERT_TRUE(gate.Configure(FeedbackMask(FeedbackStream::kMotor), 100ms, 100ms,
                             100ms));
  const auto start = ReconnectGate::TimePoint{};
  gate.Reset(start);
  gate.ObserveHeartbeat(2U, 100U, true, start + 1ms);
  gate.ObserveAuthorization((UINT64_C(1) << 32U) | UINT64_C(2), start + 1ms);
  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 1ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 2ms).ready);
  EXPECT_TRUE(gate.Evaluate(true, start + 3ms).ready);

  gate.ObserveFeedback(FeedbackStream::kMotor, true, start + 1501ms);
  const auto heartbeat_stale = gate.Evaluate(true, start + 1502ms);
  EXPECT_FALSE(heartbeat_stale.ready);
  EXPECT_EQ(heartbeat_stale.reason, RecoveryReason::kHeartbeatStale);
  gate.ObserveAuthorization(0U, start + 1503ms);
  EXPECT_FALSE(gate.Evaluate(true, start + 1504ms).ready);
}

TEST(HardwareCommonTest, FirstOrderLowPassUsesMeasuredPeriodAndResets) {
  FirstOrderLowPass filter;
  EXPECT_FALSE(filter.Configure(0.0));
  EXPECT_FALSE(filter.Configure(std::numeric_limits<double>::infinity()));
  ASSERT_TRUE(filter.Configure(5.0));

  const auto first = filter.Update(2.0, 1.0 / 30.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_DOUBLE_EQ(*first, 2.0);

  constexpr double kPeriod = 0.02;
  const double alpha = 1.0 - std::exp(-kTwoPi * 5.0 * kPeriod);
  const auto second = filter.Update(4.0, kPeriod);
  ASSERT_TRUE(second.has_value());
  EXPECT_NEAR(*second, 2.0 + alpha * 2.0, 1.0e-12);

  EXPECT_FALSE(filter.Update(4.0, 0.0).has_value());
  const auto after_invalid = filter.Update(-3.0, kPeriod);
  ASSERT_TRUE(after_invalid.has_value());
  EXPECT_DOUBLE_EQ(*after_invalid, -3.0);

  filter.Reset();
  const auto after_reset = filter.Update(7.0, kPeriod);
  ASSERT_TRUE(after_reset.has_value());
  EXPECT_DOUBLE_EQ(*after_reset, 7.0);
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
