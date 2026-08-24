#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

#include "mentor_pi_mcu/app/microros/reclaiming_arena.h"
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/domain/battery_monitor.h"
#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/button_controller.h"
#include "mentor_pi_mcu/domain/circular_dma_position.h"
#include "mentor_pi_mcu/domain/circular_rx_ring.h"
#include "mentor_pi_mcu/domain/command_mailboxes.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/motor_controller.h"
#include "mentor_pi_mcu/domain/pattern_controller.h"
#include "mentor_pi_mcu/domain/pwm_servo_controller.h"
#include "mentor_pi_mcu/domain/state_merger.h"
#include "mentor_pi_mcu/domain/validation.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"

namespace mentor_pi::mcu {
namespace {

std::uint32_t test_failures = 0;

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    ++test_failures;
    std::cerr << file << ':' << line << ": check failed: " << expression
              << '\n';
  }
}

#define CHECK(expression) \
  Check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

void CheckNear(float actual, float expected, float tolerance,
               const char* expression, const char* file, int line) {
  Check(std::fabs(actual - expected) <= tolerance, expression, file, line);
}

#define CHECK_NEAR(actual, expected, tolerance)                          \
  CheckNear((actual), (expected), (tolerance), #actual " ~= " #expected, \
            __FILE__, __LINE__)

BoundedText MakeText(const char* text) {
  BoundedText result{};
  const std::size_t length = std::strlen(text);
  CHECK(length <= kOledLineCapacity);
  result.size = static_cast<std::uint8_t>(length);
  std::memcpy(result.bytes.data(), text, length);
  result.bytes[length] = '\0';
  return result;
}

MotorControlConfiguration FullRangeTestMotorConfiguration() {
  return DefaultAdrcMotorControlConfiguration();
}

SetMotorAdrcCommand DefaultMotorAdrcCommand(std::uint8_t update_mask) {
  SetMotorAdrcCommand command{};
  command.update_mask = update_mask;
  command.known_velocity_decay_rate_s_inverse.fill(
      kMotorDefaultAdrcKnownVelocityDecayRateSInverse);
  command.input_gain_rps_per_second_per_permille.fill(
      kMotorDefaultAdrcInputGainRpsPerSecondPerPermille);
  command.controller_bandwidth_rad_s.fill(
      kMotorDefaultAdrcControllerBandwidthRadS);
  command.controller_fal_exponent.fill(
      kMotorDefaultAdrcControllerFalExponent);
  command.controller_fal_threshold_rps.fill(
      kMotorDefaultAdrcControllerFalThresholdRps);
  command.observer_bandwidth_rad_s.fill(
      kMotorDefaultAdrcObserverBandwidthRadS);
  command.observer_velocity_fal_exponent.fill(
      kMotorDefaultAdrcObserverVelocityFalExponent);
  command.observer_disturbance_fal_exponent.fill(
      kMotorDefaultAdrcObserverDisturbanceFalExponent);
  command.observer_fal_threshold_rps.fill(
      kMotorDefaultAdrcObserverFalThresholdRps);
  command.disturbance_leakage_s_inverse.fill(
      kMotorDefaultAdrcDisturbanceLeakageSInverse);
  command.disturbance_estimate_limit_rps_per_second.fill(
      kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond);
  command.velocity_filter_new_weight.fill(
      kMotorDefaultVelocityFilterNewWeight);
  command.positive_minimum_drive_permille.fill(
      kMotorDefaultPositiveMinimumDrivePermille);
  command.negative_minimum_drive_permille.fill(
      kMotorDefaultNegativeMinimumDrivePermille);
  return command;
}

template <typename Command>
void PopulateUniqueBusServoIds(Command* command, std::size_t count) {
  command->count = static_cast<std::uint8_t>(count);
  for (std::size_t index = 0U; index < count; ++index) {
    command->servo_id[index] = static_cast<std::uint8_t>(index + 1U);
  }
}

void TestValidationAndStateMerging() {
  MotorCommand motor{};
  motor.update_mask = 1U;
  motor.target_rps[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = 6.0F;
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  motor.update_mask = 0U;
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kInvalidArgument);
  for (std::uint16_t mask = 0; mask <= 0xffU; ++mask) {
    motor.update_mask = static_cast<std::uint8_t>(mask);
    motor.target_rps.fill(0.0F);
    const bool expected_valid = mask >= 1U && mask <= kAllMotorMask;
    CHECK(ValidateMotorCommand(motor, 6.0F).ok() == expected_valid);
  }

  BusServoCommand bus{};
  bus.count = 2U;
  bus.servo_id[0] = 1U;
  bus.servo_id[1] = 1U;
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kInvalidArgument);
  bus.servo_id[1] = 253U;
  bus.position[1] = 1001U;
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kOutOfRange);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.update_mask = ConfigureBusServoCommand::kSetPositionLimits;
  configure.position_min = 900U;
  configure.position_max = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);

  RgbState rgb{};
  RgbCommand rgb_first{};
  rgb_first.update_mask = kHostRgbPixelMask;
  rgb_first.red[kHostRgbPixelIndex] = 10U;
  rgb_first.green[kHostRgbPixelIndex] = 20U;
  rgb_first.blue[kHostRgbPixelIndex] = 30U;
  CHECK(MergeRgbCommand(rgb_first, &rgb).ok());
  RgbCommand rgb_second{};
  rgb_second.update_mask = 1U;
  rgb_second.red[0] = 40U;
  CHECK(MergeRgbCommand(rgb_second, &rgb).code == ResultCode::kInvalidArgument);
  CHECK(rgb.red[kHostRgbPixelIndex] == 10U &&
        rgb.green[kHostRgbPixelIndex] == 20U &&
        rgb.blue[kHostRgbPixelIndex] == 30U);
  CHECK(rgb.red[kStatusRgbPixelIndex] == 0U);

  OledState oled{};
  OledCommand oled_command{};
  oled_command.update_mask = 1U;
  oled_command.lines[0] = MakeText("ready");
  oled_command.lines[1].size = 1U;
  oled_command.lines[1].bytes[0] = static_cast<char>(0x01);
  CHECK(MergeOledCommand(oled_command, &oled).ok());
  CHECK(oled.lines[0].size == 5U);
  oled_command.update_mask = 2U;
  CHECK(MergeOledCommand(oled_command, &oled).code ==
        ResultCode::kInvalidArgument);

  OledCommand byte_command{};
  byte_command.update_mask = 1U;
  byte_command.lines[0].size = 1U;
  byte_command.lines[0].bytes[1] = '\0';
  for (std::uint16_t byte = 0; byte <= 0xffU; ++byte) {
    byte_command.lines[0].bytes[0] = static_cast<char>(byte);
    const bool printable = byte >= 0x20U && byte <= 0x7eU;
    CHECK(ValidateOledCommand(byte_command).ok() == printable);
  }

  RgbCommand rgb_mask{};
  for (std::uint16_t mask = 0; mask <= 0xffU; ++mask) {
    rgb_mask.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask == kHostRgbPixelMask;
    CHECK(ValidateRgbCommand(rgb_mask).ok() == expected_valid);
  }
}

void TestValidationBoundaries() {
  MotorCommand motor{};
  motor.update_mask = 1U;
  motor.target_rps[0] = -6.0F;
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  motor.target_rps[0] = std::nextafter(6.0F, 7.0F);
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = std::nextafter(-6.0F, -7.0F);
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = std::numeric_limits<float>::infinity();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = -std::numeric_limits<float>::infinity();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = 0.0F;
  motor.target_rps[1] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  CHECK(ValidateMotorCommand(motor, 0.0F).code == ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, -1.0F).code ==
        ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, std::numeric_limits<float>::infinity())
            .code == ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, std::numeric_limits<float>::quiet_NaN())
            .code == ResultCode::kInvalidArgument);

  SetMotorAdrcCommand adrc = DefaultMotorAdrcCommand(1U);
  adrc.velocity_filter_new_weight[0] = 1.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.update_mask = 0U;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.update_mask = 0x10U;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.update_mask = 1U;
  adrc.input_gain_rps_per_second_per_permille[0] =
      kMotorAdrcMaximumInputGain;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.input_gain_rps_per_second_per_permille[0] = std::nextafter(
      kMotorAdrcMaximumInputGain, std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.input_gain_rps_per_second_per_permille[0] = 0.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.input_gain_rps_per_second_per_permille[0] =
      std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.input_gain_rps_per_second_per_permille[0] = 0.03F;
  adrc.controller_bandwidth_rad_s[0] =
      kMotorAdrcMaximumObserverBandwidthRadS;
  adrc.observer_bandwidth_rad_s[0] =
      kMotorAdrcMaximumObserverBandwidthRadS;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.observer_bandwidth_rad_s[0] = 12.0F;
  adrc.controller_bandwidth_rad_s[0] = 13.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.controller_bandwidth_rad_s[0] = 4.0F;
  adrc.observer_bandwidth_rad_s[0] =
      std::nextafter(kMotorAdrcMaximumObserverBandwidthRadS,
                     std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.observer_bandwidth_rad_s[0] = -std::numeric_limits<float>::infinity();
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.observer_bandwidth_rad_s[0] = 12.0F;
  adrc.velocity_filter_new_weight[0] = 0.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.velocity_filter_new_weight[0] =
      std::nextafter(1.0F, std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.velocity_filter_new_weight[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.velocity_filter_new_weight[0] = 1.0F;

  adrc.known_velocity_decay_rate_s_inverse[0] =
      kMotorAdrcMaximumKnownVelocityDecayRateSInverse;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.known_velocity_decay_rate_s_inverse[0] = std::nextafter(
      kMotorAdrcMaximumKnownVelocityDecayRateSInverse,
      std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.known_velocity_decay_rate_s_inverse[0] = -0.1F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.known_velocity_decay_rate_s_inverse[0] =
      std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kInvalidArgument);
  adrc.known_velocity_decay_rate_s_inverse[0] = 0.0F;

  adrc.controller_fal_exponent[0] = kMotorAdrcMinimumFalExponent;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.controller_fal_exponent[0] =
      std::nextafter(kMotorAdrcMinimumFalExponent, 0.0F);
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.controller_fal_exponent[0] =
      std::nextafter(kMotorAdrcMaximumFalExponent,
                     std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.controller_fal_exponent[0] = 1.0F;
  adrc.controller_fal_threshold_rps[0] = kMotorAdrcMinimumFalThresholdRps;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.controller_fal_threshold_rps[0] =
      std::nextafter(kMotorAdrcMinimumFalThresholdRps, 0.0F);
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.controller_fal_threshold_rps[0] = kMotorAdrcMaximumFalThresholdRps;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.controller_fal_threshold_rps[0] =
      std::nextafter(kMotorAdrcMaximumFalThresholdRps,
                     std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.controller_fal_threshold_rps[0] =
      kMotorDefaultAdrcControllerFalThresholdRps;

  adrc.observer_velocity_fal_exponent[0] = kMotorAdrcMinimumFalExponent;
  adrc.observer_disturbance_fal_exponent[0] = kMotorAdrcMinimumFalExponent;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.observer_velocity_fal_exponent[0] = 0.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.observer_velocity_fal_exponent[0] = 1.0F;
  adrc.observer_disturbance_fal_exponent[0] =
      std::numeric_limits<float>::infinity();
  CHECK(ValidateSetMotorAdrcCommand(adrc).code ==
        ResultCode::kInvalidArgument);
  adrc.observer_disturbance_fal_exponent[0] = 1.0F;
  adrc.observer_fal_threshold_rps[0] = kMotorAdrcMinimumFalThresholdRps;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.observer_fal_threshold_rps[0] = kMotorAdrcMaximumFalThresholdRps;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.observer_fal_threshold_rps[0] = 0.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.observer_fal_threshold_rps[0] =
      kMotorDefaultAdrcObserverFalThresholdRps;

  adrc.disturbance_leakage_s_inverse[0] =
      kMotorAdrcMaximumDisturbanceLeakageSInverse;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.disturbance_leakage_s_inverse[0] = std::nextafter(
      kMotorAdrcMaximumDisturbanceLeakageSInverse,
      std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.disturbance_leakage_s_inverse[0] = -0.1F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.disturbance_leakage_s_inverse[0] = 0.0F;
  adrc.disturbance_estimate_limit_rps_per_second[0] = 0.0F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.disturbance_estimate_limit_rps_per_second[0] =
      adrc.input_gain_rps_per_second_per_permille[0] *
      kMotorAdrcHardOutputLimitPermille;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.disturbance_estimate_limit_rps_per_second[0] = std::nextafter(
      adrc.input_gain_rps_per_second_per_permille[0] *
          kMotorAdrcHardOutputLimitPermille,
      std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.disturbance_estimate_limit_rps_per_second[0] = -0.1F;
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.disturbance_estimate_limit_rps_per_second[0] =
      kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond;

  adrc.positive_minimum_drive_permille[0] =
      kMotorAdrcMaximumMinimumDrivePermille;
  adrc.negative_minimum_drive_permille[0] =
      kMotorAdrcMaximumMinimumDrivePermille;
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());
  adrc.positive_minimum_drive_permille[0] =
      static_cast<std::uint16_t>(kMotorAdrcMaximumMinimumDrivePermille + 1U);
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.positive_minimum_drive_permille[0] = 0U;
  adrc.negative_minimum_drive_permille[0] =
      static_cast<std::uint16_t>(kMotorAdrcMaximumMinimumDrivePermille + 1U);
  CHECK(ValidateSetMotorAdrcCommand(adrc).code == ResultCode::kOutOfRange);
  adrc.negative_minimum_drive_permille[0] = 0U;

  // Values outside the selected mask are not part of the atomic request.
  adrc.controller_fal_exponent[1] =
      std::numeric_limits<float>::quiet_NaN();
  adrc.positive_minimum_drive_permille[1] =
      std::numeric_limits<std::uint16_t>::max();
  CHECK(ValidateSetMotorAdrcCommand(adrc).ok());

  PwmServoCommand pwm{};
  pwm.duration_ms = 20U;
  pwm.pulse_width_us.fill(500U);
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    pwm.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllPwmServoMask;
    CHECK(ValidatePwmServoCommand(pwm).ok() == expected_valid);
  }
  pwm.update_mask = 4U;
  pwm.duration_ms = 19U;
  CHECK(ValidatePwmServoCommand(pwm).code == ResultCode::kOutOfRange);
  pwm.duration_ms = 30000U;
  pwm.pulse_width_us[2] = 2500U;
  CHECK(ValidatePwmServoCommand(pwm).ok());
  pwm.duration_ms = 30001U;
  CHECK(ValidatePwmServoCommand(pwm).code == ResultCode::kOutOfRange);
  pwm.duration_ms = 20U;
  pwm.pulse_width_us[2] = 499U;
  CHECK(ValidatePwmServoCommand(pwm).detail == 3U);
  pwm.pulse_width_us[2] = 2501U;
  CHECK(ValidatePwmServoCommand(pwm).detail == 3U);
  pwm.pulse_width_us[2] = 500U;
  pwm.pulse_width_us[3] = 65535U;
  CHECK(ValidatePwmServoCommand(pwm).ok());

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 8U;
  offsets.offset_us[3] = -100;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  offsets.offset_us[3] = 100;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  offsets.offset_us[3] = -101;
  CHECK(ValidatePwmServoOffsets(offsets).detail == 4U);
  offsets.offset_us[3] = 101;
  CHECK(ValidatePwmServoOffsets(offsets).detail == 4U);
  offsets.update_mask = 1U;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    PwmServoOffsetCommand candidate{};
    candidate.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllPwmServoMask;
    CHECK(ValidatePwmServoOffsets(candidate).ok() == expected_valid);
  }

  BusServoCommand bus{};
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kInvalidArgument);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.position.fill(1000U);
  bus.duration_ms = 30000U;
  CHECK(ValidateBusServoCommand(bus).ok());
  bus.count = static_cast<std::uint8_t>(kBusServoBatchCapacity + 1U);
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity + 1U);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.servo_id[0] = 0U;
  CHECK(ValidateBusServoCommand(bus).detail == 1U);
  bus.servo_id[0] = 254U;
  CHECK(ValidateBusServoCommand(bus).detail == 1U);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.servo_id[kBusServoBatchCapacity - 1U] = 1U;
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.position[kBusServoBatchCapacity - 1U] = 1001U;
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity);
  bus.position[kBusServoBatchCapacity - 1U] = 1000U;
  bus.duration_ms = 30001U;
  CHECK(ValidateBusServoCommand(bus).detail == 0U);

  StopBusServosCommand stop{};
  CHECK(ValidateStopBusServosCommand(stop).code ==
        ResultCode::kInvalidArgument);
  PopulateUniqueBusServoIds(&stop, kBusServoBatchCapacity);
  CHECK(ValidateStopBusServosCommand(stop).ok());
  stop.count = static_cast<std::uint8_t>(kBusServoBatchCapacity + 1U);
  CHECK(ValidateStopBusServosCommand(stop).detail ==
        kBusServoBatchCapacity + 1U);
  PopulateUniqueBusServoIds(&stop, kBusServoBatchCapacity);
  stop.servo_id[kBusServoBatchCapacity - 1U] = 1U;
  CHECK(ValidateStopBusServosCommand(stop).detail == kBusServoBatchCapacity);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.new_id = 253U;
  configure.offset = 125;
  configure.position_min = 0U;
  configure.position_max = 1000U;
  configure.voltage_min_mv = 4500U;
  configure.voltage_max_mv = 14000U;
  configure.temperature_limit_c = 100U;
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    configure.update_mask = mask;
    const bool expected_valid =
        mask >= 1U && mask <= ConfigureBusServoCommand::kAllUpdates;
    CHECK(ValidateConfigureBusServoCommand(configure).ok() == expected_valid);
  }
  configure.update_mask = ConfigureBusServoCommand::kSetId;
  configure.servo_id = 0U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 0U);
  configure.servo_id = 254U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 0U);
  configure.servo_id = 1U;
  configure.new_id = 0U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 1U);
  configure.new_id = 254U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 1U);
  configure.update_mask = ConfigureBusServoCommand::kSetOffset;
  configure.offset = -125;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.offset = -126;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 2U);
  configure.offset = 126;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 2U);
  configure.update_mask = ConfigureBusServoCommand::kSetPositionLimits;
  configure.position_min = 1000U;
  configure.position_max = 1000U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.position_max = 1001U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kOutOfRange);
  configure.position_min = 900U;
  configure.position_max = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);
  configure.update_mask = ConfigureBusServoCommand::kSetVoltageLimits;
  configure.voltage_min_mv = 4500U;
  configure.voltage_max_mv = 14000U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.voltage_min_mv = 4499U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kOutOfRange);
  configure.voltage_min_mv = 14000U;
  configure.voltage_max_mv = 13999U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);
  configure.update_mask = ConfigureBusServoCommand::kSetTemperatureLimit;
  configure.temperature_limit_c = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.temperature_limit_c = 101U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 6U);

  GetBusServoStateCommand get{};
  get.servo_id = 1U;
  for (std::uint16_t fields = 0U; fields <= 0x03ffU; ++fields) {
    get.fields = fields;
    const bool expected_valid =
        fields >= 1U && fields <= GetBusServoStateCommand::kAllFields;
    CHECK(ValidateGetBusServoStateCommand(get).ok() == expected_valid);
  }
  get.fields = GetBusServoStateCommand::kFieldId;
  get.servo_id = 253U;
  CHECK(ValidateGetBusServoStateCommand(get).ok());
  get.servo_id = 254U;
  CHECK(ValidateGetBusServoStateCommand(get).ok());
  get.fields = 2U;
  CHECK(ValidateGetBusServoStateCommand(get).code ==
        ResultCode::kInvalidArgument);
  get.fields = GetBusServoStateCommand::kFieldId;
  get.servo_id = 0U;
  CHECK(ValidateGetBusServoStateCommand(get).code == ResultCode::kOutOfRange);
  get.servo_id = 255U;
  CHECK(ValidateGetBusServoStateCommand(get).code == ResultCode::kOutOfRange);

  CHECK(ValidateLedCommand({1U, 0U, 0U, 0U}).code ==
        ResultCode::kOutOfRange);
  CHECK(ValidateLedCommand({2U, 0U, 0U, 0U}).ok());
  CHECK(ValidateLedCommand({3U, 0U, 0U, 0U}).ok());
  CHECK(ValidateLedCommand({0U, 0U, 0U, 0U}).code == ResultCode::kOutOfRange);
  CHECK(ValidateLedCommand({4U, 0U, 0U, 0U}).code == ResultCode::kOutOfRange);
  CHECK(ValidateBuzzerCommand({0U, 65535U, 65535U, 65535U}).ok());
  CHECK(ValidateBuzzerCommand({65535U, 0U, 65535U, 65535U}).ok());
  CHECK(ValidateBuzzerCommand({10U, 1U, 0U, 0U}).ok());
  CHECK(ValidateBuzzerCommand({20000U, 1U, 0U, 0U}).ok());
  CHECK(ValidateBuzzerCommand({9U, 1U, 0U, 0U}).code ==
        ResultCode::kOutOfRange);
  CHECK(ValidateBuzzerCommand({20001U, 1U, 0U, 0U}).code ==
        ResultCode::kOutOfRange);

  OledCommand oled{};
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    oled.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllOledLineMask;
    CHECK(ValidateOledCommand(oled).ok() == expected_valid);
  }
  oled.update_mask = 1U;
  oled.lines[0].size = static_cast<std::uint8_t>(kOledLineCapacity);
  oled.lines[0].bytes.fill('~');
  oled.lines[0].bytes[kOledLineCapacity] = '\0';
  CHECK(ValidateOledCommand(oled).ok());
  oled.lines[0].size = static_cast<std::uint8_t>(kOledLineCapacity + 1U);
  CHECK(ValidateOledCommand(oled).code == ResultCode::kInvalidArgument);
  oled.lines[0] = MakeText("A");
  oled.lines[0].bytes[1] = 'X';
  CHECK(ValidateOledCommand(oled).code == ResultCode::kInvalidArgument);
  oled.lines[0] = MakeText("A");
  oled.lines[1].size = static_cast<std::uint8_t>(kOledLineCapacity + 1U);
  CHECK(ValidateOledCommand(oled).ok());

  RgbCommand invalid_rgb{};
  RgbState rgb_state{};
  rgb_state.red = {10U, 20U};
  const RgbState rgb_before = rgb_state;
  CHECK(MergeRgbCommand(invalid_rgb, &rgb_state).code ==
        ResultCode::kInvalidArgument);
  CHECK(rgb_state.red == rgb_before.red);
  invalid_rgb.update_mask = 1U;
  CHECK(MergeRgbCommand(invalid_rgb, nullptr).code ==
        ResultCode::kInvalidArgument);

  OledCommand invalid_oled{};
  invalid_oled.update_mask = 1U;
  invalid_oled.lines[0].size = 1U;
  invalid_oled.lines[0].bytes[0] = static_cast<char>(0x1f);
  OledState oled_state{};
  oled_state.lines[0] = MakeText("unchanged");
  const OledState oled_before = oled_state;
  CHECK(MergeOledCommand(invalid_oled, &oled_state).code ==
        ResultCode::kInvalidArgument);
  CHECK(oled_state.lines[0].size == oled_before.lines[0].size);
  CHECK(oled_state.lines[0].bytes == oled_before.lines[0].bytes);
  CHECK(MergeOledCommand(invalid_oled, nullptr).code ==
        ResultCode::kInvalidArgument);

  CHECK(ValidateBatteryThreshold(4999U).code == ResultCode::kOutOfRange);
  CHECK(ValidateBatteryThreshold(5000U).ok());
  CHECK(ValidateBatteryThreshold(20000U).ok());
  CHECK(ValidateBatteryThreshold(20001U).code == ResultCode::kOutOfRange);
  for (std::uint16_t value = 0U; value <= 0xffU; ++value) {
    const auto model = static_cast<MotorModel>(value);
    CHECK(IsValidMotorModel(model) == (value <= 3U));
  }
}

void TestFixedContainers() {
  SaturatingCounter<std::uint8_t> counter{254U};
  counter.Increment();
  counter.Increment();
  CHECK(counter.value() == 255U);
  SaturatingCounter<std::uint32_t> diagnostic_counter{
      std::numeric_limits<std::uint32_t>::max() - 1U};
  diagnostic_counter.Increment();
  diagnostic_counter.Increment();
  CHECK(diagnostic_counter.value() ==
        std::numeric_limits<std::uint32_t>::max());

  LatestMailbox<std::uint32_t> mailbox;
  CHECK(!mailbox.Publish(10U));
  CHECK(mailbox.Publish(20U));
  std::uint32_t value = 0;
  CHECK(mailbox.ConsumeLatest(&value));
  CHECK(value == 20U);
  CHECK(!mailbox.ConsumeLatest(&value));
  CHECK(!mailbox.DiscardLatest());
  CHECK(!mailbox.Publish(30U));
  CHECK(mailbox.has_unread());
  CHECK(mailbox.DiscardLatest());
  CHECK(!mailbox.has_unread());
  CHECK(!mailbox.ConsumeLatest(&value));
  CHECK(!mailbox.Publish(40U));
  CHECK(mailbox.ConsumeLatest(&value));
  CHECK(value == 40U);

  DropOldestQueue<std::uint32_t, 16> queue;
  for (std::uint32_t item = 1; item <= 32U; ++item) {
    queue.PushDropOldest(item);
  }
  CHECK(queue.dropped_count() == 16U);
  for (std::uint32_t expected = 17U; expected <= 32U; ++expected) {
    value = 0;
    CHECK(queue.TryPop(&value));
    CHECK(value == expected);
  }
  CHECK(!queue.TryPop(&value));
}

void TestCommandMailboxes() {
  MotorCommandMailbox motors;
  MotorCommand first{};
  first.update_mask = 1U;
  first.target_rps[0] = 1.0F;
  CHECK(motors.Publish(first, 6.0F, 100U).result.ok());
  MotorCommand second{};
  second.update_mask = 2U;
  second.target_rps[1] = 2.0F;
  const CommandAdmission second_admission = motors.Publish(second, 6.0F, 200U);
  CHECK(second_admission.result.ok() && second_admission.overwrote_unread);
  CHECK(motors.overwrite_count() == 1U);
  MotorCommandSnapshot motor_snapshot{};
  CHECK(motors.ConsumeLatest(&motor_snapshot));
  CHECK_NEAR(motor_snapshot.target_rps[0], 1.0F, 0.0001F);
  CHECK_NEAR(motor_snapshot.target_rps[1], 2.0F, 0.0001F);
  CHECK(motor_snapshot.accepted_at_us[0] == 100U);
  CHECK(motor_snapshot.accepted_at_us[1] == 200U);
  CHECK(motor_snapshot.field_generation[0] == 1U);
  CHECK(motor_snapshot.field_generation[1] == 2U);

  MotorCommand invalid = second;
  invalid.target_rps[1] = 7.0F;
  CHECK(motors.Publish(invalid, 6.0F, 300U).result.code ==
        ResultCode::kOutOfRange);
  CHECK(!motors.ConsumeLatest(&motor_snapshot));

  PwmCommandMailbox pwm;
  PwmServoCommand pwm_first{};
  pwm_first.update_mask = 1U;
  pwm_first.duration_ms = 40U;
  pwm_first.pulse_width_us[0] = 1000U;
  CHECK(pwm.Publish(pwm_first).result.ok());
  PwmServoCommand pwm_second{};
  pwm_second.update_mask = 2U;
  pwm_second.duration_ms = 60U;
  pwm_second.pulse_width_us[1] = 2000U;
  CHECK(pwm.Publish(pwm_second).result.ok());
  PwmCommandSnapshot pwm_snapshot{};
  CHECK(pwm.ConsumeLatest(&pwm_snapshot));
  CHECK(pwm_snapshot.pulse_width_us[0] == 1000U);
  CHECK(pwm_snapshot.duration_ms[0] == 40U);
  CHECK(pwm_snapshot.pulse_width_us[1] == 2000U);
  CHECK(pwm_snapshot.duration_ms[1] == 60U);

  LedCommandMailbox leds;
  CHECK(leds.Publish({2U, 30U, 40U, 2U}).result.ok());
  CHECK(leds.Publish({3U, 50U, 60U, 3U}).result.ok());
  LedCommandSnapshot led_snapshot{};
  CHECK(leds.ConsumeLatest(&led_snapshot));
  CHECK(led_snapshot.commands[1].on_time_ms == 30U);
  CHECK(led_snapshot.commands[2].on_time_ms == 50U);
  CHECK(led_snapshot.commands[0].on_time_ms == 0U);

  BusMotionMailbox bus;
  BusServoCommand bus_first{};
  bus_first.count = 1U;
  bus_first.servo_id[0] = 1U;
  bus_first.position[0] = 100U;
  BusServoCommand bus_second = bus_first;
  bus_second.position[0] = 200U;
  CHECK(bus.Publish(bus_first).result.ok());
  CHECK(bus.Publish(bus_second).overwrote_unread);
  BusMotionSnapshot bus_snapshot{};
  CHECK(bus.ConsumeLatest(&bus_snapshot));
  CHECK(bus_snapshot.command.position[0] == 200U);

  BuzzerCommandMailbox buzzer;
  CHECK(buzzer.Publish({440U, 10U, 10U, 1U}).result.ok());
  CHECK(buzzer.Publish({880U, 20U, 20U, 2U}).overwrote_unread);
  BuzzerCommandSnapshot buzzer_snapshot{};
  CHECK(buzzer.ConsumeLatest(&buzzer_snapshot));
  CHECK(buzzer_snapshot.command.frequency_hz == 880U);

  RgbCommandMailbox rgb;
  RgbCommand rgb_full{};
  rgb_full.update_mask = kHostRgbPixelMask;
  rgb_full.red = {10U, 20U};
  rgb_full.green = {30U, 40U};
  rgb_full.blue = {50U, 60U};
  const CommandAdmission rgb_first = rgb.Publish(rgb_full);
  CHECK(rgb_first.result.ok());
  CHECK(!rgb_first.overwrote_unread);
  CHECK(rgb_first.generation == 1U);
  RgbCommand rgb_update{};
  rgb_update.update_mask = kHostRgbPixelMask;
  rgb_update.red[kHostRgbPixelIndex] = 70U;
  rgb_update.green[kHostRgbPixelIndex] = 80U;
  rgb_update.blue[kHostRgbPixelIndex] = 90U;
  const CommandAdmission rgb_second = rgb.Publish(rgb_update);
  CHECK(rgb_second.result.ok());
  CHECK(rgb_second.overwrote_unread);
  CHECK(rgb_second.generation == 2U);
  rgb_update.red[kHostRgbPixelIndex] = 100U;
  rgb_update.green[kHostRgbPixelIndex] = 110U;
  rgb_update.blue[kHostRgbPixelIndex] = 120U;
  const CommandAdmission rgb_third = rgb.Publish(rgb_update);
  CHECK(rgb_third.result.ok());
  CHECK(rgb_third.overwrote_unread);
  CHECK(rgb_third.generation == 3U);
  CHECK(rgb.overwrite_count() == 2U);
  RgbCommandSnapshot rgb_snapshot{};
  CHECK(rgb.ConsumeLatest(&rgb_snapshot));
  CHECK(rgb_snapshot.state.red[kHostRgbPixelIndex] == 100U);
  CHECK(rgb_snapshot.state.green[kHostRgbPixelIndex] == 110U);
  CHECK(rgb_snapshot.state.blue[kHostRgbPixelIndex] == 120U);
  CHECK(rgb_snapshot.state.red[kStatusRgbPixelIndex] == 0U);
  CHECK(rgb_snapshot.state.green[kStatusRgbPixelIndex] == 0U);
  CHECK(rgb_snapshot.state.blue[kStatusRgbPixelIndex] == 0U);
  CHECK(!rgb.ConsumeLatest(&rgb_snapshot));
  RgbCommand invalid_rgb{};
  const CommandAdmission rejected_rgb = rgb.Publish(invalid_rgb);
  CHECK(rejected_rgb.result.code == ResultCode::kInvalidArgument);
  CHECK(rejected_rgb.generation == 3U);
  CHECK(!rejected_rgb.overwrote_unread);
  CHECK(rgb.overwrite_count() == 2U);
  CHECK(!rgb.ConsumeLatest(&rgb_snapshot));

  OledCommandMailbox oled;
  OledCommand oled_full{};
  oled_full.update_mask = kAllOledLineMask;
  oled_full.lines[0] = MakeText("first");
  oled_full.lines[1] = MakeText("retained");
  const CommandAdmission oled_first = oled.Publish(oled_full);
  CHECK(oled_first.result.ok());
  CHECK(!oled_first.overwrote_unread);
  CHECK(oled_first.generation == 1U);
  OledCommand oled_update{};
  oled_update.update_mask = 1U;
  oled_update.lines[0] = MakeText("second");
  const CommandAdmission oled_second = oled.Publish(oled_update);
  CHECK(oled_second.result.ok());
  CHECK(oled_second.overwrote_unread);
  CHECK(oled_second.generation == 2U);
  oled_update.lines[0] = MakeText("newest");
  const CommandAdmission oled_third = oled.Publish(oled_update);
  CHECK(oled_third.result.ok());
  CHECK(oled_third.overwrote_unread);
  CHECK(oled_third.generation == 3U);
  CHECK(oled.overwrite_count() == 2U);
  OledCommandSnapshot oled_snapshot{};
  CHECK(oled.ConsumeLatest(&oled_snapshot));
  CHECK(oled_snapshot.state.lines[0].size == 6U);
  CHECK(std::memcmp(oled_snapshot.state.lines[0].bytes.data(), "newest", 6U) ==
        0);
  CHECK(oled_snapshot.state.lines[1].size == 8U);
  CHECK(std::memcmp(oled_snapshot.state.lines[1].bytes.data(), "retained",
                    8U) == 0);
  CHECK(!oled.ConsumeLatest(&oled_snapshot));
  OledCommand invalid_oled{};
  const CommandAdmission rejected_oled = oled.Publish(invalid_oled);
  CHECK(rejected_oled.result.code == ResultCode::kInvalidArgument);
  CHECK(rejected_oled.generation == 3U);
  CHECK(!rejected_oled.overwrote_unread);
  CHECK(oled.overwrite_count() == 2U);
  CHECK(!oled.ConsumeLatest(&oled_snapshot));

  // A session boundary discards unread merged fields without resetting the
  // monotonic mailbox generation. The first disjoint update in the next
  // session therefore contains only fields accepted after the reset.
  CHECK(motors.Publish(first, 6.0F, 400U).result.ok());
  motors.ResetMergedFields();
  CHECK(motors.Publish(second, 6.0F, 500U).result.ok());
  CHECK(motors.ConsumeLatest(&motor_snapshot));
  CHECK(motor_snapshot.field_generation[0] == 0U);
  CHECK(motor_snapshot.target_rps[0] == 0.0F);
  CHECK(motor_snapshot.field_generation[1] == 4U);

  CHECK(pwm.Publish(pwm_first).result.ok());
  pwm.ResetMergedFields();
  CHECK(pwm.Publish(pwm_second).result.ok());
  CHECK(pwm.ConsumeLatest(&pwm_snapshot));
  CHECK(pwm_snapshot.field_generation[0] == 0U);
  CHECK(pwm_snapshot.pulse_width_us[0] == 1500U);
  CHECK(pwm_snapshot.field_generation[1] == 4U);

  CHECK(leds.Publish({2U, 10U, 20U, 1U}).result.ok());
  leds.ResetMergedFields();
  CHECK(leds.Publish({3U, 30U, 40U, 2U}).result.ok());
  CHECK(leds.ConsumeLatest(&led_snapshot));
  CHECK(led_snapshot.field_generation[0] == 0U);
  CHECK(led_snapshot.field_generation[1] == 0U);
  CHECK(led_snapshot.field_generation[2] == 4U);

  CHECK(rgb.Publish(rgb_update).result.ok());
  rgb.ResetMergedFields();
  RgbCommand rgb_new_session{};
  rgb_new_session.update_mask = kHostRgbPixelMask;
  rgb_new_session.blue[kHostRgbPixelIndex] = 200U;
  CHECK(rgb.Publish(rgb_new_session).result.ok());
  CHECK(rgb.ConsumeLatest(&rgb_snapshot));
  CHECK(rgb_snapshot.field_generation[kHostRgbPixelIndex] == 5U);
  CHECK(rgb_snapshot.state.red[kHostRgbPixelIndex] == 0U);
  CHECK(rgb_snapshot.state.blue[kHostRgbPixelIndex] == 200U);
  CHECK(rgb_snapshot.field_generation[kStatusRgbPixelIndex] == 0U);
  CHECK(rgb_snapshot.state.blue[kStatusRgbPixelIndex] == 0U);

  CHECK(oled.Publish(oled_update).result.ok());
  oled.ResetMergedFields();
  OledCommand oled_new_session{};
  oled_new_session.update_mask = 2U;
  oled_new_session.lines[1] = MakeText("new session");
  CHECK(oled.Publish(oled_new_session).result.ok());
  CHECK(oled.ConsumeLatest(&oled_snapshot));
  CHECK(oled_snapshot.field_generation[0] == 0U);
  CHECK(oled_snapshot.state.lines[0].size == 0U);
  CHECK(oled_snapshot.field_generation[1] == 5U);
  CHECK(oled_snapshot.state.lines[1].size == 11U);
}

void TestMotorController() {
  MotorController controller(FullRangeTestMotorConfiguration());
  CHECK(controller.profile().model == MotorModel::kJga27);
  controller.SetSessionActive(true);

  MotorCommand command{};
  command.update_mask = 1U;
  command.target_rps[0] = 1.0F;
  CHECK(controller.AcceptCommand(command, 1000U).ok());

  MotorCommand invalid{};
  invalid.update_mask = 3U;
  invalid.target_rps[0] = 1.0F;
  invalid.target_rps[1] = 7.0F;
  CHECK(controller.AcceptCommand(invalid, 100000U).code ==
        ResultCode::kOutOfRange);
  CHECK_NEAR(controller.channels()[0].target_rps, 1.0F, 0.0001F);

  controller.EvaluateLeases(198999U);
  CHECK(controller.channels()[0].armed);
  controller.EvaluateLeases(199000U);
  CHECK(!controller.channels()[0].armed);
  CHECK(controller.watchdog_stop_mask() == 1U);
  CHECK(controller.lease_expiry_count(0) == 1U);

  command.target_rps[0] = 0.5F;
  CHECK(controller.AcceptCommand(command, 0xfffffff0U).ok());
  controller.EvaluateLeases(0x00000020U);
  CHECK(controller.channels()[0].armed);
  CHECK(controller.AcceptCommand(command, 200U).ok());
  CHECK(controller.SetModel(MotorModel::kJgb37).result.code ==
        ResultCode::kBusy);
  MotorCommand stop = command;
  stop.target_rps[0] = 0.0F;
  CHECK(controller.AcceptCommand(stop, 300U).ok());
  CHECK(controller.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(controller.profile().ticks_per_revolution == 1980U);

  MotorController model_reset(FullRangeTestMotorConfiguration());
  std::array<std::uint32_t, kMotorCount> model_counters{};
  model_reset.ControlStep(model_counters);
  model_counters[0] = 10U;
  model_reset.ControlStep(model_counters);
  CHECK(std::fabs(model_reset.channels()[0].measured_rps) > 0.0F);
  CHECK(model_reset.SetModel(MotorModel::kJgb37).result.ok());
  CHECK_NEAR(model_reset.channels()[0].measured_rps, 0.0F, 0.0001F);
  const std::int64_t count_before_rebaseline =
      model_reset.channels()[0].encoder_count;
  model_reset.ControlStep(model_counters);
  CHECK_NEAR(model_reset.channels()[0].measured_rps, 0.0F, 0.0001F);
  CHECK(model_reset.channels()[0].encoder_count == count_before_rebaseline);

  CHECK(MotorController::SignedCounterDelta(2U, 65534U, 16U) == 4);
  CHECK(MotorController::SignedCounterDelta(65534U, 2U, 16U) == -4);
  CHECK(MotorController::SignedCounterDelta(1U, 0xffffffffU, 32U) == 2);

  MotorCommand drive{};
  drive.update_mask = 1U;
  drive.target_rps[0] = 1.0F;
  CHECK(controller.AcceptCommand(drive, 400U).ok());
  const std::array<std::uint32_t, kMotorCount> counters{};
  const auto outputs = controller.ControlStep(counters);
  CHECK(outputs[0] == -133);
  CHECK(std::abs(outputs[0]) <= kMotorOutputLimitPermille);
  controller.SetSessionActive(false);
  CHECK(controller.channels()[0].output_permille == 0);
  CHECK(controller.watchdog_stop_mask() == 1U);

  MotorController independent(FullRangeTestMotorConfiguration());
  independent.SetSessionActive(true);
  MotorCommand both{};
  both.update_mask = 3U;
  both.target_rps[0] = 1.0F;
  both.target_rps[1] = 1.0F;
  CHECK(independent.AcceptCommand(both, 100U).ok());
  MotorCommand refresh_first{};
  refresh_first.update_mask = 1U;
  refresh_first.target_rps[0] = 1.0F;
  CHECK(independent.AcceptCommand(refresh_first, 100000U).ok());
  independent.EvaluateLeases(198100U);
  CHECK(independent.channels()[0].armed);
  CHECK(!independent.channels()[1].armed);
  CHECK(independent.watchdog_stop_mask() == 2U);
  independent.EvaluateLeases(297999U);
  CHECK(independent.channels()[0].armed);
  independent.EvaluateLeases(298000U);
  CHECK(!independent.channels()[0].armed);

  MotorController zero_target;
  zero_target.SetSessionActive(true);
  MotorCommand zero{};
  zero.update_mask = 1U;
  zero.target_rps[0] = 0.0F;
  CHECK(zero_target.AcceptCommand(zero, 10U).ok());
  zero_target.EvaluateLeases(1000000U);
  CHECK(zero_target.watchdog_stop_mask() == 0U);

  // The only default configuration admits LADRC motion for every valid subset
  // while enforcing the active model limit and atomic stop semantics.
  constexpr std::array<MotorModel, 4> kModels{
      MotorModel::kJgb520, MotorModel::kJgb37, MotorModel::kJga27,
      MotorModel::kJgb528};
  MotorController default_adrc;
  default_adrc.SetSessionActive(true);
  for (MotorModel model : kModels) {
    CHECK(default_adrc.SetModel(model).result.ok());
    for (std::uint16_t raw_mask = 1U; raw_mask <= kAllMotorMask; ++raw_mask) {
      MotorCommand requested{};
      requested.update_mask = static_cast<std::uint8_t>(raw_mask);
      MotorCommand zero_selected{};
      zero_selected.update_mask = requested.update_mask;
      for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
        const auto bit = static_cast<std::uint8_t>(1U << motor);
        if ((requested.update_mask & bit) != 0U) {
          requested.target_rps[motor] = 0.1F;
        }
      }
      CHECK(default_adrc.AcceptCommand(requested, 1000U).ok());
      for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
        const auto bit = static_cast<std::uint8_t>(1U << motor);
        CHECK(default_adrc.channels()[motor].armed ==
              ((requested.update_mask & bit) != 0U));
      }
      CHECK(default_adrc.AcceptCommand(zero_selected, 2000U).ok());
    }
  }
  MotorCommand over_limit{};
  over_limit.update_mask = 3U;
  over_limit.target_rps[0] = kMotorImplementationMaximumRps + 0.01F;
  over_limit.target_rps[1] = 0.1F;
  CHECK(default_adrc.AcceptCommand(over_limit, 3000U).code ==
        ResultCode::kOutOfRange);
  for (const MotorChannelState& channel : default_adrc.channels()) {
    CHECK(!channel.armed);
    CHECK_NEAR(channel.target_rps, 0.0F, 0.0001F);
  }

  MotorController rejection_accounting;
  rejection_accounting.RecordRejectedCommand(0U);
  rejection_accounting.RecordRejectedCommand(0x10U);
  rejection_accounting.RecordRejectedCommand(0x15U);
  for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
    CHECK(rejection_accounting.command_rejection_count(motor) == 0U);
  }
  rejection_accounting.RecordRejectedCommand(0x05U);
  CHECK(rejection_accounting.command_rejection_count(0U) == 1U);
  CHECK(rejection_accounting.command_rejection_count(1U) == 0U);
  CHECK(rejection_accounting.command_rejection_count(2U) == 1U);
  CHECK(rejection_accounting.command_rejection_count(3U) == 0U);
  SetMotorAdrcCommand adrc_update = DefaultMotorAdrcCommand(3U);
  adrc_update.input_gain_rps_per_second_per_permille[0] = 0.005F;
  adrc_update.input_gain_rps_per_second_per_permille[1] = 0.01F;
  adrc_update.disturbance_estimate_limit_rps_per_second[0] = 5.0F;
  adrc_update.disturbance_estimate_limit_rps_per_second[1] = 10.0F;
  adrc_update.controller_bandwidth_rad_s[0] = 4.0F;
  adrc_update.controller_bandwidth_rad_s[1] = 4.0F;
  adrc_update.observer_bandwidth_rad_s[0] = 12.0F;
  adrc_update.observer_bandwidth_rad_s[1] = 12.0F;
  adrc_update.velocity_filter_new_weight[0] = 1.0F;
  adrc_update.velocity_filter_new_weight[1] = 1.0F;
  MotorControlConfiguration invalid_configuration{};
  invalid_configuration.maximum_accepted_rps = 0.0F;
  MotorController invalid_controller(invalid_configuration);
  CHECK(!invalid_controller.configuration_valid());
  CHECK(invalid_controller.SetAdrc(adrc_update).result.code ==
        ResultCode::kUnsupported);

  MotorController adrc_controller(FullRangeTestMotorConfiguration());
  MotorAdrcUpdate adrc_result = adrc_controller.SetAdrc(adrc_update);
  CHECK(adrc_result.result.ok());
  CHECK(adrc_result.applied_mask == 3U);

  SetMotorAdrcCommand invalid_atomic = adrc_update;
  invalid_atomic.controller_fal_threshold_rps[1] = 0.0F;
  adrc_result = adrc_controller.SetAdrc(invalid_atomic);
  CHECK(adrc_result.result.code == ResultCode::kOutOfRange);
  CHECK(adrc_result.applied_mask == 0U);

  adrc_controller.SetSessionActive(true);
  MotorCommand adrc_drive{};
  adrc_drive.update_mask = 3U;
  adrc_drive.target_rps[0] = 0.5F;
  adrc_drive.target_rps[1] = 0.5F;
  CHECK(adrc_controller.AcceptCommand(adrc_drive, 0U).ok());
  std::array<std::uint32_t, kMotorCount> stationary{};
  const auto overridden_output = adrc_controller.ControlStep(stationary);
  CHECK(overridden_output[0] == -400);
  CHECK(overridden_output[1] == -200);
  CHECK(adrc_controller.SetAdrc(adrc_update).result.code == ResultCode::kBusy);

  MotorCommand adrc_stop = adrc_drive;
  adrc_stop.target_rps.fill(0.0F);
  CHECK(adrc_controller.AcceptCommand(adrc_stop, 1U).ok());
  CHECK(adrc_controller.SetAdrc(adrc_update).result.ok());

  // A transport/session loss disarms and clears LADRC state, but the volatile
  // gain override survives reconnection until reset or a model change.
  adrc_controller.SetSessionActive(false);
  adrc_controller.SetSessionActive(true);
  CHECK(adrc_controller.AcceptCommand(adrc_drive, 2U).ok());
  CHECK(adrc_controller.ControlStep(stationary)[0] == -400);
  CHECK(adrc_controller.AcceptCommand(adrc_stop, 3U).ok());
  CHECK(adrc_controller.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(adrc_controller.AcceptCommand(adrc_drive, 4U).ok());
  CHECK(adrc_controller.ControlStep(stationary)[0] == -67);

  // Measured motion alone blocks all-channel updates, even while disarmed.
  MotorController moving_adrc(FullRangeTestMotorConfiguration());
  moving_adrc.ControlStep(stationary);
  stationary[0] = 1U;
  moving_adrc.ControlStep(stationary);
  CHECK(std::fabs(moving_adrc.channels()[0].measured_rps) >=
        kMotorAdrcUpdateMaximumMeasuredRps);
  CHECK(moving_adrc.SetAdrc(adrc_update).result.code == ResultCode::kBusy);
  CHECK(moving_adrc.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(moving_adrc.SetAdrc(adrc_update).result.ok());

  MotorController saturated_adrc(FullRangeTestMotorConfiguration());
  SetMotorAdrcCommand saturation_calibration = DefaultMotorAdrcCommand(1U);
  saturation_calibration.input_gain_rps_per_second_per_permille[0] = 0.001F;
  saturation_calibration.disturbance_estimate_limit_rps_per_second[0] = 1.0F;
  saturation_calibration.controller_bandwidth_rad_s[0] = 4.0F;
  saturation_calibration.observer_bandwidth_rad_s[0] = 12.0F;
  saturation_calibration.velocity_filter_new_weight[0] = 1.0F;
  CHECK(saturated_adrc.SetAdrc(saturation_calibration).result.ok());
  saturated_adrc.SetSessionActive(true);
  MotorCommand saturation_drive{};
  saturation_drive.update_mask = 1U;
  saturation_drive.target_rps[0] = 6.0F;
  CHECK(saturated_adrc.AcceptCommand(saturation_drive, 0U).ok());
  stationary.fill(0U);
  CHECK(saturated_adrc.ControlStep(stationary)[0] == -1000);

  constexpr std::array<float, 4> kProfileLimits{1.5F, 3.0F, 6.0F, 1.1F};
  MotorController profile_limits(FullRangeTestMotorConfiguration());
  profile_limits.SetSessionActive(true);
  for (std::size_t index = 0U; index < kModels.size(); ++index) {
    CHECK(profile_limits.SetModel(kModels[index]).result.ok());
    CHECK_NEAR(
        profile_limits.profile().adrc.known_velocity_decay_rate_s_inverse,
        kMotorDefaultAdrcKnownVelocityDecayRateSInverse, 0.0001F);
    CHECK_NEAR(
        profile_limits.profile().adrc.input_gain_rps_per_second_per_permille,
        kMotorDefaultAdrcInputGainRpsPerSecondPerPermille, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.controller_bandwidth_rad_s,
               kMotorDefaultAdrcControllerBandwidthRadS, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.controller_fal_exponent,
               kMotorDefaultAdrcControllerFalExponent, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.controller_fal_threshold_rps,
               kMotorDefaultAdrcControllerFalThresholdRps, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.observer_bandwidth_rad_s,
               kMotorDefaultAdrcObserverBandwidthRadS, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.observer_velocity_fal_exponent,
               kMotorDefaultAdrcObserverVelocityFalExponent, 0.0001F);
    CHECK_NEAR(
        profile_limits.profile().adrc.observer_disturbance_fal_exponent,
        kMotorDefaultAdrcObserverDisturbanceFalExponent, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.observer_fal_threshold_rps,
               kMotorDefaultAdrcObserverFalThresholdRps, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.disturbance_leakage_s_inverse,
               kMotorDefaultAdrcDisturbanceLeakageSInverse, 0.0001F);
    CHECK_NEAR(
        profile_limits.profile()
            .adrc.disturbance_estimate_limit_rps_per_second,
        kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond, 0.0001F);
    CHECK_NEAR(profile_limits.profile().adrc.velocity_filter_new_weight,
               kMotorDefaultVelocityFilterNewWeight, 0.0001F);
    CHECK(profile_limits.profile().adrc.positive_minimum_drive_permille ==
          kMotorDefaultPositiveMinimumDrivePermille);
    CHECK(profile_limits.profile().adrc.negative_minimum_drive_permille ==
          kMotorDefaultNegativeMinimumDrivePermille);
    MotorCommand limit_command{};
    limit_command.update_mask = 1U;
    limit_command.target_rps[0] = kProfileLimits[index];
    CHECK(profile_limits.AcceptCommand(limit_command, 10U).ok());
    limit_command.target_rps[0] = 0.0F;
    CHECK(profile_limits.AcceptCommand(limit_command, 11U).ok());
    limit_command.target_rps[0] = std::nextafter(kProfileLimits[index], 7.0F);
    CHECK(profile_limits.AcceptCommand(limit_command, 12U).code ==
          ResultCode::kOutOfRange);
  }

  // Every model preserves the raw signed encoder delta. Model selection
  // changes scale and limits only; the ROS hardware plugin owns chassis signs.
  for (const MotorModel model : kModels) {
    MotorController raw_encoder(FullRangeTestMotorConfiguration());
    CHECK(raw_encoder.SetModel(model).result.ok());
    stationary.fill(0U);
    raw_encoder.ControlStep(stationary);
    stationary[0] = 10U;
    raw_encoder.ControlStep(stationary);
    CHECK(raw_encoder.channels()[0].encoder_count == 10);
  }

  // Every model and channel uses the same physical bridge inversion while
  // retaining the semantic LADRC output in the target/raw-encoder coordinate.
  for (const MotorModel model : kModels) {
    MotorController positive(FullRangeTestMotorConfiguration());
    CHECK(positive.SetModel(model).result.ok());
    positive.SetSessionActive(true);
    MotorCommand positive_target{};
    positive_target.update_mask = kAllMotorMask;
    positive_target.target_rps.fill(1.0F);
    CHECK(positive.AcceptCommand(positive_target, 0U).ok());
    stationary.fill(0U);
    const auto positive_bridge = positive.ControlStep(stationary);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      CHECK(positive.channels()[motor].output_permille == 133);
      CHECK(positive_bridge[motor] == -133);
    }

    MotorController negative(FullRangeTestMotorConfiguration());
    CHECK(negative.SetModel(model).result.ok());
    negative.SetSessionActive(true);
    MotorCommand negative_target{};
    negative_target.update_mask = kAllMotorMask;
    negative_target.target_rps.fill(-1.0F);
    CHECK(negative.AcceptCommand(negative_target, 0U).ok());
    const auto negative_bridge = negative.ControlStep(stationary);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      CHECK(negative.channels()[motor].output_permille == -133);
      CHECK(negative_bridge[motor] == 133);
    }
  }
}

void TestFirstOrderAdrcController() {
  const std::array<std::uint32_t, kMotorCount> stationary{};
  auto configure_adrc = [](MotorController* controller, float input_gain,
                           float controller_bandwidth, float observer_bandwidth,
                           float filter_weight = 1.0F) {
    SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
    calibration.input_gain_rps_per_second_per_permille[0] = input_gain;
    calibration.disturbance_estimate_limit_rps_per_second[0] =
        input_gain * kMotorAdrcHardOutputLimitPermille;
    calibration.controller_bandwidth_rad_s[0] = controller_bandwidth;
    calibration.observer_bandwidth_rad_s[0] = observer_bandwidth;
    calibration.velocity_filter_new_weight[0] = filter_weight;
    CHECK(controller->SetAdrc(calibration).result.ok());
    controller->SetSessionActive(true);
  };
  auto command_speed = [](MotorController* controller, float target_rps,
                          std::uint32_t now_us = 0U) {
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = target_rps;
    CHECK(controller->AcceptCommand(command, now_us).ok());
  };

  MotorController nominal(FullRangeTestMotorConfiguration());
  configure_adrc(&nominal, 0.01F, 4.0F, 12.0F);
  command_speed(&nominal, 1.0F);
  CHECK(nominal.ControlStep(stationary)[0] == -400);
  CHECK(nominal.ControlStep(stationary)[0] == -384);

  MotorController negative(FullRangeTestMotorConfiguration());
  configure_adrc(&negative, 0.01F, 4.0F, 12.0F);
  command_speed(&negative, -1.0F);
  CHECK(negative.ControlStep(stationary)[0] == 400);

  MotorController saturated(FullRangeTestMotorConfiguration());
  configure_adrc(&saturated, 0.001F, 4.0F, 12.0F);
  command_speed(&saturated, 6.0F);
  CHECK(saturated.ControlStep(stationary)[0] ==
        -kMotorOutputLimitPermille);
  command_speed(&saturated, -6.0F, 1U);
  CHECK(saturated.ControlStep(stationary)[0] ==
        kMotorOutputLimitPermille);
  CHECK(saturated.channels()[0].output_permille ==
        -kMotorOutputLimitPermille);

  // The ESO observes filtered encoder velocity and reduces the command as the
  // measured speed approaches the reference.
  MotorController observed(FullRangeTestMotorConfiguration());
  configure_adrc(&observed, 0.01F, 4.0F, 12.0F, 1.0F);
  std::array<std::uint32_t, kMotorCount> counters{};
  counters[0] = 100U;
  command_speed(&observed, 1.0F);
  const std::int16_t initial_output = observed.ControlStep(counters)[0];
  counters[0] = 101U;
  const std::int16_t feedback_output = observed.ControlStep(counters)[0];
  CHECK(initial_output == -400);
  CHECK(feedback_output < 0);
  CHECK(std::abs(feedback_output) < std::abs(initial_output));
  CHECK(observed.channels()[0].measured_rps > 0.0F);

  // Zero, lease, and session-loss paths clear the observer and applied-output
  // state without clearing the volatile calibration override.
  MotorController reset(FullRangeTestMotorConfiguration());
  configure_adrc(&reset, 0.01F, 4.0F, 12.0F);
  command_speed(&reset, 1.0F);
  CHECK(reset.ControlStep(stationary)[0] == -400);
  static_cast<void>(reset.ControlStep(stationary));
  command_speed(&reset, 0.0F, 10U);
  command_speed(&reset, 1.0F, 11U);
  CHECK(reset.ControlStep(stationary)[0] == -400);
  reset.EvaluateLeases(11U + kMotorLeaseExpiryUs);
  CHECK(!reset.channels()[0].armed);
  reset.SetSessionActive(false);
  reset.SetSessionActive(true);
  command_speed(&reset, 1.0F, 12U + kMotorLeaseExpiryUs);
  CHECK(reset.ControlStep(stationary)[0] == -400);

  // Forward-Euler observer timing is fail-closed outside wo * dt <= 0.5.
  MotorController invalid_period(FullRangeTestMotorConfiguration());
  configure_adrc(&invalid_period, 0.01F, 4.0F, 12.0F);
  command_speed(&invalid_period, 1.0F);
  CHECK(invalid_period.ControlStep(stationary, 50000U)[0] == 0);
  CHECK(!invalid_period.channels()[0].armed);
  CHECK(invalid_period.watchdog_stop_mask() == 1U);
}

void TestNonlinearAdrcAndDirectionalMinimumDrive() {
  const std::array<std::uint32_t, kMotorCount> stationary{};
  auto command_speed = [](MotorController* controller, float target_rps,
                          std::uint32_t now_us = 0U) {
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = target_rps;
    CHECK(controller->AcceptCommand(command, now_us).ok());
  };

  // Compatibility defaults preserve the former linear first-step behavior:
  // 4 rad/s * 0.2 RPS / 0.03 = 26.67 semantic permille.
  MotorController compatible(FullRangeTestMotorConfiguration());
  compatible.SetSessionActive(true);
  command_speed(&compatible, 0.2F);
  CHECK(compatible.ControlStep(stationary)[0] == -27);
  CHECK(compatible.channels()[0].output_permille == 27);

  // The nonlinear FAL keeps unit slope below delta and uses the
  // dimension-preserving delta * (|e| / delta)^alpha map above it.
  MotorController nonlinear(FullRangeTestMotorConfiguration());
  SetMotorAdrcCommand nonlinear_calibration = DefaultMotorAdrcCommand(1U);
  nonlinear_calibration.input_gain_rps_per_second_per_permille[0] = 0.01F;
  nonlinear_calibration.disturbance_estimate_limit_rps_per_second[0] = 10.0F;
  nonlinear_calibration.controller_fal_exponent[0] = 0.5F;
  nonlinear_calibration.controller_fal_threshold_rps[0] = 0.25F;
  nonlinear_calibration.velocity_filter_new_weight[0] = 1.0F;
  CHECK(nonlinear.SetAdrc(nonlinear_calibration).result.ok());
  nonlinear.SetSessionActive(true);
  command_speed(&nonlinear, 0.125F);
  CHECK(nonlinear.ControlStep(stationary)[0] == -50);
  command_speed(&nonlinear, 0.0F, 1U);
  command_speed(&nonlinear, 1.0F, 2U);
  CHECK(nonlinear.ControlStep(stationary)[0] == -200);
  command_speed(&nonlinear, 0.0F, 3U);
  command_speed(&nonlinear, -1.0F, 4U);
  CHECK(nonlinear.ControlStep(stationary)[0] == 200);

  SetMotorAdrcCommand floor_calibration = DefaultMotorAdrcCommand(1U);
  floor_calibration.velocity_filter_new_weight[0] = 1.0F;
  floor_calibration.positive_minimum_drive_permille[0] = 120U;
  floor_calibration.negative_minimum_drive_permille[0] = 180U;

  MotorController directional_floor(FullRangeTestMotorConfiguration());
  CHECK(directional_floor.SetAdrc(floor_calibration).result.ok());
  directional_floor.SetSessionActive(true);
  command_speed(&directional_floor, 0.2F);
  CHECK(directional_floor.ControlStep(stationary)[0] == -120);
  CHECK(directional_floor.channels()[0].output_permille == 120);

  // Feed the actual post-floor +120 permille back to the ESO. A positive
  // encoder impulse then asks for a small reverse correction; the positive
  // target floor must not turn that correction into either +/-120 permille.
  std::array<std::uint32_t, kMotorCount> moved{};
  moved[0] = 3U;
  const std::int16_t reverse_correction =
      directional_floor.ControlStep(moved)[0];
  CHECK(reverse_correction > 0);
  CHECK(reverse_correction < 120);
  CHECK(directional_floor.channels()[0].output_permille < 0);
  CHECK(std::abs(directional_floor.channels()[0].output_permille) < 120);

  // Exact zero is an immediate disarm/reset. A new command starts from the
  // current filtered measurement and retains the volatile calibration.
  command_speed(&directional_floor, 0.0F, 1U);
  CHECK(!directional_floor.channels()[0].armed);
  CHECK(directional_floor.channels()[0].output_permille == 0);
  CHECK(directional_floor.ControlStep(moved)[0] == 0);

  // Asymmetric friction compensation applies the reverse-direction floor.
  MotorController sign_reversal(FullRangeTestMotorConfiguration());
  CHECK(sign_reversal.SetAdrc(floor_calibration).result.ok());
  sign_reversal.SetSessionActive(true);
  command_speed(&sign_reversal, 0.2F);
  CHECK(sign_reversal.ControlStep(stationary)[0] == -120);
  CHECK(sign_reversal.ControlStep(stationary)[0] == -120);
  command_speed(&sign_reversal, -0.2F, 1U);
  CHECK(sign_reversal.ControlStep(stationary)[0] == 180);
  CHECK(sign_reversal.channels()[0].output_permille == -180);

  // A direct nonzero target-sign reversal resets the ESO before computing the
  // opposite-direction command; stale forward state would produce a different
  // first reverse output here.
  MotorController reversal_reset(FullRangeTestMotorConfiguration());
  CHECK(reversal_reset.SetAdrc(DefaultMotorAdrcCommand(1U)).result.ok());
  reversal_reset.SetSessionActive(true);
  command_speed(&reversal_reset, 1.0F);
  CHECK(reversal_reset.ControlStep(stationary)[0] == -133);
  static_cast<void>(reversal_reset.ControlStep(stationary));
  command_speed(&reversal_reset, -0.2F, 1U);
  CHECK(reversal_reset.ControlStep(stationary)[0] == 27);
  CHECK(reversal_reset.channels()[0].output_permille == -27);

  // Disturbance compensation is independently clip-bounded. With the same
  // positive encoder impulse, a zero clip limit removes z2 from the control
  // law.
  auto feedback_output = [&](float disturbance_limit) {
    MotorController controller(FullRangeTestMotorConfiguration());
    SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
    calibration.velocity_filter_new_weight[0] = 1.0F;
    calibration.disturbance_estimate_limit_rps_per_second[0] =
        disturbance_limit;
    CHECK(controller.SetAdrc(calibration).result.ok());
    controller.SetSessionActive(true);
    command_speed(&controller, 1.0F);
    static_cast<void>(controller.ControlStep(stationary));
    std::array<std::uint32_t, kMotorCount> counters{};
    counters[0] = 10U;
    return controller.ControlStep(counters)[0];
  };
  const std::int16_t compensated_output = feedback_output(30.0F);
  const std::int16_t uncompensated_output = feedback_output(0.0F);
  CHECK(compensated_output < 0);
  CHECK(uncompensated_output < 0);
  CHECK(std::abs(compensated_output) < std::abs(uncompensated_output));

  // Each forward-Euler rate term has the same fail-closed timing bound.
  auto check_timing_guard = [&](float known_decay, float leakage) {
    MotorController controller(FullRangeTestMotorConfiguration());
    SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
    calibration.known_velocity_decay_rate_s_inverse[0] = known_decay;
    calibration.disturbance_leakage_s_inverse[0] = leakage;
    CHECK(controller.SetAdrc(calibration).result.ok());
    controller.SetSessionActive(true);
    command_speed(&controller, 0.2F);
    CHECK(controller.ControlStep(stationary, 10001U)[0] == 0);
    CHECK(!controller.channels()[0].armed);
    CHECK(controller.watchdog_stop_mask() == 1U);
  };
  check_timing_guard(50.0F, 0.0F);
  check_timing_guard(0.0F, 50.0F);
}

void TestNonlinearFalBoundaryAndSign() {
  const std::array<std::uint32_t, kMotorCount> stationary{};
  constexpr float kExponent = 0.5F;
  constexpr float kThresholdRps = 0.25F;
  auto initial_semantic_output = [&](float target_rps) {
    MotorController controller(FullRangeTestMotorConfiguration());
    SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
    calibration.input_gain_rps_per_second_per_permille[0] = 0.01F;
    calibration.disturbance_estimate_limit_rps_per_second[0] = 10.0F;
    calibration.controller_fal_exponent[0] = kExponent;
    calibration.controller_fal_threshold_rps[0] = kThresholdRps;
    calibration.velocity_filter_new_weight[0] = 1.0F;
    CHECK(controller.SetAdrc(calibration).result.ok());
    controller.SetSessionActive(true);
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = target_rps;
    CHECK(controller.AcceptCommand(command, 0U).ok());
    static_cast<void>(controller.ControlStep(stationary));
    return controller.channels()[0].output_permille;
  };

  const float positive_below = std::nextafter(kThresholdRps, 0.0F);
  const float positive_above = std::nextafter(
      kThresholdRps, std::numeric_limits<float>::infinity());
  const float negative_below = std::nextafter(-kThresholdRps, 0.0F);
  const float negative_above = std::nextafter(
      -kThresholdRps, -std::numeric_limits<float>::infinity());

  // Both sides meet at exactly phi(delta) = delta. The nearest representable
  // inputs retain the same quantized value, while the signs remain odd.
  CHECK(initial_semantic_output(positive_below) == 100);
  CHECK(initial_semantic_output(kThresholdRps) == 100);
  CHECK(initial_semantic_output(positive_above) == 100);
  CHECK(initial_semantic_output(negative_below) == -100);
  CHECK(initial_semantic_output(-kThresholdRps) == -100);
  CHECK(initial_semantic_output(negative_above) == -100);

  // Unit slope inside delta and the normalized power branch outside delta are
  // checked on both signs, including a small nonzero error.
  CHECK(initial_semantic_output(0.01F) == 4);
  CHECK(initial_semantic_output(-0.01F) == -4);
  CHECK(initial_semantic_output(0.24F) == 96);
  CHECK(initial_semantic_output(-0.24F) == -96);
  CHECK(initial_semantic_output(0.26F) == 102);
  CHECK(initial_semantic_output(-0.26F) == -102);
}

void TestAlphaOneMatchesLegacyLinearTrajectory() {
  constexpr float kPeriodSeconds =
      static_cast<float>(kMotorControlPeriodUs) / 1000000.0F;
  constexpr float kInputGain = 0.03F;
  constexpr float kControllerBandwidth = 4.0F;
  constexpr float kObserverBandwidth = 12.0F;
  constexpr float kFilterWeight = 0.35F;
  constexpr float kTargetRps = 0.4F;
  constexpr std::array<std::uint32_t, 10> kCounterSamples{
      0U, 2U, 3U, 1U, 4U, 4U, 7U, 5U, 8U, 8U};

  MotorController controller(FullRangeTestMotorConfiguration());
  SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
  calibration.velocity_filter_new_weight[0] = kFilterWeight;
  CHECK(controller.SetAdrc(calibration).result.ok());
  controller.SetSessionActive(true);
  MotorCommand command{};
  command.update_mask = 1U;
  command.target_rps[0] = kTargetRps;
  CHECK(controller.AcceptCommand(command, 0U).ok());

  // Independent legacy first-order LADRC recurrence. With a=0, leakage=0,
  // all alpha=1, zero floors, and an inactive disturbance clip, the expanded
  // implementation must be output-for-output identical over many steps.
  float measured_rps = 0.0F;
  float observed_velocity_rps = 0.0F;
  float observed_disturbance_rps_per_second = 0.0F;
  float applied_output_permille = 0.0F;
  std::uint32_t previous_counter = kCounterSamples[0];
  for (std::size_t step = 0U; step < kCounterSamples.size(); ++step) {
    const std::int32_t encoder_delta =
        step == 0U
            ? 0
            : static_cast<std::int32_t>(kCounterSamples[step]) -
                  static_cast<std::int32_t>(previous_counter);
    previous_counter = kCounterSamples[step];
    const float instantaneous_rps =
        static_cast<float>(encoder_delta) /
        (static_cast<float>(controller.profile().ticks_per_revolution) *
         kPeriodSeconds);
    measured_rps = kFilterWeight * instantaneous_rps +
                   (1.0F - kFilterWeight) * measured_rps;
    const float observer_error = observed_velocity_rps - measured_rps;
    observed_velocity_rps +=
        kPeriodSeconds *
        (observed_disturbance_rps_per_second +
         kInputGain * applied_output_permille -
         2.0F * kObserverBandwidth * observer_error);
    observed_disturbance_rps_per_second +=
        kPeriodSeconds * (-kObserverBandwidth * kObserverBandwidth *
                          observer_error);
    CHECK(std::fabs(observed_disturbance_rps_per_second) <
          kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond);
    float candidate =
        (kControllerBandwidth * (kTargetRps - observed_velocity_rps) -
         observed_disturbance_rps_per_second) /
        kInputGain;
    candidate = std::max(-kMotorAdrcHardOutputLimitPermille,
                         std::min(kMotorAdrcHardOutputLimitPermille,
                                  candidate));
    const auto expected_semantic_output =
        static_cast<std::int16_t>(std::lround(candidate));
    applied_output_permille =
        static_cast<float>(expected_semantic_output);

    std::array<std::uint32_t, kMotorCount> counters{};
    counters[0] = kCounterSamples[step];
    const auto bridge_output = controller.ControlStep(counters);
    CHECK(controller.channels()[0].output_permille ==
          expected_semantic_output);
    CHECK(bridge_output[0] ==
          static_cast<std::int16_t>(-expected_semantic_output));
    CHECK_NEAR(controller.channels()[0].measured_rps, measured_rps, 0.000001F);
  }
}

void TestNonlinearObserverEvolutionAndDynamicTerms() {
  constexpr float kPeriodSeconds =
      static_cast<float>(kMotorControlPeriodUs) / 1000000.0F;
  constexpr float kInputGain = 0.03F;
  constexpr float kControllerBandwidth = 4.0F;
  constexpr float kObserverBandwidth = 12.0F;
  constexpr float kObserverExponent = 0.5F;
  constexpr float kObserverThresholdRps = 0.1F;

  MotorController nonlinear_observer(FullRangeTestMotorConfiguration());
  SetMotorAdrcCommand nonlinear_calibration = DefaultMotorAdrcCommand(1U);
  nonlinear_calibration.observer_velocity_fal_exponent[0] =
      kObserverExponent;
  nonlinear_calibration.observer_disturbance_fal_exponent[0] =
      kObserverExponent;
  nonlinear_calibration.observer_fal_threshold_rps[0] =
      kObserverThresholdRps;
  nonlinear_calibration.velocity_filter_new_weight[0] = 1.0F;
  CHECK(nonlinear_observer.SetAdrc(nonlinear_calibration).result.ok());
  nonlinear_observer.SetSessionActive(true);
  const float measured_target_rps =
      10.0F /
      (static_cast<float>(nonlinear_observer.profile().ticks_per_revolution) *
       kPeriodSeconds);
  MotorCommand observer_command{};
  observer_command.update_mask = 1U;
  observer_command.target_rps[0] = measured_target_rps;
  CHECK(nonlinear_observer.AcceptCommand(observer_command, 0U).ok());

  auto reference_fal = [](float error, float exponent, float threshold) {
    const float magnitude = std::fabs(error);
    if (magnitude <= threshold) {
      return error;
    }
    return std::copysign(
        threshold * std::pow(magnitude / threshold, exponent), error);
  };
  float observed_velocity_rps = 0.0F;
  float observed_disturbance_rps_per_second = 0.0F;
  float applied_output_permille = 0.0F;
  float first_feedback_error = 0.0F;
  float final_feedback_error = 0.0F;
  for (std::uint32_t step = 0U; step < 40U; ++step) {
    const float measured_rps = step == 0U ? 0.0F : measured_target_rps;
    const float observer_error = observed_velocity_rps - measured_rps;
    const float correction = reference_fal(
        observer_error, kObserverExponent, kObserverThresholdRps);
    observed_velocity_rps +=
        kPeriodSeconds *
        (observed_disturbance_rps_per_second +
         kInputGain * applied_output_permille -
         2.0F * kObserverBandwidth * correction);
    observed_disturbance_rps_per_second +=
        kPeriodSeconds *
        (-kObserverBandwidth * kObserverBandwidth * correction);
    const float compensated_disturbance = std::max(
        -kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond,
        std::min(kMotorDefaultAdrcDisturbanceEstimateLimitRpsPerSecond,
                 observed_disturbance_rps_per_second));
    float candidate =
        (kControllerBandwidth *
             (measured_target_rps - observed_velocity_rps) -
         compensated_disturbance) /
        kInputGain;
    candidate = std::max(-kMotorAdrcHardOutputLimitPermille,
                         std::min(kMotorAdrcHardOutputLimitPermille,
                                  candidate));
    const auto expected_semantic_output =
        static_cast<std::int16_t>(std::lround(candidate));
    applied_output_permille =
        static_cast<float>(expected_semantic_output);

    std::array<std::uint32_t, kMotorCount> counters{};
    counters[0] = step * 10U;
    const auto bridge_output = nonlinear_observer.ControlStep(counters);
    CHECK(nonlinear_observer.channels()[0].output_permille ==
          expected_semantic_output);
    CHECK(bridge_output[0] ==
          static_cast<std::int16_t>(-expected_semantic_output));
    if (step == 1U) {
      first_feedback_error =
          std::fabs(observed_velocity_rps - measured_rps);
    }
    if (step == 39U) {
      final_feedback_error =
          std::fabs(observed_velocity_rps - measured_rps);
    }
  }
  CHECK(first_feedback_error > 0.8F);
  CHECK(final_feedback_error < 0.001F);
  CHECK(final_feedback_error < first_feedback_error * 0.01F);
  CHECK(observed_disturbance_rps_per_second > 4.0F);

  auto dynamic_trajectory = [](float known_decay, float leakage) {
    std::array<std::int16_t, 8> outputs{};
    MotorController controller(FullRangeTestMotorConfiguration());
    SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
    calibration.known_velocity_decay_rate_s_inverse[0] = known_decay;
    calibration.disturbance_leakage_s_inverse[0] = leakage;
    calibration.velocity_filter_new_weight[0] = 1.0F;
    CHECK(controller.SetAdrc(calibration).result.ok());
    controller.SetSessionActive(true);
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = 1.0F;
    CHECK(controller.AcceptCommand(command, 0U).ok());
    for (std::size_t step = 0U; step < outputs.size(); ++step) {
      std::array<std::uint32_t, kMotorCount> counters{};
      counters[0] = static_cast<std::uint32_t>(step * 10U);
      static_cast<void>(controller.ControlStep(counters));
      outputs[step] = controller.channels()[0].output_permille;
    }
    return outputs;
  };

  const auto baseline = dynamic_trajectory(0.0F, 0.0F);
  const auto with_known_decay = dynamic_trajectory(10.0F, 0.0F);
  const auto with_leakage = dynamic_trajectory(0.0F, 10.0F);
  CHECK(baseline ==
        (std::array<std::int16_t, 8>{133, 51, -8, -51, -81, -103, -119,
                                    -130}));
  CHECK(with_known_decay ==
        (std::array<std::int16_t, 8>{133, 141, 147, 151, 155, 157, 159,
                                    160}));
  CHECK(with_leakage ==
        (std::array<std::int16_t, 8>{133, 51, -3, -39, -60, -72, -78,
                                    -79}));
  CHECK(with_known_decay[1] > baseline[1]);
  CHECK(with_leakage.back() > baseline.back());
}

void TestMotorLeaseBoundariesAndScheduleModel() {
  constexpr std::uint32_t kSafetyLimitUs = 200000U;
  constexpr std::uint32_t kNominalReleasePeriodUs = 1000U;
  constexpr std::uint32_t kMaximumEvaluationGapUs = 2000U;

  MotorCommand all_motors{};
  all_motors.update_mask = kAllMotorMask;
  all_motors.target_rps.fill(1.0F);

  // Check every integer-microsecond age on the lower lease boundary.
  MotorController every_age(FullRangeTestMotorConfiguration());
  every_age.SetSessionActive(true);
  constexpr std::uint32_t kAcceptedAtUs = 1000U;
  CHECK(every_age.AcceptCommand(all_motors, kAcceptedAtUs).ok());
  const std::array<std::uint32_t, kMotorCount> counters{};
  const auto running_outputs = every_age.ControlStep(counters);
  for (const std::int16_t output : running_outputs) {
    CHECK(output != 0);
  }
  for (std::uint32_t age_us = 0U; age_us < kMotorLeaseExpiryUs; ++age_us) {
    every_age.EvaluateLeases(kAcceptedAtUs + age_us);
    for (const MotorChannelState& channel : every_age.channels()) {
      CHECK(channel.armed);
      CHECK(channel.output_permille != 0);
    }
  }
  every_age.EvaluateLeases(kAcceptedAtUs + kMotorLeaseExpiryUs);
  CHECK(every_age.watchdog_stop_mask() == kAllMotorMask);
  for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
    CHECK(!every_age.channels()[motor].armed);
    CHECK(every_age.channels()[motor].output_permille == 0);
    CHECK(every_age.lease_expiry_count(motor) == 1U);
  }

  // Every motor subset expires atomically at the inclusive 198 ms boundary;
  // unselected channels remain untouched.
  for (std::uint16_t raw_mask = 1U; raw_mask <= kAllMotorMask; ++raw_mask) {
    MotorController subset(FullRangeTestMotorConfiguration());
    subset.SetSessionActive(true);
    MotorCommand command{};
    command.update_mask = static_cast<std::uint8_t>(raw_mask);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      if ((command.update_mask & bit) != 0U) {
        command.target_rps[motor] = 1.0F;
      }
    }
    CHECK(subset.AcceptCommand(command, 37U).ok());
    subset.EvaluateLeases(37U + kMotorLeaseExpiryUs - 1U);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      CHECK(subset.channels()[motor].armed ==
            ((command.update_mask & bit) != 0U));
    }
    subset.EvaluateLeases(37U + kMotorLeaseExpiryUs);
    CHECK(subset.watchdog_stop_mask() == command.update_mask);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      CHECK(!subset.channels()[motor].armed);
      CHECK(subset.lease_expiry_count(motor) ==
            ((command.update_mask & bit) != 0U ? 1U : 0U));
    }
  }

  // Exhaust every acceptance phase relative to the nominal 1 kHz TIM7
  // release. Model the worst permitted completion jitter by skipping one
  // release, producing a completed-evaluation gap of exactly 2 ms.
  for (std::uint32_t phase_us = 0U; phase_us < kNominalReleasePeriodUs;
       ++phase_us) {
    const std::uint32_t expiry_us = phase_us + kMotorLeaseExpiryUs;
    const std::uint32_t first_release_at_or_after_expiry =
        ((expiry_us + kNominalReleasePeriodUs - 1U) / kNominalReleasePeriodUs) *
        kNominalReleasePeriodUs;
    const std::uint32_t delayed_evaluation_us =
        first_release_at_or_after_expiry + kNominalReleasePeriodUs;
    const std::uint32_t prior_evaluation_us =
        delayed_evaluation_us - kMaximumEvaluationGapUs;

    MotorController phased(FullRangeTestMotorConfiguration());
    phased.SetSessionActive(true);
    CHECK(phased.AcceptCommand(all_motors, phase_us).ok());
    phased.EvaluateLeases(prior_evaluation_us);
    for (const MotorChannelState& channel : phased.channels()) {
      CHECK(channel.armed);
    }
    phased.EvaluateLeases(delayed_evaluation_us);
    const std::uint32_t stop_age_us = delayed_evaluation_us - phase_us;
    CHECK(delayed_evaluation_us - prior_evaluation_us <=
          kMaximumEvaluationGapUs);
    CHECK(stop_age_us >= kMotorLeaseExpiryUs);
    CHECK(stop_age_us <= kSafetyLimitUs);
    for (const MotorChannelState& channel : phased.channels()) {
      CHECK(!channel.armed);
      CHECK(channel.output_permille == 0);
    }
  }

  // Unsigned age arithmetic preserves both sides of the boundary across the
  // 32-bit microsecond clock wrap.
  MotorController wrapped(FullRangeTestMotorConfiguration());
  wrapped.SetSessionActive(true);
  constexpr std::uint32_t kWrappedAcceptanceUs = 0xffffff00U;
  CHECK(wrapped.AcceptCommand(all_motors, kWrappedAcceptanceUs).ok());
  const std::uint32_t wrapped_expiry_us =
      kWrappedAcceptanceUs + kMotorLeaseExpiryUs;
  wrapped.EvaluateLeases(wrapped_expiry_us - 1U);
  for (const MotorChannelState& channel : wrapped.channels()) {
    CHECK(channel.armed);
  }
  wrapped.EvaluateLeases(wrapped_expiry_us);
  CHECK(wrapped.watchdog_stop_mask() == kAllMotorMask);
}

void TestPwmServoController() {
  PwmServoController controller;
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);
  PwmFrameUpdate submitted = controller.PrepareFollowingFrame();
  const auto prepare_pending = [&controller, &submitted]() {
    const PwmFrameUpdate replacement =
        controller.PreparePendingFrame(submitted);
    controller.ConfirmPendingFrameSubmitted(replacement);
    submitted = replacement;
  };
  const auto commit_boundary = [&controller, &submitted]() {
    controller.CommitFrame(submitted);
    submitted = controller.PrepareFollowingFrame();
  };

  PwmServoCommand command{};
  command.update_mask = 1U;
  command.duration_ms = 40U;
  command.pulse_width_us[0] = 1601U;
  CHECK(controller.AcceptCommand(command).ok());
  CHECK(controller.state().target_pulse_width_us[0] == 1601U);

  prepare_pending();
  CHECK(submitted.output_pulse_width_us()[0] == 1500U);
  commit_boundary();  // B0 starts interpolation without installing step one.
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);
  CHECK(controller.state().moving_mask == 1U);
  CHECK(submitted.output_pulse_width_us()[0] == 1551U);
  commit_boundary();  // B1.
  CHECK(controller.state().output_pulse_width_us[0] == 1551U);
  CHECK(submitted.output_pulse_width_us()[0] == 1601U);
  commit_boundary();  // B2, 40 ms after B0.
  CHECK(controller.state().output_pulse_width_us[0] == 1601U);
  CHECK(controller.state().moving_mask == 0U);

  command.pulse_width_us[0] = 1500U;
  CHECK(controller.AcceptCommand(command).ok());
  prepare_pending();
  commit_boundary();  // B0 remains at 1601 us.
  CHECK(submitted.output_pulse_width_us()[0] == 1550U);
  commit_boundary();
  commit_boundary();
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 1U;
  offsets.offset_us[0] = 100;
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(controller.offset_commit_pending());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 1U);
  CHECK(submitted.output_pulse_width_us()[0] == 1600U);
  CHECK(controller.offset_commit_pending());
  commit_boundary();
  CHECK(!controller.offset_commit_pending());
  CHECK(controller.state().output_pulse_width_us[0] == 1600U);
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(!controller.offset_commit_pending());

  offsets.update_mask = 3U;
  offsets.offset_us[0] = 100;
  offsets.offset_us[1] = -50;
  CHECK(controller.StageOffsets(offsets).ok());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 3U);
  commit_boundary();
  CHECK(controller.state().offset_us[0] == 100);
  CHECK(controller.state().offset_us[1] == -50);

  // Session loss freezes an in-progress trajectory at the exact active pulse.
  command.duration_ms = 100U;
  command.pulse_width_us[0] = 1800U;
  CHECK(controller.AcceptCommand(command).ok());
  prepare_pending();
  commit_boundary();  // B0.
  commit_boundary();  // B1 is now physical; B2 is only submitted.
  const std::uint16_t held_output = controller.state().output_pulse_width_us[0];
  CHECK(submitted.output_pulse_width_us()[0] > held_output);
  CHECK(controller.state().moving_mask == 1U);
  const auto held_offsets = controller.state().offset_us;
  controller.HoldCurrentOutputAndCancelPending(
      controller.state().output_pulse_width_us, held_offsets);
  submitted = controller.PrepareFollowingFrame();
  CHECK(controller.state().moving_mask == 0U);
  CHECK(controller.state().target_pulse_width_us[0] == held_output - 100U);
  for (std::size_t boundary = 0U; boundary < 8U; ++boundary) {
    commit_boundary();
    CHECK(controller.state().output_pulse_width_us[0] == held_output);
  }

  // An offset not yet committed at a common frame boundary is canceled too.
  offsets.update_mask = 1U;
  offsets.offset_us[0] = -100;
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(controller.offset_commit_pending());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 1U);
  const auto committed_offsets = controller.state().offset_us;
  controller.HoldCurrentOutputAndCancelPending(
      controller.state().output_pulse_width_us, committed_offsets);
  submitted = controller.PrepareFollowingFrame();
  CHECK(!controller.offset_commit_pending());
  CHECK(submitted.offset_commit_mask() == 0U);
  CHECK(controller.state().offset_us[0] == 100);
  commit_boundary();
  CHECK(controller.state().output_pulse_width_us[0] == held_output);
}

void TestButtons() {
  ButtonController buttons;
  std::array<bool, kButtonCount> raw{};
  ButtonEvent event{};

  raw[0] = true;
  buttons.Sample(raw, 0U);
  CHECK(!buttons.PopEvent(&event));
  buttons.Sample(raw, 30U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);

  raw[0] = false;
  buttons.Sample(raw, 60U);
  buttons.Sample(raw, 90U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kReleaseFromShortPress);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kClick);

  raw[0] = true;
  buttons.Sample(raw, 120U);
  buttons.Sample(raw, 150U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kDoubleClick);

  raw[0] = false;
  buttons.Sample(raw, 180U);
  buttons.Sample(raw, 210U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kReleaseFromShortPress);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kClick);
  raw[0] = true;
  buttons.Sample(raw, 240U);
  buttons.Sample(raw, 270U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kTripleClick);

  ButtonController long_press;
  raw = {};
  raw[1] = true;
  long_press.Sample(raw, 0U);
  long_press.Sample(raw, 30U);
  CHECK(long_press.PopEvent(&event));
  for (std::uint32_t now_ms = 60U; now_ms <= 1530U; now_ms += 30U) {
    long_press.Sample(raw, now_ms);
  }
  CHECK(long_press.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kLongPress);
  CHECK(event.timestamp_ms == 1530U);
  for (std::uint32_t now_ms = 1560U; now_ms <= 1950U; now_ms += 30U) {
    long_press.Sample(raw, now_ms);
  }
  CHECK(long_press.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kLongPressRepeat);
  CHECK(event.timestamp_ms == 1950U);
}

ButtonEvent SequencedButtonEvent(std::uint32_t sequence) {
  return {sequence,
          static_cast<std::uint8_t>(((sequence - 1U) % kButtonCount) + 1U),
          ButtonEventType::kClick};
}

void CheckButtonEventRange(ButtonEventQueue* queue, std::uint32_t first,
                           std::uint32_t last) {
  ButtonEvent event{};
  for (std::uint32_t expected = first; expected <= last; ++expected) {
    CHECK(queue->TryPop(&event));
    CHECK(event.timestamp_ms == expected);
    CHECK(event.button_id == SequencedButtonEvent(expected).button_id);
    CHECK(event.event == ButtonEventType::kClick);
  }
  CHECK(!queue->TryPop(&event));
}

void TestButtonEventQueueBounds() {
  ButtonEventQueue exactly_full;
  for (std::uint32_t sequence = 1U; sequence <= 16U; ++sequence) {
    CHECK(!exactly_full.PushDropOldest(SequencedButtonEvent(sequence)));
  }
  CHECK(exactly_full.dropped_count() == 0U);
  CheckButtonEventRange(&exactly_full, 1U, 16U);

  ButtonEventQueue seventeen;
  for (std::uint32_t sequence = 1U; sequence <= 17U; ++sequence) {
    CHECK(seventeen.PushDropOldest(SequencedButtonEvent(sequence)) ==
          (sequence == 17U));
  }
  CHECK(seventeen.dropped_count() == 1U);
  CheckButtonEventRange(&seventeen, 2U, 17U);

  ButtonEventQueue thirty_two;
  for (std::uint32_t sequence = 1U; sequence <= 32U; ++sequence) {
    CHECK(thirty_two.PushDropOldest(SequencedButtonEvent(sequence)) ==
          (sequence > 16U));
  }
  CHECK(thirty_two.dropped_count() == 16U);
  CheckButtonEventRange(&thirty_two, 17U, 32U);
}

void TestBatteryMonitor() {
  BatteryMonitor battery;
  BatteryUpdate update = battery.AddSample(10000U, true, 0U);
  CHECK(update.state.valid);
  CHECK(update.state.voltage_mv == 10000U);
  update = battery.AddSample(12000U, true, 50U);
  CHECK(update.state.voltage_mv == 10100U);
  update = battery.AddSample(0U, false, 100U);
  CHECK(!update.state.valid && update.state.voltage_mv == 0U);

  BatteryMonitor battery_presence;
  update = battery_presence.AddSample(kBatteryPresentMinimumMv - 1U, true, 0U);
  CHECK(!update.state.valid && update.state.voltage_mv == 0U);
  CHECK(!update.state.below_threshold && !update.request_alarm_pattern);
  update = battery_presence.AddSample(kBatteryPresentMinimumMv, true, 50U);
  CHECK(update.state.valid &&
        update.state.voltage_mv == kBatteryPresentMinimumMv);

  BatteryMonitor low_battery;
  for (std::uint32_t now_ms = 0U; now_ms <= 10000U; now_ms += 50U) {
    update = low_battery.AddSample(6000U, true, now_ms);
  }
  CHECK(update.state.below_threshold);
  CHECK(update.request_alarm_pattern);
  update = low_battery.AddSample(6000U, true, 20000U);
  CHECK(update.request_alarm_pattern);

  bool cleared = false;
  for (std::uint32_t now_ms = 20050U; now_ms <= 23000U; now_ms += 50U) {
    update = low_battery.AddSample(20000U, true, now_ms);
    if (!update.state.below_threshold) {
      cleared = true;
      break;
    }
  }
  CHECK(cleared);

  const BatteryThresholdUpdate rejected = low_battery.SetLowThreshold(4999U);
  CHECK(rejected.result.code == ResultCode::kOutOfRange);
  const BatteryThresholdUpdate accepted = low_battery.SetLowThreshold(5000U);
  CHECK(accepted.result.ok() && accepted.active_threshold_mv == 5000U);
}

void TestPatterns() {
  LedController leds;
  LedCommand led{kFirstHostLedId, 100U, 50U, 2U};
  CHECK(leds.AcceptCommand(led, 0U).ok());
  CHECK(leds.Update(0U)[kFirstHostLedId - 1U]);
  CHECK(!leds.Update(100U)[kFirstHostLedId - 1U]);
  CHECK(leds.Update(150U)[kFirstHostLedId - 1U]);
  CHECK(!leds.Update(300U)[kFirstHostLedId - 1U]);

  BuzzerController buzzer;
  const BuzzerCommand host{440U, 100U, 100U, 0U};
  CHECK(buzzer.AcceptHostCommand(host, 0U).ok());
  CHECK(buzzer.Update(0U).frequency_hz == 440U);
  buzzer.TriggerBatteryAlarm(10U);
  BuzzerOutput output = buzzer.Update(10U);
  CHECK(output.frequency_hz == 2100U && output.battery_alarm_active);
  output = buzzer.Update(5010U);
  CHECK(!output.battery_alarm_active);
  CHECK(output.frequency_hz == 440U);
}

void TestBusServoCodecAndScheduler() {
  const std::array<std::uint8_t, 4> arguments{0xe8U, 0x03U, 0xe8U, 0x03U};
  BusServoFrame frame{};
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  nullptr)
            .code == ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveStop, nullptr, 1U,
                                  &frame)
            .code == ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::BuildFrame(0U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(255U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(
            1U, BusServoOpcode::kMoveStop, arguments.data(),
            mentor_pi::mcu::kBusServoMaximumArguments + 1U, &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveTimeWrite,
                                  arguments.data(), arguments.size(), &frame)
            .ok());
  CHECK(frame.size == 10U);
  CHECK(frame.bytes[9] == 0x20U);
  ParsedBusServoFrame parsed =
      BusServoCodec::ParseFrame(frame.bytes.data(), frame.size);
  CHECK(parsed.result.ok());
  CHECK(parsed.servo_id == 1U && parsed.argument_count == 4U);

  CHECK(BusServoCodec::ParseFrame(nullptr, 0U).result.code ==
        ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::ParseFrame(frame.bytes.data(), 5U).result.code ==
        ResultCode::kInvalidArgument);
  BusServoFrame malformed = frame;
  malformed.bytes[0] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 1U);
  malformed = frame;
  malformed.bytes[1] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 1U);
  malformed = frame;
  malformed.bytes[3] = 2U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[3] = 12U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[3] = 3U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[2] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.code == ResultCode::kOutOfRange);
  malformed = frame;
  malformed.bytes[2] = 255U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::Checksum(nullptr, 1U) == 0U);
  frame.bytes[9] ^= 1U;
  parsed = BusServoCodec::ParseFrame(frame.bytes.data(), frame.size);
  CHECK(parsed.result.code == ResultCode::kIoError);

  BusServoScheduler scheduler;
  BusServoCommand move{};
  move.count = 2U;
  move.servo_id[0] = 1U;
  move.servo_id[1] = 2U;
  move.position[0] = 100U;
  move.position[1] = 200U;
  move.duration_ms = 500U;
  CHECK(scheduler.SubmitMove(move).result.ok());
  ScheduledBusFrame scheduled = scheduler.BeginFrame();
  CHECK(scheduled.result.ok());
  CHECK(scheduled.kind == ScheduledBusFrameKind::kMove);
  CHECK(scheduled.frame.bytes[2] == 1U);

  BusServoCommand pre_stop_pending{};
  pre_stop_pending.count = 1U;
  pre_stop_pending.servo_id[0] = 5U;
  pre_stop_pending.position[0] = 250U;
  CHECK(scheduler.SubmitMove(pre_stop_pending).result.ok());

  StopBusServosCommand stop{};
  stop.count = 1U;
  stop.servo_id[0] = 9U;
  CHECK(scheduler.AcceptStop(stop).ok());
  BusServoCommand post_stop{};
  post_stop.count = 1U;
  post_stop.servo_id[0] = 7U;
  post_stop.position[0] = 300U;
  CHECK(scheduler.SubmitMove(post_stop).result.ok());
  scheduler.CompleteFrame(true);

  scheduled = scheduler.BeginFrame();
  CHECK(scheduled.kind == ScheduledBusFrameKind::kStop);
  CHECK(scheduled.frame.bytes[2] == 9U);
  scheduler.CompleteFrame(true);
  scheduled = scheduler.BeginFrame();
  CHECK(scheduled.kind == ScheduledBusFrameKind::kMove);
  CHECK(scheduled.frame.bytes[2] == 7U);

  BusServoScheduler edge_scheduler;
  CHECK(!edge_scheduler.has_work());
  CHECK(edge_scheduler.SubmitMove(BusServoCommand{}).result.code ==
        ResultCode::kInvalidArgument);
  CHECK(edge_scheduler.AcceptStop(StopBusServosCommand{}).code ==
        ResultCode::kInvalidArgument);
  const BusMoveAdmission first = edge_scheduler.SubmitMove(move);
  CHECK(first.result.ok() && !first.overwrote_pending);
  const BusMoveAdmission overwritten = edge_scheduler.SubmitMove(move);
  CHECK(overwritten.result.ok() && overwritten.overwrote_pending);
  CHECK(edge_scheduler.move_overwrite_count() == 1U);
  CHECK(edge_scheduler.has_work());
  scheduled = edge_scheduler.BeginFrame();
  CHECK(scheduled.result.ok() && edge_scheduler.frame_in_progress());
  CHECK(edge_scheduler.BeginFrame().result.code == ResultCode::kBusy);
  edge_scheduler.CompleteFrame(false);
  CHECK(!edge_scheduler.has_work());
  edge_scheduler.CompleteFrame(true);

  BusServoScheduler progression;
  CHECK(progression.SubmitMove(move).result.ok());
  CHECK(progression.BeginFrame().batch_index == 0U);
  progression.CompleteFrame(true);
  CHECK(progression.has_work());
  CHECK(progression.BeginFrame().batch_index == 1U);
  progression.CompleteFrame(true);
  CHECK(!progression.has_work());
  CHECK(progression.BeginFrame().result.code == ResultCode::kBusy);

  BusServoScheduler stop_scheduler;
  StopBusServosCommand two_stops{};
  two_stops.count = 2U;
  two_stops.servo_id[0] = 3U;
  two_stops.servo_id[1] = 4U;
  CHECK(stop_scheduler.AcceptStop(two_stops).ok());
  CHECK(stop_scheduler.AcceptStop(two_stops).code == ResultCode::kBusy);
  CHECK(stop_scheduler.BeginFrame().batch_index == 0U);
  stop_scheduler.CompleteFrame(false);
  CHECK(!stop_scheduler.has_work());
  CHECK(stop_scheduler.AcceptStop(two_stops).ok());
  CHECK(stop_scheduler.BeginFrame().batch_index == 0U);
  stop_scheduler.CompleteFrame(true);
  CHECK(stop_scheduler.has_work());
  CHECK(stop_scheduler.BeginFrame().batch_index == 1U);
  stop_scheduler.CompleteFrame(true);
  CHECK(!stop_scheduler.has_work());

  BusServoScheduler cancel_scheduler;
  CHECK(cancel_scheduler.SubmitMove(move).result.ok());
  CHECK(cancel_scheduler.BeginFrame().result.ok());
  cancel_scheduler.CancelAll();
  CHECK(!cancel_scheduler.has_work() && !cancel_scheduler.frame_in_progress());

  BusServoScheduler between_frames;
  CHECK(between_frames.SubmitMove(move).result.ok());
  CHECK(between_frames.BeginFrame().result.ok());
  between_frames.CompleteFrame(true);
  CHECK(between_frames.AcceptStop(stop).ok());
  CHECK(between_frames.BeginFrame().kind == ScheduledBusFrameKind::kStop);
}

void TestCircularDmaPosition() {
  using TinyRing = CircularDmaPosition<8U>;
  static_assert(TinyRing::kHalfSizeBytes == 4U);

  CHECK(TinyRing::IsConsistent(0U, 0U));
  CHECK(TinyRing::IsConsistent(0U, 3U));
  CHECK(!TinyRing::IsConsistent(0U, 4U));
  CHECK(TinyRing::IsConsistent(1U, 4U));
  CHECK(TinyRing::IsConsistent(1U, 7U));
  CHECK(!TinyRing::IsConsistent(1U, 0U));
  CHECK(!TinyRing::IsConsistent(2U, 8U));

  CHECK(TinyRing::Reconstruct(0U, 3U) == 3U);
  CHECK(TinyRing::Reconstruct(1U, 4U) == 4U);
  CHECK(TinyRing::Reconstruct(2U, 0U) == 8U);
  CHECK(TinyRing::Reconstruct(3U, 7U) == 15U);
  CHECK(TinyRing::Reconstruct(4U, 0U) == 16U);

  for (std::uint32_t absolute = 0U; absolute < 1024U; ++absolute) {
    const std::uint32_t boundary_count = absolute / TinyRing::kHalfSizeBytes;
    const std::uint32_t cursor = absolute % 8U;
    CHECK(TinyRing::IsConsistent(boundary_count, cursor));
    CHECK(TinyRing::Reconstruct(boundary_count, cursor) == absolute);
    CHECK(!TinyRing::IsConsistent(boundary_count ^ 1U, cursor));
  }

  // A complete ring between observations remains visible instead of aliasing
  // to zero movement as it would with a cursor-only modulo subtraction.
  const std::uint32_t before = TinyRing::Reconstruct(1U, 6U);
  const std::uint32_t after_one_lap = TinyRing::Reconstruct(3U, 6U);
  CHECK(after_one_lap - before == 8U);

  using TargetRing = CircularDmaPosition<8192U>;
  CHECK(TargetRing::IsConsistent(0xfffffffeU, 4095U));
  CHECK(TargetRing::IsConsistent(0xffffffffU, 4096U));
  const std::uint32_t before_boundary_wrap =
      TargetRing::Reconstruct(0xffffffffU, 8191U);
  const std::uint32_t after_boundary_wrap = TargetRing::Reconstruct(0U, 0U);
  CHECK(after_boundary_wrap - before_boundary_wrap == 1U);
}

void TestUsart1WriteDeadlines() {
  using mentor_pi_mcu::platform::stm32::Usart1WriteDeadlineMs;

  CHECK(Usart1WriteDeadlineMs(23U, 115200U) == 4U);
  CHECK(Usart1WriteDeadlineMs(512U, 115200U) == 47U);
  CHECK(Usart1WriteDeadlineMs(1024U, 115200U) == 91U);
  CHECK(Usart1WriteDeadlineMs(512U, 921600U) == 8U);
  CHECK(Usart1WriteDeadlineMs(1024U, 921600U) == 14U);
  CHECK(Usart1WriteDeadlineMs(512U) == 8U);
}

void TestCircularRxRing() {
  using TinyRing = CircularRxRing<8U>;
  const std::array<std::uint8_t, 8> ring{0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};

  TinyRing exact;
  const TinyRing::ProducerUpdate exact_update = exact.UpdateProducer(4U, true);
  CHECK(exact_update.consistent);
  CHECK(!exact_update.overrun);
  CHECK(exact_update.delta == 4U);
  CHECK(exact_update.occupied == 4U);
  TinyRing::ReadPlan plan = exact.PrepareRead(3U);
  CHECK(!plan.overrun);
  CHECK(plan.copy_length == 3U);
  CHECK(plan.ring_offset == 0U);
  CHECK(plan.first_length == 3U);
  std::array<std::uint8_t, 8> output{};
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 3>{output[0], output[1], output[2]}) ==
        (std::array<std::uint8_t, 3>{0U, 1U, 2U}));
  CHECK(exact.CommitRead(plan));
  CHECK(!exact.CommitRead(plan));
  CHECK(exact.consumer_position() == 3U);
  plan = exact.PrepareRead(output.size());
  CHECK(plan.copy_length == 1U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(output[0] == 3U);
  CHECK(exact.CommitRead(plan));

  // DMA producer progress between the unlocked copy and the critical-section
  // commit is safe only while unread occupancy still fits in the ring.
  TinyRing producer_race_safe;
  CHECK(producer_race_safe.UpdateProducer(4U, true).consistent);
  plan = producer_race_safe.PrepareRead(3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(!producer_race_safe.UpdateProducer(8U, true).overrun);
  CHECK(producer_race_safe.CommitRead(plan));
  CHECK(producer_race_safe.consumer_position() == 3U);

  TinyRing producer_race_overrun;
  CHECK(producer_race_overrun.UpdateProducer(4U, true).consistent);
  plan = producer_race_overrun.PrepareRead(3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(producer_race_overrun.UpdateProducer(9U, true).overrun);
  CHECK(!producer_race_overrun.CommitRead(plan));
  CHECK(producer_race_overrun.consumer_position() == 0U);

  TinyRing wrapped;
  wrapped.ResetPositions(6U);
  const TinyRing::ProducerUpdate wrapped_update =
      wrapped.UpdateProducer(10U, true);
  CHECK(wrapped_update.delta == 4U);
  plan = wrapped.PrepareRead(4U);
  CHECK(plan.ring_offset == 6U);
  CHECK(plan.first_length == 2U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 4>{output[0], output[1], output[2],
                                     output[3]}) ==
        (std::array<std::uint8_t, 4>{6U, 7U, 0U, 1U}));
  CHECK(wrapped.CommitRead(plan));

  TinyRing full;
  const TinyRing::ProducerUpdate full_update = full.UpdateProducer(8U, true);
  CHECK(!full_update.overrun);
  CHECK(full_update.occupied == 8U);
  plan = full.PrepareRead(8U);
  CHECK(!plan.overrun && plan.copy_length == 8U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(output == ring);
  CHECK(full.CommitRead(plan));

  TinyRing overrun;
  const TinyRing::ProducerUpdate overrun_update =
      overrun.UpdateProducer(9U, true);
  CHECK(overrun_update.overrun);
  CHECK(overrun_update.occupied == 9U);
  plan = overrun.PrepareRead(output.size());
  CHECK(plan.overrun && plan.copy_length == 0U);
  CHECK(!TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(!overrun.CommitRead(plan));
  CHECK(overrun.consumer_position() == 0U);
  CHECK(overrun.high_water_bytes() == 9U);
  CHECK(overrun.rx_wire_bytes() == 9U);

  TinyRing inconsistent;
  CHECK(inconsistent.UpdateProducer(3U, true).consistent);
  const std::uint32_t producer_before = inconsistent.producer_position();
  const std::uint32_t previous_before = inconsistent.previous_dma_position();
  const std::uint32_t high_water_before = inconsistent.high_water_bytes();
  const std::uint64_t wire_before = inconsistent.rx_wire_bytes();
  const TinyRing::ProducerUpdate rejected =
      inconsistent.UpdateProducer(99U, false);
  CHECK(!rejected.consistent && !rejected.overrun);
  CHECK(rejected.delta == 0U && rejected.occupied == 0U);
  CHECK(inconsistent.producer_position() == producer_before);
  CHECK(inconsistent.previous_dma_position() == previous_before);
  CHECK(inconsistent.high_water_bytes() == high_water_before);
  CHECK(inconsistent.rx_wire_bytes() == wire_before);

  TinyRing position_wrap;
  constexpr std::uint32_t kBeforeWrap =
      std::numeric_limits<std::uint32_t>::max() - 2U;
  position_wrap.ResetPositions(kBeforeWrap);
  const TinyRing::ProducerUpdate wrap_update =
      position_wrap.UpdateProducer(1U, true);
  CHECK(wrap_update.consistent && !wrap_update.overrun);
  CHECK(wrap_update.delta == 4U && wrap_update.occupied == 4U);
  CHECK(position_wrap.producer_position() == 1U);
  plan = position_wrap.PrepareRead(4U);
  CHECK(plan.ring_offset == 5U && plan.first_length == 3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 4>{output[0], output[1], output[2],
                                     output[3]}) ==
        (std::array<std::uint8_t, 4>{5U, 6U, 7U, 0U}));
  CHECK(position_wrap.CommitRead(plan));
  CHECK(position_wrap.consumer_position() == 1U);

  const std::uint32_t retained_high_water = position_wrap.high_water_bytes();
  const std::uint64_t retained_wire = position_wrap.rx_wire_bytes();
  position_wrap.ResetPositions();
  CHECK(position_wrap.producer_position() == 0U);
  CHECK(position_wrap.consumer_position() == 0U);
  CHECK(position_wrap.previous_dma_position() == 0U);
  CHECK(position_wrap.high_water_bytes() == retained_high_water);
  CHECK(position_wrap.rx_wire_bytes() == retained_wire);

  TinyRing malformed;
  malformed.ResetPositions(6U);
  CHECK(malformed.UpdateProducer(10U, true).consistent);
  const std::uint32_t malformed_consumer = malformed.consumer_position();
  TinyRing::ReadPlan malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.copy_length = 9U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.first_length = 1U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.ring_offset = 7U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.consumer_position += 8U;
  CHECK(TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);
}

void TestPeriodicReleaseSchedule() {
  using mentor_pi_mcu::app::microros::AdvancePeriodicRelease;

  CHECK(AdvancePeriodicRelease(120U, 100U, 20U) == 120U);
  CHECK(AdvancePeriodicRelease(121U, 100U, 20U) == 120U);
  CHECK(AdvancePeriodicRelease(171U, 100U, 20U) == 160U);
  CHECK(AdvancePeriodicRelease(119U, 100U, 20U) == 100U);
  CHECK(AdvancePeriodicRelease(120U, 100U, 0U) == 100U);

  constexpr std::uint32_t kBeforeWrap =
      std::numeric_limits<std::uint32_t>::max() - 9U;
  CHECK(AdvancePeriodicRelease(15U, kBeforeWrap, 20U) == 10U);
}

void TestReclaimingArena() {
  using mentor_pi_mcu::app::microros::ReclaimingArena;

  alignas(std::max_align_t) std::array<std::uint8_t, 1024U> storage{};
  ReclaimingArena arena;
  CHECK(arena.Initialize(storage.data(), storage.size()));
  CHECK(arena.healthy());
  CHECK(arena.bytes_used() == 0U);

  void* const first = arena.Allocate(192U);
  void* const second = arena.Allocate(192U);
  void* const third = arena.Allocate(192U);
  CHECK(first != nullptr && second != nullptr && third != nullptr);
  CHECK(arena.Deallocate(first));
  CHECK(arena.Deallocate(second));
  void* const coalesced = arena.Allocate(384U);
  CHECK(coalesced == first);
  CHECK(arena.Deallocate(coalesced));
  CHECK(arena.Deallocate(third));
  CHECK(arena.bytes_used() == 0U);

  auto* bytes = static_cast<std::uint8_t*>(arena.Allocate(64U));
  CHECK(bytes != nullptr);
  if (bytes != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      bytes[index] = static_cast<std::uint8_t>(index);
    }
  }
  auto* const grown = static_cast<std::uint8_t*>(arena.Reallocate(bytes, 160U));
  CHECK(grown == bytes);
  if (grown != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      CHECK(grown[index] == static_cast<std::uint8_t>(index));
    }
  }
  CHECK(arena.Deallocate(grown));

  auto* const zeroed = static_cast<std::uint8_t*>(arena.ZeroAllocate(8U, 8U));
  CHECK(zeroed != nullptr);
  if (zeroed != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      CHECK(zeroed[index] == 0U);
    }
  }
  CHECK(arena.Deallocate(zeroed));
  CHECK(arena.ZeroAllocate(std::numeric_limits<std::size_t>::max(), 2U) ==
        nullptr);

  arena.Reset();
  CHECK(arena.healthy());
  CHECK(arena.bytes_used() == 0U);
  CHECK(arena.Allocate(900U) != nullptr);
}

}  // namespace
}  // namespace mentor_pi::mcu

int main() {
  mentor_pi::mcu::TestValidationAndStateMerging();
  mentor_pi::mcu::TestValidationBoundaries();
  mentor_pi::mcu::TestFixedContainers();
  mentor_pi::mcu::TestCommandMailboxes();
  mentor_pi::mcu::TestMotorController();
  mentor_pi::mcu::TestFirstOrderAdrcController();
  mentor_pi::mcu::TestNonlinearAdrcAndDirectionalMinimumDrive();
  mentor_pi::mcu::TestNonlinearFalBoundaryAndSign();
  mentor_pi::mcu::TestAlphaOneMatchesLegacyLinearTrajectory();
  mentor_pi::mcu::TestNonlinearObserverEvolutionAndDynamicTerms();
  mentor_pi::mcu::TestMotorLeaseBoundariesAndScheduleModel();
  mentor_pi::mcu::TestPwmServoController();
  mentor_pi::mcu::TestButtons();
  mentor_pi::mcu::TestButtonEventQueueBounds();
  mentor_pi::mcu::TestBatteryMonitor();
  mentor_pi::mcu::TestPatterns();
  mentor_pi::mcu::TestBusServoCodecAndScheduler();
  mentor_pi::mcu::TestCircularDmaPosition();
  mentor_pi::mcu::TestUsart1WriteDeadlines();
  mentor_pi::mcu::TestCircularRxRing();
  mentor_pi::mcu::TestPeriodicReleaseSchedule();
  mentor_pi::mcu::TestReclaimingArena();

  if (mentor_pi::mcu::test_failures != 0U) {
    std::cerr << mentor_pi::mcu::test_failures << " test checks failed\n";
    return 1;
  }
  std::cout << "All MCU domain checks passed\n";
  return 0;
}
