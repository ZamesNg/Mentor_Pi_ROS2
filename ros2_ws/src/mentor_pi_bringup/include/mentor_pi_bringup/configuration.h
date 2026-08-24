// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__CONFIGURATION_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__CONFIGURATION_H_

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace mentor_pi_bringup {

enum class MotorModel : std::uint8_t {
  kJgb520 = 0,
  kJgb37 = 1,
  kJga27 = 2,
  kJgb528 = 3,
};

struct DeploymentConfiguration {
  static constexpr std::uint16_t kDefaultBatteryLowThresholdMv = 6300;

  MotorModel motor_model = MotorModel::kJga27;
  std::array<float, 4> known_velocity_decay_rate_s_inverse{0.0F, 0.0F, 0.0F,
                                                           0.0F};
  std::array<float, 4> input_gain_rps_per_second_per_permille{0.03F, 0.03F,
                                                              0.03F, 0.03F};
  std::array<float, 4> controller_bandwidth_rad_s{4.0F, 4.0F, 4.0F, 4.0F};
  std::array<float, 4> controller_fal_exponent{1.0F, 1.0F, 1.0F, 1.0F};
  std::array<float, 4> controller_fal_threshold_rps{0.1F, 0.1F, 0.1F, 0.1F};
  std::array<float, 4> observer_bandwidth_rad_s{12.0F, 12.0F, 12.0F, 12.0F};
  std::array<float, 4> observer_velocity_fal_exponent{1.0F, 1.0F, 1.0F,
                                                      1.0F};
  std::array<float, 4> observer_disturbance_fal_exponent{1.0F, 1.0F, 1.0F,
                                                         1.0F};
  std::array<float, 4> observer_fal_threshold_rps{0.1F, 0.1F, 0.1F, 0.1F};
  std::array<float, 4> disturbance_leakage_s_inverse{0.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 4> disturbance_estimate_limit_rps_per_second{
      30.0F, 30.0F, 30.0F, 30.0F};
  std::array<float, 4> velocity_filter_new_weight{0.8F, 0.8F, 0.8F, 0.8F};
  std::array<std::uint16_t, 4> positive_minimum_drive_permille{0, 0, 0, 0};
  std::array<std::uint16_t, 4> negative_minimum_drive_permille{0, 0, 0, 0};
  std::array<std::int16_t, 4> pwm_servo_offsets_us{0, 0, 0, 0};
  std::uint16_t battery_low_threshold_mv = kDefaultBatteryLowThresholdMv;

  bool operator==(const DeploymentConfiguration& other) const;
};

// The monostate alternative represents a parameter with a type outside the
// exact deployment schema. Keeping this value type independent of rclcpp makes
// schema validation available to host-native tests.
using ConfigurationValue =
    std::variant<std::monostate, std::string, std::int64_t,
                 std::vector<std::int64_t>, std::vector<double>>;
using ConfigurationMap = std::map<std::string, ConfigurationValue>;

struct ConfigurationValidation {
  bool ok = false;
  DeploymentConfiguration configuration{};
  std::string error;
};

// Nonzero values are stable diagnostic detail codes used when an MCU returns
// OK for SetMotorModel but does not echo the exact effective profile.
enum class MotorProfileResponseError : std::uint8_t {
  kNone = 0,
  kUnknownRequestedModel = 1,
  kActiveModelMismatch = 2,
  kTicksPerRevolutionMismatch = 3,
  kMaxRpsMismatch = 4,
};

// Validates the exact schema from architecture.md (HOST-001). Every key is
// required; unknown keys and values of the wrong variant are rejected.
ConfigurationValidation ValidateConfiguration(
    const ConfigurationMap& parameters);

MotorProfileResponseError ValidateMotorProfileResponse(
    MotorModel requested_model, std::uint8_t active_model,
    std::uint32_t ticks_per_revolution, float max_rps);

const char* MotorModelName(MotorModel model);
std::uint8_t MotorModelWireValue(MotorModel model);
const char* MotorProfileResponseErrorName(MotorProfileResponseError error);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__CONFIGURATION_H_
