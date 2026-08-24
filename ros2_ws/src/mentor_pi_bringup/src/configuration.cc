// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/configuration.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <utility>

#include "mentor_pi_interfaces/motor_profile_contract.hpp"

namespace mentor_pi_bringup {
namespace {

constexpr const char* kMotorModelParameter = "motor_model";
constexpr const char* kKnownVelocityDecayRateParameter =
    "known_velocity_decay_rate_s_inverse";
constexpr const char* kInputGainParameter =
    "input_gain_rps_per_second_per_permille";
constexpr const char* kControllerBandwidthParameter =
    "controller_bandwidth_rad_s";
constexpr const char* kControllerFalExponentParameter =
    "controller_fal_exponent";
constexpr const char* kControllerFalThresholdParameter =
    "controller_fal_threshold_rps";
constexpr const char* kObserverBandwidthParameter = "observer_bandwidth_rad_s";
constexpr const char* kObserverVelocityFalExponentParameter =
    "observer_velocity_fal_exponent";
constexpr const char* kObserverDisturbanceFalExponentParameter =
    "observer_disturbance_fal_exponent";
constexpr const char* kObserverFalThresholdParameter =
    "observer_fal_threshold_rps";
constexpr const char* kDisturbanceLeakageParameter =
    "disturbance_leakage_s_inverse";
constexpr const char* kDisturbanceEstimateLimitParameter =
    "disturbance_estimate_limit_rps_per_second";
constexpr const char* kVelocityFilterParameter = "velocity_filter_new_weight";
constexpr const char* kPositiveMinimumDriveParameter =
    "positive_minimum_drive_permille";
constexpr const char* kNegativeMinimumDriveParameter =
    "negative_minimum_drive_permille";
constexpr const char* kPwmOffsetsParameter = "pwm_servo_offsets_us";
constexpr const char* kBatteryThresholdParameter = "battery_low_threshold_mv";
constexpr std::int64_t kMinimumPwmOffsetUs = -100;
constexpr std::int64_t kMaximumPwmOffsetUs = 100;
constexpr std::int64_t kMinimumBatteryThresholdMv = 5000;
constexpr std::int64_t kMaximumBatteryThresholdMv = 20000;
constexpr double kMaximumAdrcInputGain = 1000.0;
constexpr double kMaximumAdrcObserverBandwidthRadS = 50.0;
constexpr double kMaximumKnownVelocityDecayRateSInverse = 50.0;
constexpr double kMinimumFalExponent = 0.1;
constexpr double kMaximumFalExponent = 1.0;
constexpr double kMinimumFalThresholdRps = 0.001;
constexpr double kMaximumFalThresholdRps = 6.0;
constexpr double kMaximumDisturbanceLeakageSInverse = 50.0;
constexpr double kMaximumVelocityFilterWeight = 1.0;
constexpr std::int64_t kMaximumMinimumDrivePermille = 250;

bool IsKnownKey(const std::string& key) {
  return key == kMotorModelParameter ||
         key == kKnownVelocityDecayRateParameter ||
         key == kInputGainParameter ||
         key == kControllerBandwidthParameter ||
         key == kControllerFalExponentParameter ||
         key == kControllerFalThresholdParameter ||
         key == kObserverBandwidthParameter ||
         key == kObserverVelocityFalExponentParameter ||
         key == kObserverDisturbanceFalExponentParameter ||
         key == kObserverFalThresholdParameter ||
         key == kDisturbanceLeakageParameter ||
         key == kDisturbanceEstimateLimitParameter ||
         key == kVelocityFilterParameter ||
         key == kPositiveMinimumDriveParameter ||
         key == kNegativeMinimumDriveParameter ||
         key == kPwmOffsetsParameter ||
         key == kBatteryThresholdParameter;
}

ConfigurationValidation Error(std::string message) {
  ConfigurationValidation validation;
  validation.error = std::move(message);
  return validation;
}

bool ParseMotorModel(const std::string& value, MotorModel* model) {
  if (value == "JGB520") {
    *model = MotorModel::kJgb520;
    return true;
  }
  if (value == "JGB37") {
    *model = MotorModel::kJgb37;
    return true;
  }
  if (value == "JGA27") {
    *model = MotorModel::kJga27;
    return true;
  }
  if (value == "JGB528") {
    *model = MotorModel::kJgb528;
    return true;
  }
  return false;
}

}  // namespace

bool DeploymentConfiguration::operator==(
    const DeploymentConfiguration& other) const {
  return motor_model == other.motor_model &&
         known_velocity_decay_rate_s_inverse ==
             other.known_velocity_decay_rate_s_inverse &&
         input_gain_rps_per_second_per_permille ==
             other.input_gain_rps_per_second_per_permille &&
         controller_bandwidth_rad_s == other.controller_bandwidth_rad_s &&
         controller_fal_exponent == other.controller_fal_exponent &&
         controller_fal_threshold_rps == other.controller_fal_threshold_rps &&
         observer_bandwidth_rad_s == other.observer_bandwidth_rad_s &&
         observer_velocity_fal_exponent ==
             other.observer_velocity_fal_exponent &&
         observer_disturbance_fal_exponent ==
             other.observer_disturbance_fal_exponent &&
         observer_fal_threshold_rps == other.observer_fal_threshold_rps &&
         disturbance_leakage_s_inverse ==
             other.disturbance_leakage_s_inverse &&
         disturbance_estimate_limit_rps_per_second ==
             other.disturbance_estimate_limit_rps_per_second &&
         velocity_filter_new_weight == other.velocity_filter_new_weight &&
         positive_minimum_drive_permille ==
             other.positive_minimum_drive_permille &&
         negative_minimum_drive_permille ==
             other.negative_minimum_drive_permille &&
         pwm_servo_offsets_us == other.pwm_servo_offsets_us &&
         battery_low_threshold_mv == other.battery_low_threshold_mv;
}

ConfigurationValidation ValidateConfiguration(
    const ConfigurationMap& parameters) {
  for (const auto& parameter : parameters) {
    if (!IsKnownKey(parameter.first)) {
      return Error("unknown configuration key: " + parameter.first);
    }
  }

  const auto model_entry = parameters.find(kMotorModelParameter);
  const auto known_velocity_decay_entry =
      parameters.find(kKnownVelocityDecayRateParameter);
  const auto input_gain_entry = parameters.find(kInputGainParameter);
  const auto controller_bandwidth_entry =
      parameters.find(kControllerBandwidthParameter);
  const auto controller_fal_exponent_entry =
      parameters.find(kControllerFalExponentParameter);
  const auto controller_fal_threshold_entry =
      parameters.find(kControllerFalThresholdParameter);
  const auto observer_bandwidth_entry =
      parameters.find(kObserverBandwidthParameter);
  const auto observer_velocity_fal_exponent_entry =
      parameters.find(kObserverVelocityFalExponentParameter);
  const auto observer_disturbance_fal_exponent_entry =
      parameters.find(kObserverDisturbanceFalExponentParameter);
  const auto observer_fal_threshold_entry =
      parameters.find(kObserverFalThresholdParameter);
  const auto disturbance_leakage_entry =
      parameters.find(kDisturbanceLeakageParameter);
  const auto disturbance_estimate_limit_entry =
      parameters.find(kDisturbanceEstimateLimitParameter);
  const auto filter_entry = parameters.find(kVelocityFilterParameter);
  const auto positive_minimum_drive_entry =
      parameters.find(kPositiveMinimumDriveParameter);
  const auto negative_minimum_drive_entry =
      parameters.find(kNegativeMinimumDriveParameter);
  const auto offsets_entry = parameters.find(kPwmOffsetsParameter);
  const auto threshold_entry = parameters.find(kBatteryThresholdParameter);
  if (model_entry == parameters.end()) {
    return Error("missing required configuration key: motor_model");
  }
  if (known_velocity_decay_entry == parameters.end() ||
      input_gain_entry == parameters.end() ||
      controller_bandwidth_entry == parameters.end() ||
      controller_fal_exponent_entry == parameters.end() ||
      controller_fal_threshold_entry == parameters.end() ||
      observer_bandwidth_entry == parameters.end() ||
      observer_velocity_fal_exponent_entry == parameters.end() ||
      observer_disturbance_fal_exponent_entry == parameters.end() ||
      observer_fal_threshold_entry == parameters.end() ||
      disturbance_leakage_entry == parameters.end() ||
      disturbance_estimate_limit_entry == parameters.end() ||
      filter_entry == parameters.end() ||
      positive_minimum_drive_entry == parameters.end() ||
      negative_minimum_drive_entry == parameters.end()) {
    return Error("missing required motor-control configuration key");
  }
  if (offsets_entry == parameters.end()) {
    return Error("missing required configuration key: pwm_servo_offsets_us");
  }
  if (threshold_entry == parameters.end()) {
    return Error(
        "missing required configuration key: battery_low_threshold_mv");
  }

  const auto* model_name = std::get_if<std::string>(&model_entry->second);
  if (model_name == nullptr) {
    return Error("motor_model must be a string");
  }

  DeploymentConfiguration configuration;
  if (!ParseMotorModel(*model_name, &configuration.motor_model)) {
    return Error("motor_model must be JGB520, JGB37, JGA27, or JGB528");
  }

  const auto parse_adrc_array =
      [](const ConfigurationMap::const_iterator& entry, const char* key,
         double minimum, double maximum,
         std::array<float, 4>* destination) -> std::string {
    const auto* values = std::get_if<std::vector<double>>(&entry->second);
    if (values == nullptr) {
      return std::string(key) + " must be a double array";
    }
    if (values->size() != destination->size()) {
      return std::string(key) + " must contain exactly four doubles";
    }
    for (std::size_t index = 0; index < values->size(); ++index) {
      const double value = (*values)[index];
      if (!std::isfinite(value) || value < minimum || value > maximum ||
          value > static_cast<double>(std::numeric_limits<float>::max())) {
        std::ostringstream message;
        message << key << "[" << index << "] must be finite and in [" << minimum
                << ", " << maximum << "]";
        return message.str();
      }
      (*destination)[index] = static_cast<float>(value);
    }
    return {};
  };
  constexpr double kSmallestPositive =
      static_cast<double>(std::numeric_limits<float>::denorm_min());
  std::string adrc_error = parse_adrc_array(
      known_velocity_decay_entry, kKnownVelocityDecayRateParameter, 0.0,
      kMaximumKnownVelocityDecayRateSInverse,
      &configuration.known_velocity_decay_rate_s_inverse);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      input_gain_entry, kInputGainParameter, kSmallestPositive,
      kMaximumAdrcInputGain,
      &configuration.input_gain_rps_per_second_per_permille);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      controller_bandwidth_entry, kControllerBandwidthParameter,
      kSmallestPositive, kMaximumAdrcObserverBandwidthRadS,
      &configuration.controller_bandwidth_rad_s);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      controller_fal_exponent_entry, kControllerFalExponentParameter,
      kMinimumFalExponent, kMaximumFalExponent,
      &configuration.controller_fal_exponent);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      controller_fal_threshold_entry, kControllerFalThresholdParameter,
      kMinimumFalThresholdRps, kMaximumFalThresholdRps,
      &configuration.controller_fal_threshold_rps);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error =
      parse_adrc_array(observer_bandwidth_entry, kObserverBandwidthParameter,
                       kSmallestPositive, kMaximumAdrcObserverBandwidthRadS,
                       &configuration.observer_bandwidth_rad_s);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      observer_velocity_fal_exponent_entry,
      kObserverVelocityFalExponentParameter, kMinimumFalExponent,
      kMaximumFalExponent, &configuration.observer_velocity_fal_exponent);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      observer_disturbance_fal_exponent_entry,
      kObserverDisturbanceFalExponentParameter, kMinimumFalExponent,
      kMaximumFalExponent, &configuration.observer_disturbance_fal_exponent);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      observer_fal_threshold_entry, kObserverFalThresholdParameter,
      kMinimumFalThresholdRps, kMaximumFalThresholdRps,
      &configuration.observer_fal_threshold_rps);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      disturbance_leakage_entry, kDisturbanceLeakageParameter, 0.0,
      kMaximumDisturbanceLeakageSInverse,
      &configuration.disturbance_leakage_s_inverse);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(
      disturbance_estimate_limit_entry, kDisturbanceEstimateLimitParameter,
      0.0, static_cast<double>(std::numeric_limits<float>::max()),
      &configuration.disturbance_estimate_limit_rps_per_second);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  adrc_error = parse_adrc_array(filter_entry, kVelocityFilterParameter, 0.0,
                                kMaximumVelocityFilterWeight,
                                &configuration.velocity_filter_new_weight);
  if (!adrc_error.empty()) {
    return Error(adrc_error);
  }
  for (std::size_t index = 0;
       index < configuration.controller_bandwidth_rad_s.size(); ++index) {
    if (configuration.controller_bandwidth_rad_s[index] >
        configuration.observer_bandwidth_rad_s[index]) {
      std::ostringstream message;
      message << kControllerBandwidthParameter << "[" << index
              << "] must not exceed " << kObserverBandwidthParameter << "["
              << index << "]";
      return Error(message.str());
    }
    if (configuration.disturbance_estimate_limit_rps_per_second[index] >
        configuration.input_gain_rps_per_second_per_permille[index] *
            1000.0F) {
      std::ostringstream message;
      message << kDisturbanceEstimateLimitParameter << "[" << index
              << "] must not exceed " << kInputGainParameter << "[" << index
              << "] * 1000";
      return Error(message.str());
    }
  }

  const auto parse_minimum_drive_array =
      [](const ConfigurationMap::const_iterator& entry, const char* key,
         std::array<std::uint16_t, 4>* destination) -> std::string {
    const auto* values =
        std::get_if<std::vector<std::int64_t>>(&entry->second);
    if (values == nullptr) {
      return std::string(key) + " must be an integer array";
    }
    if (values->size() != destination->size()) {
      return std::string(key) + " must contain exactly four integers";
    }
    for (std::size_t index = 0; index < values->size(); ++index) {
      const std::int64_t value = (*values)[index];
      if (value < 0 || value > kMaximumMinimumDrivePermille) {
        std::ostringstream message;
        message << key << "[" << index << "] must be in [0, "
                << kMaximumMinimumDrivePermille << "]";
        return message.str();
      }
      (*destination)[index] = static_cast<std::uint16_t>(value);
    }
    return {};
  };
  std::string minimum_drive_error = parse_minimum_drive_array(
      positive_minimum_drive_entry, kPositiveMinimumDriveParameter,
      &configuration.positive_minimum_drive_permille);
  if (!minimum_drive_error.empty()) {
    return Error(minimum_drive_error);
  }
  minimum_drive_error = parse_minimum_drive_array(
      negative_minimum_drive_entry, kNegativeMinimumDriveParameter,
      &configuration.negative_minimum_drive_permille);
  if (!minimum_drive_error.empty()) {
    return Error(minimum_drive_error);
  }

  const auto* offsets =
      std::get_if<std::vector<std::int64_t>>(&offsets_entry->second);
  if (offsets == nullptr) {
    return Error("pwm_servo_offsets_us must be an integer array");
  }
  if (offsets->size() != configuration.pwm_servo_offsets_us.size()) {
    return Error("pwm_servo_offsets_us must contain exactly four integers");
  }
  for (std::size_t index = 0; index < offsets->size(); ++index) {
    const std::int64_t offset = (*offsets)[index];
    if (offset < kMinimumPwmOffsetUs || offset > kMaximumPwmOffsetUs) {
      std::ostringstream message;
      message << "pwm_servo_offsets_us[" << index << "] must be in [-100, 100]";
      return Error(message.str());
    }
    configuration.pwm_servo_offsets_us[index] =
        static_cast<std::int16_t>(offset);
  }

  const auto* threshold = std::get_if<std::int64_t>(&threshold_entry->second);
  if (threshold == nullptr) {
    return Error("battery_low_threshold_mv must be an integer");
  }
  if (*threshold < kMinimumBatteryThresholdMv ||
      *threshold > kMaximumBatteryThresholdMv) {
    return Error("battery_low_threshold_mv must be in [5000, 20000]");
  }
  static_assert(kMaximumBatteryThresholdMv <=
                std::numeric_limits<std::uint16_t>::max());
  configuration.battery_low_threshold_mv =
      static_cast<std::uint16_t>(*threshold);

  ConfigurationValidation validation;
  validation.ok = true;
  validation.configuration = configuration;
  return validation;
}

MotorProfileResponseError ValidateMotorProfileResponse(
    MotorModel requested_model, std::uint8_t active_model,
    std::uint32_t ticks_per_revolution, float max_rps) {
  const auto requested_wire_model = MotorModelWireValue(requested_model);
  const auto* expected =
      mentor_pi_interfaces::FindMotorProfileContract(requested_wire_model);
  if (expected == nullptr) {
    return MotorProfileResponseError::kUnknownRequestedModel;
  }
  if (active_model != expected->model) {
    return MotorProfileResponseError::kActiveModelMismatch;
  }
  if (ticks_per_revolution != expected->ticks_per_revolution) {
    return MotorProfileResponseError::kTicksPerRevolutionMismatch;
  }
  if (!std::isfinite(max_rps) || max_rps != expected->max_rps) {
    return MotorProfileResponseError::kMaxRpsMismatch;
  }
  return MotorProfileResponseError::kNone;
}

const char* MotorModelName(MotorModel model) {
  switch (model) {
    case MotorModel::kJgb520:
      return "JGB520";
    case MotorModel::kJgb37:
      return "JGB37";
    case MotorModel::kJga27:
      return "JGA27";
    case MotorModel::kJgb528:
      return "JGB528";
  }
  return "UNKNOWN";
}

std::uint8_t MotorModelWireValue(MotorModel model) {
  return static_cast<std::uint8_t>(model);
}

const char* MotorProfileResponseErrorName(MotorProfileResponseError error) {
  switch (error) {
    case MotorProfileResponseError::kNone:
      return "NONE";
    case MotorProfileResponseError::kUnknownRequestedModel:
      return "UNKNOWN_REQUESTED_MODEL";
    case MotorProfileResponseError::kActiveModelMismatch:
      return "ACTIVE_MODEL_MISMATCH";
    case MotorProfileResponseError::kTicksPerRevolutionMismatch:
      return "TICKS_PER_REVOLUTION_MISMATCH";
    case MotorProfileResponseError::kMaxRpsMismatch:
      return "MAX_RPS_MISMATCH";
  }
  return "UNKNOWN_MOTOR_PROFILE_RESPONSE_ERROR";
}

}  // namespace mentor_pi_bringup
