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
constexpr const char* kInputGainParameter =
    "input_gain_rps_per_second_per_permille";
constexpr const char* kControllerBandwidthParameter =
    "controller_bandwidth_rad_s";
constexpr const char* kObserverBandwidthParameter = "observer_bandwidth_rad_s";
constexpr const char* kVelocityFilterParameter = "velocity_filter_new_weight";
constexpr const char* kPwmOffsetsParameter = "pwm_servo_offsets_us";
constexpr const char* kBatteryThresholdParameter = "battery_low_threshold_mv";
constexpr std::int64_t kMinimumPwmOffsetUs = -100;
constexpr std::int64_t kMaximumPwmOffsetUs = 100;
constexpr std::int64_t kMinimumBatteryThresholdMv = 5000;
constexpr std::int64_t kMaximumBatteryThresholdMv = 20000;
constexpr double kMaximumAdrcInputGain = 1000.0;
constexpr double kMaximumAdrcObserverBandwidthRadS = 50.0;
constexpr double kMaximumVelocityFilterWeight = 1.0;

bool IsKnownKey(const std::string& key) {
  return key == kMotorModelParameter || key == kInputGainParameter ||
         key == kControllerBandwidthParameter ||
         key == kObserverBandwidthParameter ||
         key == kVelocityFilterParameter || key == kPwmOffsetsParameter ||
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
         input_gain_rps_per_second_per_permille ==
             other.input_gain_rps_per_second_per_permille &&
         controller_bandwidth_rad_s == other.controller_bandwidth_rad_s &&
         observer_bandwidth_rad_s == other.observer_bandwidth_rad_s &&
         velocity_filter_new_weight == other.velocity_filter_new_weight &&
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
  const auto input_gain_entry = parameters.find(kInputGainParameter);
  const auto controller_bandwidth_entry =
      parameters.find(kControllerBandwidthParameter);
  const auto observer_bandwidth_entry =
      parameters.find(kObserverBandwidthParameter);
  const auto filter_entry = parameters.find(kVelocityFilterParameter);
  const auto offsets_entry = parameters.find(kPwmOffsetsParameter);
  const auto threshold_entry = parameters.find(kBatteryThresholdParameter);
  if (model_entry == parameters.end()) {
    return Error("missing required configuration key: motor_model");
  }
  if (input_gain_entry == parameters.end() ||
      controller_bandwidth_entry == parameters.end() ||
      observer_bandwidth_entry == parameters.end() ||
      filter_entry == parameters.end()) {
    return Error("missing required ADRC or filter configuration key");
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
      static_cast<double>(std::numeric_limits<float>::min());
  std::string adrc_error =
      parse_adrc_array(input_gain_entry, kInputGainParameter, kSmallestPositive,
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
  adrc_error =
      parse_adrc_array(observer_bandwidth_entry, kObserverBandwidthParameter,
                       kSmallestPositive, kMaximumAdrcObserverBandwidthRadS,
                       &configuration.observer_bandwidth_rad_s);
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
