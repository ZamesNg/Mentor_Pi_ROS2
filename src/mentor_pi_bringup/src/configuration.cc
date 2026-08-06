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
constexpr const char* kPwmOffsetsParameter = "pwm_servo_offsets_us";
constexpr const char* kBatteryThresholdParameter = "battery_low_threshold_mv";
constexpr std::int64_t kMinimumPwmOffsetUs = -100;
constexpr std::int64_t kMaximumPwmOffsetUs = 100;
constexpr std::int64_t kMinimumBatteryThresholdMv = 5000;
constexpr std::int64_t kMaximumBatteryThresholdMv = 20000;

bool IsKnownKey(const std::string& key) {
  return key == kMotorModelParameter || key == kPwmOffsetsParameter ||
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
  const auto offsets_entry = parameters.find(kPwmOffsetsParameter);
  const auto threshold_entry = parameters.find(kBatteryThresholdParameter);
  if (model_entry == parameters.end()) {
    return Error("missing required configuration key: motor_model");
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
