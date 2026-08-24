// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/configuration.h"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

// These literals are the contract boundaries and adjacent invalid values under
// test; replacing each with a named production constant would obscure the
// boundary cases this independent test is intended to verify.
// NOLINTBEGIN(readability-magic-numbers)

using mentor_pi_bringup::ConfigurationMap;
using mentor_pi_bringup::ConfigurationValue;
using mentor_pi_bringup::MotorModel;
using mentor_pi_bringup::MotorModelName;
using mentor_pi_bringup::MotorModelWireValue;
using mentor_pi_bringup::MotorProfileResponseError;
using mentor_pi_bringup::MotorProfileResponseErrorName;
using mentor_pi_bringup::ValidateConfiguration;
using mentor_pi_bringup::ValidateMotorProfileResponse;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

ConfigurationMap ValidParameters() {
  return {
      {"battery_low_threshold_mv", std::int64_t{6300}},
      {"known_velocity_decay_rate_s_inverse",
       std::vector<double>{0.0, 0.0, 0.0, 0.0}},
      {"observer_bandwidth_rad_s", std::vector<double>{12.0, 12.0, 12.0, 12.0}},
      {"controller_bandwidth_rad_s", std::vector<double>{4.0, 4.0, 4.0, 4.0}},
      {"controller_fal_exponent", std::vector<double>{1.0, 1.0, 1.0, 1.0}},
      {"controller_fal_threshold_rps",
       std::vector<double>{0.1, 0.1, 0.1, 0.1}},
      {"disturbance_estimate_limit_rps_per_second",
       std::vector<double>{30.0, 30.0, 30.0, 30.0}},
      {"disturbance_leakage_s_inverse",
       std::vector<double>{0.0, 0.0, 0.0, 0.0}},
      {"motor_model", std::string{"JGA27"}},
      {"negative_minimum_drive_permille",
       std::vector<std::int64_t>{0, 0, 0, 0}},
      {"input_gain_rps_per_second_per_permille",
       std::vector<double>{0.03, 0.03, 0.03, 0.03}},
      {"observer_disturbance_fal_exponent",
       std::vector<double>{1.0, 1.0, 1.0, 1.0}},
      {"observer_fal_threshold_rps",
       std::vector<double>{0.1, 0.1, 0.1, 0.1}},
      {"observer_velocity_fal_exponent",
       std::vector<double>{1.0, 1.0, 1.0, 1.0}},
      {"positive_minimum_drive_permille",
       std::vector<std::int64_t>{0, 0, 0, 0}},
      {"pwm_servo_offsets_us", std::vector<std::int64_t>{0, 0, 0, 0}},
      {"velocity_filter_new_weight", std::vector<double>{0.8, 0.8, 0.8, 0.8}}};
}

void TestValidConfiguration() {
  const auto validation = ValidateConfiguration(ValidParameters());
  Expect(validation.ok, "default schema must be valid");
  Expect(validation.configuration.motor_model == MotorModel::kJga27,
         "JGA27 model mapping");
  Expect(validation.configuration.battery_low_threshold_mv == 6300,
         "battery threshold mapping");
  Expect(validation.configuration.known_velocity_decay_rate_s_inverse[0] ==
                 0.0F &&
             validation.configuration
                     .input_gain_rps_per_second_per_permille[0] == 0.03F &&
             validation.configuration.controller_bandwidth_rad_s[1] == 4.0F &&
             validation.configuration.controller_fal_exponent[1] == 1.0F &&
             validation.configuration.controller_fal_threshold_rps[1] ==
                 0.1F &&
             validation.configuration.observer_bandwidth_rad_s[2] == 12.0F &&
             validation.configuration.observer_velocity_fal_exponent[2] ==
                 1.0F &&
             validation.configuration.observer_disturbance_fal_exponent[2] ==
                 1.0F &&
             validation.configuration.observer_fal_threshold_rps[2] == 0.1F &&
             validation.configuration.disturbance_leakage_s_inverse[3] ==
                 0.0F &&
             validation.configuration
                     .disturbance_estimate_limit_rps_per_second[3] == 30.0F &&
             validation.configuration.velocity_filter_new_weight[3] == 0.8F &&
             validation.configuration.positive_minimum_drive_permille[0] ==
                 0U &&
             validation.configuration.negative_minimum_drive_permille[3] ==
                 0U,
         "nonlinear ADRC and minimum-drive arrays map safely");

  const std::vector<std::pair<std::string, MotorModel>> models{
      {"JGB520", MotorModel::kJgb520},
      {"JGB37", MotorModel::kJgb37},
      {"JGA27", MotorModel::kJga27},
      {"JGB528", MotorModel::kJgb528}};
  for (const auto& model : models) {
    auto parameters = ValidParameters();
    parameters["motor_model"] = model.first;
    const auto result = ValidateConfiguration(parameters);
    Expect(result.ok && result.configuration.motor_model == model.second,
           "all exact motor model strings must be accepted");
  }

  auto boundaries = ValidParameters();
  boundaries["pwm_servo_offsets_us"] =
      std::vector<std::int64_t>{-100, 100, -100, 100};
  boundaries["battery_low_threshold_mv"] = std::int64_t{5000};
  auto result = ValidateConfiguration(boundaries);
  Expect(result.ok, "inclusive lower boundaries must be accepted");
  boundaries["battery_low_threshold_mv"] = std::int64_t{20000};
  result = ValidateConfiguration(boundaries);
  Expect(result.ok, "inclusive upper battery boundary must be accepted");
}

void TestExactKeys() {
  auto parameters = ValidParameters();
  parameters["unexpected"] = std::int64_t{1};
  auto result = ValidateConfiguration(parameters);
  Expect(!result.ok && result.error.find("unknown") != std::string::npos,
         "unknown key must be rejected precisely");

  const std::vector<std::string> required_keys{
      "motor_model",
      "known_velocity_decay_rate_s_inverse",
      "input_gain_rps_per_second_per_permille",
      "controller_bandwidth_rad_s",
      "controller_fal_exponent",
      "controller_fal_threshold_rps",
      "observer_bandwidth_rad_s",
      "observer_velocity_fal_exponent",
      "observer_disturbance_fal_exponent",
      "observer_fal_threshold_rps",
      "disturbance_leakage_s_inverse",
      "disturbance_estimate_limit_rps_per_second",
      "velocity_filter_new_weight",
      "positive_minimum_drive_permille",
      "negative_minimum_drive_permille",
      "pwm_servo_offsets_us",
      "battery_low_threshold_mv"};
  for (const auto& key : required_keys) {
    parameters = ValidParameters();
    parameters.erase(key);
    result = ValidateConfiguration(parameters);
    Expect(!result.ok && result.error.find("missing") != std::string::npos,
           "every schema key must be required");
  }
}

void TestTypesAndRanges() {
  auto parameters = ValidParameters();
  parameters["motor_model"] = std::int64_t{2};
  Expect(!ValidateConfiguration(parameters).ok,
         "numeric motor model must be rejected");

  parameters = ValidParameters();
  parameters["motor_model"] = std::string{"jga27"};
  Expect(!ValidateConfiguration(parameters).ok,
         "motor model matching must be case-sensitive");

  parameters = ValidParameters();
  parameters["pwm_servo_offsets_us"] = std::string{"0,0,0,0"};
  Expect(!ValidateConfiguration(parameters).ok,
         "string PWM offset list must be rejected");

  for (const std::size_t count : {std::size_t{3}, std::size_t{5}}) {
    parameters = ValidParameters();
    parameters["pwm_servo_offsets_us"] = std::vector<std::int64_t>(count, 0);
    Expect(!ValidateConfiguration(parameters).ok,
           "PWM offset list must contain exactly four values");
  }

  for (const std::int64_t offset : {std::int64_t{-101}, std::int64_t{101}}) {
    parameters = ValidParameters();
    parameters["pwm_servo_offsets_us"] =
        std::vector<std::int64_t>{0, 0, offset, 0};
    const auto result = ValidateConfiguration(parameters);
    Expect(!result.ok && result.error.find("[2]") != std::string::npos,
           "adjacent PWM range failure must identify its zero-based index");
  }

  parameters = ValidParameters();
  parameters["battery_low_threshold_mv"] = std::string{"6300"};
  Expect(!ValidateConfiguration(parameters).ok,
         "string battery threshold must be rejected");

  for (const std::int64_t threshold :
       {std::int64_t{4999}, std::int64_t{20001}}) {
    parameters = ValidParameters();
    parameters["battery_low_threshold_mv"] = threshold;
    Expect(!ValidateConfiguration(parameters).ok,
           "adjacent battery threshold range must be rejected");
  }

  parameters = ValidParameters();
  parameters["battery_low_threshold_mv"] = ConfigurationValue{};
  Expect(!ValidateConfiguration(parameters).ok,
         "unsupported parameter type must be rejected");

  parameters = ValidParameters();
  parameters["input_gain_rps_per_second_per_permille"] =
      std::vector<double>{0.03, 0.03, 1000.0, 0.03};
  Expect(ValidateConfiguration(parameters).ok,
         "inclusive ADRC input-gain maximum must be accepted");
  parameters["input_gain_rps_per_second_per_permille"] =
      std::vector<double>{0.03, 0.03, 1000.1, 0.03};
  Expect(!ValidateConfiguration(parameters).ok,
         "ADRC input gain above its maximum must be rejected");

  parameters = ValidParameters();
  parameters["observer_bandwidth_rad_s"] =
      std::vector<double>{12.0, 12.0, 50.0, 12.0};
  Expect(ValidateConfiguration(parameters).ok,
         "inclusive ADRC observer-bandwidth maximum must be accepted");
  parameters["observer_bandwidth_rad_s"] =
      std::vector<double>{12.0, 12.0, 50.1, 12.0};
  Expect(!ValidateConfiguration(parameters).ok,
         "ADRC observer bandwidth above its maximum must be rejected");

  for (const std::string key :
       {"input_gain_rps_per_second_per_permille", "controller_bandwidth_rad_s",
        "observer_bandwidth_rad_s"}) {
    parameters = ValidParameters();
    parameters[key] = std::vector<double>{1.0, 0.0, 1.0, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           "zero ADRC dynamics value must be rejected");
    parameters[key] = std::vector<double>{1.0, -0.1, 1.0, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           "negative ADRC dynamics value must be rejected");
    parameters[key] = std::vector<double>{
        1.0, std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           "non-finite ADRC dynamics value must be rejected");
    parameters[key] = std::vector<double>{1.0, 1.0, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           "ADRC array must contain exactly four values");
  }

  parameters = ValidParameters();
  parameters["controller_bandwidth_rad_s"] =
      std::vector<double>{13.0, 4.0, 4.0, 4.0};
  Expect(!ValidateConfiguration(parameters).ok,
         "controller bandwidth above observer bandwidth must be rejected");

  const std::vector<std::string> bounded_zero_to_fifty_keys{
      "known_velocity_decay_rate_s_inverse",
      "disturbance_leakage_s_inverse"};
  for (const auto& key : bounded_zero_to_fifty_keys) {
    parameters = ValidParameters();
    parameters[key] = std::vector<double>{0.0, 50.0, 1.0, 2.0};
    Expect(ValidateConfiguration(parameters).ok,
           key + " inclusive boundaries must be accepted");
    parameters[key] = std::vector<double>{0.0, 50.1, 1.0, 2.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " above 50 must be rejected");
    parameters[key] = std::vector<double>{0.0, -0.1, 1.0, 2.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " below zero must be rejected");
  }

  const std::vector<std::string> exponent_keys{
      "controller_fal_exponent", "observer_velocity_fal_exponent",
      "observer_disturbance_fal_exponent"};
  for (const auto& key : exponent_keys) {
    parameters = ValidParameters();
    parameters[key] = std::vector<double>{0.1, 1.0, 0.5, 0.75};
    Expect(ValidateConfiguration(parameters).ok,
           key + " inclusive boundaries must be accepted");
    parameters[key] = std::vector<double>{0.099, 1.0, 0.5, 0.75};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " below 0.1 must be rejected");
    parameters[key] = std::vector<double>{0.1, 1.001, 0.5, 0.75};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " above one must be rejected");
  }

  const std::vector<std::string> fal_threshold_keys{
      "controller_fal_threshold_rps", "observer_fal_threshold_rps"};
  for (const auto& key : fal_threshold_keys) {
    parameters = ValidParameters();
    parameters[key] = std::vector<double>{0.001, 6.0, 0.1, 1.0};
    Expect(ValidateConfiguration(parameters).ok,
           key + " inclusive boundaries must be accepted");
    parameters[key] = std::vector<double>{0.0009, 6.0, 0.1, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " below 0.001 RPS must be rejected");
    parameters[key] = std::vector<double>{0.001, 6.1, 0.1, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " above 6 RPS must be rejected");
  }

  parameters = ValidParameters();
  parameters["disturbance_estimate_limit_rps_per_second"] =
      std::vector<double>{0.0, 30.0, 30.0, 30.0};
  Expect(ValidateConfiguration(parameters).ok,
         "disturbance-estimate limit accepts zero and b0-scaled maximum");
  parameters["disturbance_estimate_limit_rps_per_second"] =
      std::vector<double>{0.0, 30.1, 30.0, 30.0};
  Expect(!ValidateConfiguration(parameters).ok,
         "disturbance-estimate limit above b0 * 1000 must be rejected");
  parameters = ValidParameters();
  parameters["disturbance_estimate_limit_rps_per_second"] =
      std::vector<double>{0.0, -0.1, 30.0, 30.0};
  Expect(!ValidateConfiguration(parameters).ok,
         "negative disturbance-estimate limit must be rejected");

  for (const std::string key : {"positive_minimum_drive_permille",
                                "negative_minimum_drive_permille"}) {
    parameters = ValidParameters();
    parameters[key] = std::vector<std::int64_t>{0, 250, 1, 249};
    Expect(ValidateConfiguration(parameters).ok,
           key + " inclusive boundaries must be accepted");
    parameters[key] = std::vector<std::int64_t>{0, 251, 1, 249};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " above 250 permille must be rejected");
    parameters[key] = std::vector<std::int64_t>{0, -1, 1, 249};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " below zero must be rejected");
    parameters[key] = std::vector<std::int64_t>{0, 1, 2};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " must contain exactly four integers");
    parameters[key] = std::vector<double>{0.0, 1.0, 2.0, 3.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " must reject a double array");
  }

  const std::vector<std::string> float_array_keys{
      "known_velocity_decay_rate_s_inverse",
      "input_gain_rps_per_second_per_permille",
      "controller_bandwidth_rad_s",
      "controller_fal_exponent",
      "controller_fal_threshold_rps",
      "observer_bandwidth_rad_s",
      "observer_velocity_fal_exponent",
      "observer_disturbance_fal_exponent",
      "observer_fal_threshold_rps",
      "disturbance_leakage_s_inverse",
      "disturbance_estimate_limit_rps_per_second",
      "velocity_filter_new_weight"};
  for (const auto& key : float_array_keys) {
    parameters = ValidParameters();
    parameters[key] = std::vector<double>{
        1.0, std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0};
    Expect(!ValidateConfiguration(parameters).ok,
           key + " must reject a non-finite selected value");
  }

  parameters = ValidParameters();
  parameters["velocity_filter_new_weight"] =
      std::vector<double>{0.0, 1.0, 0.5, 0.5};
  Expect(ValidateConfiguration(parameters).ok,
         "inclusive velocity-filter boundaries must be accepted");
  parameters["velocity_filter_new_weight"] =
      std::vector<double>{0.0, 1.1, 0.5, 0.5};
  Expect(!ValidateConfiguration(parameters).ok,
         "velocity-filter weight above one must be rejected");
}

void TestConfigurationValueSemanticsAndWireMappings() {
  const auto valid = ValidateConfiguration(ValidParameters());
  Expect(valid.ok, "mapping test requires a valid baseline");
  Expect(valid.configuration == valid.configuration,
         "deployment configuration equality must be reflexive");

  auto changed = valid.configuration;
  changed.battery_low_threshold_mv = 6301;
  Expect(!(valid.configuration == changed),
         "battery threshold participates in configuration equality");
  changed = valid.configuration;
  changed.input_gain_rps_per_second_per_permille[0] = 0.031F;
  Expect(!(valid.configuration == changed),
         "LADRC gains participate in configuration equality");
  changed = valid.configuration;
  changed.known_velocity_decay_rate_s_inverse[0] = 1.0F;
  Expect(!(valid.configuration == changed),
         "known velocity decay participates in configuration equality");
  changed = valid.configuration;
  changed.controller_fal_exponent[0] = 0.9F;
  Expect(!(valid.configuration == changed),
         "controller fal exponent participates in configuration equality");
  changed = valid.configuration;
  changed.controller_fal_threshold_rps[0] = 0.2F;
  Expect(!(valid.configuration == changed),
         "controller fal threshold participates in configuration equality");
  changed = valid.configuration;
  changed.observer_velocity_fal_exponent[0] = 0.9F;
  Expect(!(valid.configuration == changed),
         "observer velocity fal exponent participates in equality");
  changed = valid.configuration;
  changed.observer_disturbance_fal_exponent[0] = 0.9F;
  Expect(!(valid.configuration == changed),
         "observer disturbance fal exponent participates in equality");
  changed = valid.configuration;
  changed.observer_fal_threshold_rps[0] = 0.2F;
  Expect(!(valid.configuration == changed),
         "observer fal threshold participates in configuration equality");
  changed = valid.configuration;
  changed.disturbance_leakage_s_inverse[0] = 1.0F;
  Expect(!(valid.configuration == changed),
         "disturbance leakage participates in configuration equality");
  changed = valid.configuration;
  changed.disturbance_estimate_limit_rps_per_second[0] = 29.0F;
  Expect(!(valid.configuration == changed),
         "disturbance limit participates in configuration equality");
  changed = valid.configuration;
  changed.positive_minimum_drive_permille[0] = 1U;
  Expect(!(valid.configuration == changed),
         "positive minimum drive participates in configuration equality");
  changed = valid.configuration;
  changed.negative_minimum_drive_permille[0] = 1U;
  Expect(!(valid.configuration == changed),
         "negative minimum drive participates in configuration equality");
  changed = valid.configuration;
  changed.pwm_servo_offsets_us[3] = 1;
  Expect(!(valid.configuration == changed),
         "PWM offsets participate in configuration equality");
  changed = valid.configuration;
  changed.motor_model = MotorModel::kJgb520;
  Expect(!(valid.configuration == changed),
         "motor model participates in configuration equality");

  const std::vector<std::pair<MotorModel, std::string>> models{
      {MotorModel::kJgb520, "JGB520"},
      {MotorModel::kJgb37, "JGB37"},
      {MotorModel::kJga27, "JGA27"},
      {MotorModel::kJgb528, "JGB528"}};
  for (const auto& model : models) {
    Expect(MotorModelName(model.first) == model.second,
           "every motor model has a stable display name");
    Expect(MotorModelWireValue(model.first) ==
               static_cast<std::uint8_t>(model.first),
           "motor model wire value matches its explicit enum value");
  }
  Expect(
      std::string{MotorModelName(static_cast<MotorModel>(255U))} == "UNKNOWN",
      "unknown motor model has a defensive display value");
}

void TestMotorProfileResponseValidation() {
  struct ProfileCase {
    MotorModel model;
    std::uint32_t ticks_per_revolution;
    float max_rps;
  };
  const std::vector<ProfileCase> profiles{
      {MotorModel::kJgb520, 3960U, 1.5F},
      {MotorModel::kJgb37, 1980U, 3.0F},
      {MotorModel::kJga27, 1040U, 6.0F},
      {MotorModel::kJgb528, 5764U, 1.1F},
  };

  for (const auto& profile : profiles) {
    const auto exact = ValidateMotorProfileResponse(
        profile.model, MotorModelWireValue(profile.model),
        profile.ticks_per_revolution, profile.max_rps);
    Expect(exact == MotorProfileResponseError::kNone,
           "exact returned motor profile must be accepted");

    const auto active_model_mismatch = ValidateMotorProfileResponse(
        profile.model,
        static_cast<std::uint8_t>(MotorModelWireValue(profile.model) ^ 1U),
        profile.ticks_per_revolution, profile.max_rps);
    Expect(active_model_mismatch ==
               MotorProfileResponseError::kActiveModelMismatch,
           "active-model mismatch must fail closed");

    const auto tick_mismatch = ValidateMotorProfileResponse(
        profile.model, MotorModelWireValue(profile.model),
        profile.ticks_per_revolution + 1U, profile.max_rps);
    Expect(
        tick_mismatch == MotorProfileResponseError::kTicksPerRevolutionMismatch,
        "encoder-scale mismatch must fail closed");

    const auto max_rps_mismatch = ValidateMotorProfileResponse(
        profile.model, MotorModelWireValue(profile.model),
        profile.ticks_per_revolution,
        std::nextafter(profile.max_rps,
                       std::numeric_limits<float>::infinity()));
    Expect(max_rps_mismatch == MotorProfileResponseError::kMaxRpsMismatch,
           "adjacent representable speed-limit mismatch must fail closed");
  }

  for (const float invalid_max_rps :
       {std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()}) {
    Expect(ValidateMotorProfileResponse(MotorModel::kJga27, 2U, 1040U,
                                        invalid_max_rps) ==
               MotorProfileResponseError::kMaxRpsMismatch,
           "non-finite returned speed limit must fail closed");
  }

  Expect(ValidateMotorProfileResponse(static_cast<MotorModel>(255U), 255U,
                                      1040U, 6.0F) ==
             MotorProfileResponseError::kUnknownRequestedModel,
         "unknown requested model must fail closed");
  Expect(std::string{MotorProfileResponseErrorName(
             MotorProfileResponseError::kTicksPerRevolutionMismatch)} ==
             "TICKS_PER_REVOLUTION_MISMATCH",
         "profile response failure has a stable diagnostic name");
}

// NOLINTEND(readability-magic-numbers)

}  // namespace

int main() {
  try {
    TestValidConfiguration();
    TestExactKeys();
    TestTypesAndRanges();
    TestConfigurationValueSemanticsAndWireMappings();
    TestMotorProfileResponseValidation();
    if (g_failures != 0) {
      std::cerr << g_failures << " configuration tests failed\n";
      return 1;
    }
    std::cout << "configuration tests passed\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "unexpected configuration-test exception: " << exception.what()
              << '\n';
    return 1;
  } catch (...) {
    std::cerr << "unexpected non-standard configuration-test exception\n";
    return 1;
  }
}
