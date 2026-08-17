#include "mentor_pi_mcu/domain/motor_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "mentor_pi_interfaces/motor_profile_contract.hpp"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

// Every model intentionally shares the same first-order LADRC starting values.
// They are safe-bounded but remain release-provisional until D3 motor HIL
// records the final gains, filter, and minimum-drive floor for each physical
// motor profile.
constexpr const auto& kJgb520Contract =
    mentor_pi_interfaces::kMotorProfileContracts[0];
constexpr const auto& kJgb37Contract =
    mentor_pi_interfaces::kMotorProfileContracts[1];
constexpr const auto& kJga27Contract =
    mentor_pi_interfaces::kMotorProfileContracts[2];
constexpr const auto& kJgb528Contract =
    mentor_pi_interfaces::kMotorProfileContracts[3];

static_assert(kJgb520Contract.model ==
              static_cast<std::uint8_t>(MotorModel::kJgb520));
static_assert(kJgb37Contract.model ==
              static_cast<std::uint8_t>(MotorModel::kJgb37));
static_assert(kJga27Contract.model ==
              static_cast<std::uint8_t>(MotorModel::kJga27));
static_assert(kJgb528Contract.model ==
              static_cast<std::uint8_t>(MotorModel::kJgb528));

constexpr std::array<MotorProfile, 4> kMotorProfiles{{
    {MotorModel::kJgb520,
     kJgb520Contract.ticks_per_revolution,
     kJgb520Contract.max_rps,
     {}},
    {MotorModel::kJgb37,
     kJgb37Contract.ticks_per_revolution,
     kJgb37Contract.max_rps,
     {}},
    {MotorModel::kJga27,
     kJga27Contract.ticks_per_revolution,
     kJga27Contract.max_rps,
     {}},
    {MotorModel::kJgb528,
     kJgb528Contract.ticks_per_revolution,
     kJgb528Contract.max_rps,
     {}},
}};

float ClampOutput(float candidate_output, float output_limit) {
  return std::max(-output_limit, std::min(output_limit, candidate_output));
}

std::int64_t SaturatingAdd(std::int64_t value, std::int32_t increment) {
  if (increment > 0 &&
      value > std::numeric_limits<std::int64_t>::max() - increment) {
    return std::numeric_limits<std::int64_t>::max();
  }
  if (increment < 0 &&
      value < std::numeric_limits<std::int64_t>::min() - increment) {
    return std::numeric_limits<std::int64_t>::min();
  }
  return value + increment;
}

}  // namespace

const MotorProfile& GetMotorProfile(MotorModel model) {
  if (!IsValidMotorModel(model)) {
    return kMotorProfiles[static_cast<std::size_t>(MotorModel::kJga27)];
  }
  return kMotorProfiles[static_cast<std::size_t>(model)];
}

MotorController::MotorController(MotorControlConfiguration configuration)
    : configuration_(configuration),
      profile_(&GetMotorProfile(MotorModel::kJga27)) {
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const std::uint8_t bits = configuration_.counter_bits[index];
    if (bits != 16U && bits != 32U) {
      configuration_.counter_bits[index] = 16U;
      configuration_valid_ = false;
    }
  }
  if (!std::isfinite(configuration_.maximum_accepted_rps) ||
      configuration_.maximum_accepted_rps <= 0.0F ||
      configuration_.maximum_accepted_rps > kMotorImplementationMaximumRps ||
      configuration_.output_limit_permille <= 0 ||
      configuration_.output_limit_permille > kMotorOutputLimitPermille) {
    configuration_valid_ = false;
  }
  if (!configuration_valid_) {
    configuration_.maximum_accepted_rps = 0.0F;
    configuration_.output_limit_permille = 0;
  }
}

void MotorController::SetSessionActive(bool active) {
  if (!active) {
    DisarmAll(true);
  }
  session_active_ = active;
}

Result MotorController::AcceptCommand(const MotorCommand& command,
                                      std::uint32_t now_us) {
  if (!session_active_) {
    RecordRejectedCommand(command.update_mask);
    return {ResultCode::kBusy, 0};
  }
  const Result result = ValidateMotorCommand(command, profile_->max_rps);
  if (!result.ok()) {
    RecordRejectedCommand(command.update_mask);
    return result;
  }

  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U ||
        command.target_rps[index] == 0.0F) {
      continue;
    }
    if (!configuration_valid_) {
      RecordRejectedCommand(command.update_mask);
      return {ResultCode::kUnsupported, static_cast<std::uint16_t>(index + 1U)};
    }
    if (std::fabs(command.target_rps[index]) >
        configuration_.maximum_accepted_rps) {
      RecordRejectedCommand(command.update_mask);
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    MotorChannelState& channel = channels_[index];
    channel.target_rps = command.target_rps[index];
    channel.watchdog_stopped = false;
    last_command_us_[index] = now_us;
    if (channel.target_rps == 0.0F) {
      StopChannel(index, false);
      channel.watchdog_stopped = false;
    } else {
      channel.armed = true;
    }
  }
  return OkResult();
}

MotorModelChange MotorController::SetModel(MotorModel model) {
  if (!IsValidMotorModel(model)) {
    return {{ResultCode::kInvalidArgument, 0}, *profile_};
  }
  if (model == profile_->model) {
    return {OkResult(), *profile_};
  }
  for (const MotorChannelState& channel : channels_) {
    if (channel.target_rps != 0.0F) {
      return {{ResultCode::kBusy, 0}, *profile_};
    }
  }
  profile_ = &GetMotorProfile(model);
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    ResetAdrc(index);
    channels_[index].measured_rps = 0.0F;
    adrc_overrides_[index] = {};
  }
  // Tick scale is a model property. Establish a fresh counter baseline on the
  // next sample so changing it cannot turn the pre-change interval into a
  // false velocity impulse.
  encoder_initialized_ = false;
  return {OkResult(), *profile_};
}

MotorAdrcUpdate MotorController::SetAdrc(const SetMotorAdrcCommand& command) {
  const Result validation = ValidateSetMotorAdrcCommand(command);
  if (!validation.ok()) {
    return {validation, 0U};
  }
  if (!configuration_valid_) {
    return {{ResultCode::kUnsupported, 0U}, 0U};
  }

  for (const MotorChannelState& channel : channels_) {
    if (channel.armed || channel.target_rps != 0.0F ||
        std::fabs(channel.measured_rps) >= kMotorAdrcUpdateMaximumMeasuredRps) {
      return {{ResultCode::kBusy, 0U}, 0U};
    }
  }

  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    adrc_overrides_[index].active = true;
    adrc_overrides_[index].calibration = {
        command.input_gain_rps_per_second_per_permille[index],
        command.controller_bandwidth_rad_s[index],
        command.observer_bandwidth_rad_s[index],
        command.velocity_filter_new_weight[index]};
    ResetAdrc(index);
  }
  return {OkResult(), command.update_mask};
}

void MotorController::EvaluateLeases(std::uint32_t now_us) {
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    MotorChannelState& channel = channels_[index];
    if (!channel.armed || channel.target_rps == 0.0F) {
      continue;
    }
    const std::uint32_t age_us = now_us - last_command_us_[index];
    if (age_us >= kMotorLeaseExpiryUs) {
      StopChannel(index, true);
      lease_expiry_count_[index].Increment();
    }
  }
}

std::array<std::int16_t, kMotorCount> MotorController::ControlStep(
    const std::array<std::uint32_t, kMotorCount>& raw_encoder_counters,
    std::uint32_t period_us) {
  std::array<std::int16_t, kMotorCount> outputs{};
  if (period_us == 0U) {
    DisarmAll(true);
    return outputs;
  }
  if (!encoder_initialized_) {
    previous_counter_ = raw_encoder_counters;
    encoder_initialized_ = true;
  }

  const float period_seconds = static_cast<float>(period_us) / 1000000.0F;
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const std::int32_t encoder_delta = SignedCounterDelta(
        raw_encoder_counters[index], previous_counter_[index],
        configuration_.counter_bits[index]);
    previous_counter_[index] = raw_encoder_counters[index];
    MotorChannelState& channel = channels_[index];
    channel.encoder_count = SaturatingAdd(channel.encoder_count, encoder_delta);

    const float instantaneous_rps =
        static_cast<float>(encoder_delta) /
        (static_cast<float>(profile_->ticks_per_revolution) * period_seconds);
    const AdrcCalibration calibration = adrc_overrides_[index].active
                                            ? adrc_overrides_[index].calibration
                                            : profile_->adrc;
    const float filter_weight = calibration.velocity_filter_new_weight;
    channel.measured_rps = filter_weight * instantaneous_rps +
                           (1.0F - filter_weight) * channel.measured_rps;

    if (!session_active_ || !channel.armed || channel.target_rps == 0.0F) {
      StopChannel(index, channel.watchdog_stopped);
      outputs[index] = 0;
      continue;
    }
    if (calibration.observer_bandwidth_rad_s * period_seconds > 0.5F) {
      StopChannel(index, true);
      outputs[index] = 0;
      continue;
    }

    AdrcState& state = adrc_state_[index];
    const float observer_error =
        state.observed_velocity_rps - channel.measured_rps;
    const float observer_bandwidth = calibration.observer_bandwidth_rad_s;
    state.observed_velocity_rps +=
        period_seconds * (state.observed_disturbance_rps_per_second +
                          calibration.input_gain_rps_per_second_per_permille *
                              state.applied_output_permille -
                          2.0F * observer_bandwidth * observer_error);
    state.observed_disturbance_rps_per_second +=
        period_seconds *
        (-observer_bandwidth * observer_bandwidth * observer_error);
    if (!std::isfinite(state.observed_velocity_rps) ||
        !std::isfinite(state.observed_disturbance_rps_per_second)) {
      StopChannel(index, true);
      outputs[index] = 0;
      continue;
    }

    const float candidate =
        (calibration.controller_bandwidth_rad_s *
             (channel.target_rps - state.observed_velocity_rps) -
         state.observed_disturbance_rps_per_second) /
        calibration.input_gain_rps_per_second_per_permille;
    if (!std::isfinite(candidate)) {
      StopChannel(index, true);
      outputs[index] = 0;
      continue;
    }
    const float output_limit =
        static_cast<float>(configuration_.output_limit_permille);
    const float output = ClampOutput(candidate, output_limit);

    const float absolute_output = std::fabs(output);
    const std::int16_t effective_minimum_drive = std::min(
        kMotorMinimumDrivePermille, configuration_.output_limit_permille);
    if (absolute_output > 0.0F &&
        absolute_output < static_cast<float>(effective_minimum_drive)) {
      channel.output_permille =
          output > 0.0F ? effective_minimum_drive
                        : static_cast<std::int16_t>(-effective_minimum_drive);
    } else {
      channel.output_permille = static_cast<std::int16_t>(std::lround(output));
    }
    // Target, raw encoder measurement, and LADRC state share one semantic MCU
    // coordinate. The fixed bridge inversion closes the negative-polarity
    // physical plant; it is identical for every channel and motor model. Keep
    // the pre-inversion value in the observer because that is the input whose
    // effect on the raw encoder coordinate is positive.
    state.applied_output_permille = static_cast<float>(channel.output_permille);
    outputs[index] = static_cast<std::int16_t>(
        kMotorBridgeOutputPolarity * channel.output_permille);
  }
  return outputs;
}

float MotorController::maximum_accepted_rps() const {
  return configuration_valid_
             ? std::min(profile_->max_rps, configuration_.maximum_accepted_rps)
             : 0.0F;
}

void MotorController::DisarmAll(bool record_watchdog_stop) {
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const bool stopped_running_motor =
        channels_[index].armed && channels_[index].target_rps != 0.0F;
    StopChannel(index, record_watchdog_stop && stopped_running_motor);
  }
}

std::uint8_t MotorController::watchdog_stop_mask() const {
  std::uint8_t mask = 0;
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    if (channels_[index].watchdog_stopped) {
      mask = static_cast<std::uint8_t>(mask | (1U << index));
    }
  }
  return mask;
}

std::uint32_t MotorController::lease_expiry_count(
    std::size_t motor_index) const {
  return motor_index < kMotorCount ? lease_expiry_count_[motor_index].value()
                                   : 0U;
}

std::uint32_t MotorController::command_rejection_count(
    std::size_t motor_index) const {
  return motor_index < kMotorCount
             ? command_rejection_count_[motor_index].value()
             : 0U;
}

std::int32_t MotorController::SignedCounterDelta(std::uint32_t current,
                                                 std::uint32_t previous,
                                                 std::uint8_t counter_bits) {
  if (counter_bits == 16U) {
    const std::uint32_t difference =
        (current - previous) & static_cast<std::uint32_t>(0xffffU);
    const std::int64_t signed_difference =
        difference <= 0x7fffU
            ? static_cast<std::int64_t>(difference)
            : static_cast<std::int64_t>(difference) - 0x10000LL;
    return static_cast<std::int32_t>(signed_difference);
  }
  const std::uint32_t difference = current - previous;
  const std::int64_t signed_difference =
      difference <= 0x7fffffffU
          ? static_cast<std::int64_t>(difference)
          : static_cast<std::int64_t>(difference) - 0x100000000LL;
  return static_cast<std::int32_t>(signed_difference);
}

void MotorController::ResetAdrc(std::size_t index) {
  adrc_state_[index] = {};
  adrc_state_[index].observed_velocity_rps = channels_[index].measured_rps;
  channels_[index].output_permille = 0;
}

void MotorController::StopChannel(std::size_t index, bool watchdog_stop) {
  MotorChannelState& channel = channels_[index];
  channel.target_rps = 0.0F;
  channel.output_permille = 0;
  channel.armed = false;
  if (watchdog_stop) {
    channel.watchdog_stopped = true;
  }
  ResetAdrc(index);
}

void MotorController::RecordRejectedCommand(std::uint8_t update_mask) {
  if (update_mask == 0U ||
      (update_mask & static_cast<std::uint8_t>(~kAllMotorMask)) != 0U) {
    return;
  }
  const std::uint8_t known_mask =
      static_cast<std::uint8_t>(update_mask & kAllMotorMask);
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    if ((known_mask & static_cast<std::uint8_t>(1U << index)) != 0U) {
      command_rejection_count_[index].Increment();
    }
  }
}

}  // namespace mentor_pi::mcu
