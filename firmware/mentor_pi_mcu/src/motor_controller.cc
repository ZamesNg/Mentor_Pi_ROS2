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

// Every model intentionally shares the same positional-PID starting values.
// They are safe-bounded but remain release-provisional until D3 motor HIL
// records the final gains, filter, and deadband for each physical motor
// profile.
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
  bool configuration_valid =
      configuration_.mode == MotorControlMode::kLocked ||
      configuration_.mode == MotorControlMode::kDirectionCheck ||
      configuration_.mode == MotorControlMode::kClosedLoop;
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const std::uint8_t bits = configuration_.counter_bits[index];
    if (bits != 16U && bits != 32U) {
      configuration_.counter_bits[index] = 16U;
      configuration_valid = false;
    }
    if (configuration_.channel_wiring_sign[index] != -1 &&
        configuration_.channel_wiring_sign[index] != 1) {
      configuration_.channel_wiring_sign[index] = 1;
      configuration_valid = false;
    }
  }
  if (!std::isfinite(configuration_.maximum_accepted_rps) ||
      configuration_.maximum_accepted_rps <= 0.0F ||
      configuration_.maximum_accepted_rps > 6.0F ||
      configuration_.output_limit_permille <= 0 ||
      configuration_.output_limit_permille > kMotorOutputLimitPermille ||
      (configuration_.mode == MotorControlMode::kDirectionCheck &&
       configuration_.output_limit_permille <
           kMotorDirectionCheckDutyPermille)) {
    configuration_valid = false;
  }
  if (configuration_.mode == MotorControlMode::kLocked ||
      !configuration_valid) {
    configuration_.mode = MotorControlMode::kLocked;
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
    if (!nonzero_motion_enabled()) {
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
    ResetPid(index);
    channels_[index].measured_rps = 0.0F;
    pid_overrides_[index] = {};
  }
  // Tick scale and provisional polarity are model properties. Establish a
  // fresh counter baseline on the next sample so changing either cannot turn
  // the pre-change interval into a false velocity impulse.
  encoder_initialized_ = false;
  return {OkResult(), *profile_};
}

MotorPidUpdate MotorController::SetPid(const SetMotorPidCommand& command) {
  const Result validation = ValidateSetMotorPidCommand(command);
  if (!validation.ok()) {
    return {validation, 0U};
  }
  if (configuration_.mode != MotorControlMode::kClosedLoop) {
    return {{ResultCode::kUnsupported, 0U}, 0U};
  }

  for (const MotorChannelState& channel : channels_) {
    if (channel.armed || channel.target_rps != 0.0F ||
        std::fabs(channel.measured_rps) >= kMotorPidUpdateMaximumMeasuredRps) {
      return {{ResultCode::kBusy, 0U}, 0U};
    }
  }

  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    pid_overrides_[index].active = true;
    pid_overrides_[index].gains = {command.proportional_gain[index],
                                   command.integral_gain[index],
                                   command.derivative_gain[index],
                                   command.velocity_filter_new_weight[index]};
    ResetPid(index);
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
    const std::int32_t raw_delta = SignedCounterDelta(
        raw_encoder_counters[index], previous_counter_[index],
        configuration_.counter_bits[index]);
    previous_counter_[index] = raw_encoder_counters[index];
    const std::int64_t signed_delta =
        static_cast<std::int64_t>(raw_delta) *
        configuration_.channel_wiring_sign[index] *
        ProvisionalModelEncoderPolarity(profile_->model);
    const std::int32_t normalized_delta = static_cast<std::int32_t>(std::max(
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()),
        std::min(
            static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()),
            signed_delta)));
    MotorChannelState& channel = channels_[index];
    channel.encoder_count =
        SaturatingAdd(channel.encoder_count, normalized_delta);

    const float instantaneous_rps =
        static_cast<float>(normalized_delta) /
        (static_cast<float>(profile_->ticks_per_revolution) * period_seconds);
    const PidCalibration gains = pid_overrides_[index].active
                                     ? pid_overrides_[index].gains
                                     : profile_->pid;
    const float filter_weight = gains.velocity_filter_new_weight;
    channel.measured_rps = filter_weight * instantaneous_rps +
                           (1.0F - filter_weight) * channel.measured_rps;

    if (!session_active_ || !channel.armed || channel.target_rps == 0.0F) {
      StopChannel(index, channel.watchdog_stopped);
      outputs[index] = 0;
      continue;
    }

    if (configuration_.mode == MotorControlMode::kDirectionCheck) {
      if (std::fabs(channel.measured_rps) >= kMotorDirectionCheckOverspeedRps) {
        StopChannel(index, true);
        outputs[index] = 0;
        continue;
      }
      channel.output_permille = channel.target_rps > 0.0F
                                    ? kMotorDirectionCheckDutyPermille
                                    : -kMotorDirectionCheckDutyPermille;
      outputs[index] = channel.output_permille;
      continue;
    }

    PidState& pid = pid_state_[index];
    const float error = channel.target_rps - channel.measured_rps;
    const float proportional = gains.proportional_gain * error;
    const float derivative =
        gains.derivative_gain * (error - pid.previous_error) / period_seconds;

    float trial_accumulated_error = pid.accumulated_error;
    if (gains.integral_gain > 0.0F) {
      const float trial = pid.accumulated_error + period_seconds * error;
      if (std::isfinite(trial)) {
        trial_accumulated_error = trial;
      }
    } else {
      trial_accumulated_error = 0.0F;
    }

    const float trial_integral = gains.integral_gain * trial_accumulated_error;
    float candidate = proportional + trial_integral + derivative;
    // Conditional integration prevents the integral state from growing while
    // the positional PID output is saturated and the current error would drive
    // it farther into saturation. P and D remain able to recover immediately.
    const float output_limit =
        static_cast<float>(configuration_.output_limit_permille);
    if ((candidate > output_limit && error > 0.0F) ||
        (candidate < -output_limit && error < 0.0F)) {
      candidate = proportional + gains.integral_gain * pid.accumulated_error +
                  derivative;
    } else {
      pid.accumulated_error = trial_accumulated_error;
    }
    pid.previous_error = error;
    const float output = ClampOutput(candidate, output_limit);

    const float absolute_output = std::fabs(output);
    const std::int16_t effective_deadband = std::min(
        kMotorOutputDeadbandPermille, configuration_.output_limit_permille);
    if (absolute_output > 0.0F &&
        absolute_output < static_cast<float>(effective_deadband)) {
      channel.output_permille =
          output > 0.0F ? effective_deadband
                        : static_cast<std::int16_t>(-effective_deadband);
    } else {
      channel.output_permille = static_cast<std::int16_t>(std::lround(output));
    }
    outputs[index] = channel.output_permille;
  }
  return outputs;
}

float MotorController::maximum_accepted_rps() const {
  return nonzero_motion_enabled()
             ? std::min(profile_->max_rps, configuration_.maximum_accepted_rps)
             : 0.0F;
}

std::int8_t MotorController::ProvisionalModelEncoderPolarity(MotorModel model) {
  // The legacy JGA27 profile alone used negative PID gains while every other
  // retained model used positive gains. The corrected controller has positive
  // gains for all profiles, so JGA27 provisionally inverts encoder polarity.
  // D3 HIL must confirm or replace this evidence-derived value.
  return model == MotorModel::kJga27 ? static_cast<std::int8_t>(-1)
                                     : static_cast<std::int8_t>(1);
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

void MotorController::ResetPid(std::size_t index) {
  pid_state_[index] = {};
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
  ResetPid(index);
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
