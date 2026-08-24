#include "mentor_pi_mcu/domain/validation.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu {
namespace {

Result ValidateMask(std::uint16_t mask, std::uint16_t allowed_mask) {
  const std::uint16_t invalid_bits = static_cast<std::uint16_t>(
      mask & static_cast<std::uint16_t>(~allowed_mask));
  if (mask == 0U || invalid_bits != 0U) {
    return {ResultCode::kInvalidArgument, invalid_bits};
  }
  return OkResult();
}

bool IsBusServoId(std::uint8_t servo_id) {
  return servo_id >= 1U && servo_id <= 253U;
}

template <typename Command>
Result ValidateBusServoIdList(const Command& command) {
  if (command.count == 0U || command.count > kBusServoBatchCapacity) {
    return {ResultCode::kInvalidArgument, command.count};
  }
  for (std::size_t index = 0; index < command.count; ++index) {
    if (!IsBusServoId(command.servo_id[index])) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
    for (std::size_t prior = 0; prior < index; ++prior) {
      if (command.servo_id[prior] == command.servo_id[index]) {
        return {ResultCode::kInvalidArgument,
                static_cast<std::uint16_t>(index + 1U)};
      }
    }
  }
  return OkResult();
}

}  // namespace

Result ValidateMotorCommand(const MotorCommand& command, float max_rps) {
  const Result mask_result = ValidateMask(command.update_mask, kAllMotorMask);
  if (!mask_result.ok()) {
    return mask_result;
  }
  if (!std::isfinite(max_rps) || max_rps <= 0.0F) {
    return {ResultCode::kInvalidArgument, 0};
  }
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    const float value = command.target_rps[index];
    if (!std::isfinite(value) || std::fabs(value) > max_rps) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ValidatePwmServoCommand(const PwmServoCommand& command) {
  const Result mask_result =
      ValidateMask(command.update_mask, kAllPwmServoMask);
  if (!mask_result.ok()) {
    return mask_result;
  }
  if (command.duration_ms < 20U || command.duration_ms > 30000U) {
    return {ResultCode::kOutOfRange, 0};
  }
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        (command.pulse_width_us[index] < 500U ||
         command.pulse_width_us[index] > 2500U)) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ValidatePwmServoOffsets(const PwmServoOffsetCommand& command) {
  const Result mask_result =
      ValidateMask(command.update_mask, kAllPwmServoMask);
  if (!mask_result.ok()) {
    return mask_result;
  }
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        (command.offset_us[index] < -100 || command.offset_us[index] > 100)) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ValidateBusServoCommand(const BusServoCommand& command) {
  const Result ids_result = ValidateBusServoIdList(command);
  if (!ids_result.ok()) {
    return ids_result;
  }
  if (command.duration_ms > 30000U) {
    return {ResultCode::kOutOfRange, 0};
  }
  for (std::size_t index = 0; index < command.count; ++index) {
    if (command.position[index] > 1000U) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ValidateStopBusServosCommand(const StopBusServosCommand& command) {
  return ValidateBusServoIdList(command);
}

Result ValidateConfigureBusServoCommand(
    const ConfigureBusServoCommand& command) {
  if (!IsBusServoId(command.servo_id)) {
    return {ResultCode::kOutOfRange, 0};
  }
  const Result mask_result =
      ValidateMask(command.update_mask, ConfigureBusServoCommand::kAllUpdates);
  if (!mask_result.ok()) {
    return mask_result;
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetId) != 0U &&
      !IsBusServoId(command.new_id)) {
    return {ResultCode::kOutOfRange, 1};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetOffset) != 0U &&
      (command.offset < -125 || command.offset > 125)) {
    return {ResultCode::kOutOfRange, 2};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetPositionLimits) !=
          0U &&
      (command.position_min > 1000U || command.position_max > 1000U ||
       command.position_min > command.position_max)) {
    return {command.position_min > command.position_max
                ? ResultCode::kInvalidArgument
                : ResultCode::kOutOfRange,
            4};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetVoltageLimits) !=
          0U &&
      (command.voltage_min_mv < 4500U || command.voltage_min_mv > 14000U ||
       command.voltage_max_mv < 4500U || command.voltage_max_mv > 14000U ||
       command.voltage_min_mv > command.voltage_max_mv)) {
    return {command.voltage_min_mv > command.voltage_max_mv
                ? ResultCode::kInvalidArgument
                : ResultCode::kOutOfRange,
            5};
  }
  if ((command.update_mask & ConfigureBusServoCommand::kSetTemperatureLimit) !=
          0U &&
      command.temperature_limit_c > 100U) {
    return {ResultCode::kOutOfRange, 6};
  }
  return OkResult();
}

Result ValidateSetMotorAdrcCommand(const SetMotorAdrcCommand& command) {
  const Result mask_result = ValidateMask(command.update_mask, kAllMotorMask);
  if (!mask_result.ok()) {
    return mask_result;
  }
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    const float known_velocity_decay =
        command.known_velocity_decay_rate_s_inverse[index];
    const float input_gain =
        command.input_gain_rps_per_second_per_permille[index];
    const float controller_bandwidth =
        command.controller_bandwidth_rad_s[index];
    const float controller_fal_exponent =
        command.controller_fal_exponent[index];
    const float controller_fal_threshold =
        command.controller_fal_threshold_rps[index];
    const float observer_bandwidth = command.observer_bandwidth_rad_s[index];
    const float observer_velocity_fal_exponent =
        command.observer_velocity_fal_exponent[index];
    const float observer_disturbance_fal_exponent =
        command.observer_disturbance_fal_exponent[index];
    const float observer_fal_threshold =
        command.observer_fal_threshold_rps[index];
    const float disturbance_leakage =
        command.disturbance_leakage_s_inverse[index];
    const float disturbance_estimate_limit =
        command.disturbance_estimate_limit_rps_per_second[index];
    const float filter = command.velocity_filter_new_weight[index];
    if (!std::isfinite(known_velocity_decay) || !std::isfinite(input_gain) ||
        !std::isfinite(controller_bandwidth) ||
        !std::isfinite(controller_fal_exponent) ||
        !std::isfinite(controller_fal_threshold) ||
        !std::isfinite(observer_bandwidth) ||
        !std::isfinite(observer_velocity_fal_exponent) ||
        !std::isfinite(observer_disturbance_fal_exponent) ||
        !std::isfinite(observer_fal_threshold) ||
        !std::isfinite(disturbance_leakage) ||
        !std::isfinite(disturbance_estimate_limit) || !std::isfinite(filter)) {
      return {ResultCode::kInvalidArgument,
              static_cast<std::uint16_t>(index + 1U)};
    }
    const float maximum_disturbance_estimate =
        input_gain * kMotorAdrcHardOutputLimitPermille;
    if (known_velocity_decay < 0.0F ||
        known_velocity_decay >
            kMotorAdrcMaximumKnownVelocityDecayRateSInverse ||
        input_gain <= 0.0F || input_gain > kMotorAdrcMaximumInputGain ||
        controller_bandwidth <= 0.0F || observer_bandwidth <= 0.0F ||
        controller_bandwidth > observer_bandwidth ||
        observer_bandwidth > kMotorAdrcMaximumObserverBandwidthRadS ||
        controller_fal_exponent < kMotorAdrcMinimumFalExponent ||
        controller_fal_exponent > kMotorAdrcMaximumFalExponent ||
        controller_fal_threshold < kMotorAdrcMinimumFalThresholdRps ||
        controller_fal_threshold > kMotorAdrcMaximumFalThresholdRps ||
        observer_velocity_fal_exponent < kMotorAdrcMinimumFalExponent ||
        observer_velocity_fal_exponent > kMotorAdrcMaximumFalExponent ||
        observer_disturbance_fal_exponent < kMotorAdrcMinimumFalExponent ||
        observer_disturbance_fal_exponent > kMotorAdrcMaximumFalExponent ||
        observer_fal_threshold < kMotorAdrcMinimumFalThresholdRps ||
        observer_fal_threshold > kMotorAdrcMaximumFalThresholdRps ||
        disturbance_leakage < 0.0F ||
        disturbance_leakage > kMotorAdrcMaximumDisturbanceLeakageSInverse ||
        disturbance_estimate_limit < 0.0F ||
        disturbance_estimate_limit > maximum_disturbance_estimate ||
        filter < 0.0F || filter > 1.0F ||
        command.positive_minimum_drive_permille[index] >
            kMotorAdrcMaximumMinimumDrivePermille ||
        command.negative_minimum_drive_permille[index] >
            kMotorAdrcMaximumMinimumDrivePermille) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(index + 1U)};
    }
  }
  return OkResult();
}

Result ValidateGetBusServoStateCommand(const GetBusServoStateCommand& command) {
  const Result mask_result =
      ValidateMask(command.fields, GetBusServoStateCommand::kAllFields);
  if (!mask_result.ok()) {
    return mask_result;
  }
  if (command.servo_id == 254U) {
    return command.fields == GetBusServoStateCommand::kFieldId
               ? OkResult()
               : Result{ResultCode::kInvalidArgument, command.fields};
  }
  if (!IsBusServoId(command.servo_id)) {
    return {ResultCode::kOutOfRange, 0};
  }
  return OkResult();
}

Result ValidateLedCommand(const LedCommand& command) {
  if (command.led_id < kFirstHostLedId || command.led_id > kLastHostLedId) {
    return {ResultCode::kOutOfRange, command.led_id};
  }
  return OkResult();
}

Result ValidateBuzzerCommand(const BuzzerCommand& command) {
  if (command.frequency_hz == 0U || command.on_time_ms == 0U) {
    return OkResult();
  }
  if (command.frequency_hz < 10U || command.frequency_hz > 20000U) {
    return {ResultCode::kOutOfRange, 0};
  }
  return OkResult();
}

Result ValidateRgbCommand(const RgbCommand& command) {
  return ValidateMask(command.update_mask, kHostRgbPixelMask);
}

Result ValidateOledCommand(const OledCommand& command) {
  const Result mask_result =
      ValidateMask(command.update_mask, kAllOledLineMask);
  if (!mask_result.ok()) {
    return mask_result;
  }
  for (std::size_t line = 0; line < kOledHostLineCount; ++line) {
    const auto bit = static_cast<std::uint8_t>(1U << line);
    if ((command.update_mask & bit) == 0U) {
      continue;
    }
    const BoundedText& text = command.lines[line];
    if (text.size > kOledLineCapacity || text.bytes[text.size] != '\0') {
      return {ResultCode::kInvalidArgument,
              static_cast<std::uint16_t>(line + 1U)};
    }
    for (std::size_t index = 0; index < text.size; ++index) {
      const auto byte = static_cast<unsigned char>(text.bytes[index]);
      if (byte < 0x20U || byte > 0x7eU) {
        return {ResultCode::kInvalidArgument,
                static_cast<std::uint16_t>(line + 1U)};
      }
    }
  }
  return OkResult();
}

Result ValidateBatteryThreshold(std::uint16_t threshold_mv) {
  if (threshold_mv < 5000U || threshold_mv > 20000U) {
    return {ResultCode::kOutOfRange, 0};
  }
  return OkResult();
}

bool IsValidMotorModel(MotorModel model) {
  const auto value = static_cast<std::uint8_t>(model);
  return value <= static_cast<std::uint8_t>(MotorModel::kJgb528);
}

}  // namespace mentor_pi::mcu
