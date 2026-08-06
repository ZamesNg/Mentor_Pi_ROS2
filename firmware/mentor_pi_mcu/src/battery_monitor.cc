#include "mentor_pi_mcu/domain/battery_monitor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

std::uint32_t SaturatingAdd(std::uint32_t value, std::uint32_t increment) {
  return increment > std::numeric_limits<std::uint32_t>::max() - value
             ? std::numeric_limits<std::uint32_t>::max()
             : value + increment;
}

}  // namespace

BatteryUpdate BatteryMonitor::AddSample(std::uint32_t raw_voltage_mv,
                                        bool internal_reference_valid,
                                        std::uint32_t now_ms) {
  BatteryUpdate update{};
  if (state_.below_threshold && alarm_has_fired_ &&
      now_ms - last_alarm_ms_ >= kBatteryAlarmRepeatMs) {
    update.request_alarm_pattern = true;
    last_alarm_ms_ = now_ms;
  }

  const bool sample_valid = internal_reference_valid && raw_voltage_mv > 0U &&
                            raw_voltage_mv <= 20000U;
  const std::uint32_t valid_elapsed_ms = sample_valid && previous_sample_valid_
                                             ? now_ms - previous_sample_ms_
                                             : 0U;
  previous_sample_ms_ = now_ms;
  previous_sample_valid_ = sample_valid;

  if (!sample_valid) {
    state_.valid = false;
    state_.voltage_mv = 0;
    update.state = state_;
    return update;
  }

  if (!filter_initialized_) {
    filtered_voltage_mv_ = static_cast<float>(raw_voltage_mv);
    filter_initialized_ = true;
  } else {
    filtered_voltage_mv_ +=
        kBatteryFilterCoefficient *
        (static_cast<float>(raw_voltage_mv) - filtered_voltage_mv_);
  }
  state_.valid = true;
  state_.voltage_mv = static_cast<std::uint16_t>(
      std::lround(std::max(0.0F, std::min(20000.0F, filtered_voltage_mv_))));

  if (!state_.below_threshold) {
    update.request_alarm_pattern |=
        UpdateLowThresholdDebounce(valid_elapsed_ms, now_ms);
  } else {
    UpdateClearThresholdDebounce(valid_elapsed_ms);
  }

  update.state = state_;
  return update;
}

bool BatteryMonitor::UpdateLowThresholdDebounce(std::uint32_t valid_elapsed_ms,
                                                std::uint32_t now_ms) {
  clear_tracking_ = false;
  clear_elapsed_ms_ = 0;
  if (state_.voltage_mv >= state_.low_threshold_mv) {
    low_tracking_ = false;
    low_elapsed_ms_ = 0;
    return false;
  }

  if (!low_tracking_) {
    low_tracking_ = true;
    low_elapsed_ms_ = 0;
  } else {
    low_elapsed_ms_ = SaturatingAdd(low_elapsed_ms_, valid_elapsed_ms);
  }
  if (low_elapsed_ms_ < kBatteryLowAssertionMs) {
    return false;
  }

  state_.below_threshold = true;
  low_tracking_ = false;
  low_elapsed_ms_ = 0;
  alarm_has_fired_ = true;
  last_alarm_ms_ = now_ms;
  return true;
}

void BatteryMonitor::UpdateClearThresholdDebounce(
    std::uint32_t valid_elapsed_ms) {
  low_tracking_ = false;
  low_elapsed_ms_ = 0;
  const std::uint32_t clear_threshold_mv = std::min<std::uint32_t>(
      20000U, static_cast<std::uint32_t>(state_.low_threshold_mv) +
                  static_cast<std::uint32_t>(kBatteryClearHysteresisMv));
  if (state_.voltage_mv < clear_threshold_mv) {
    clear_tracking_ = false;
    clear_elapsed_ms_ = 0;
    return;
  }

  if (!clear_tracking_) {
    clear_tracking_ = true;
    clear_elapsed_ms_ = 0;
  } else {
    clear_elapsed_ms_ = SaturatingAdd(clear_elapsed_ms_, valid_elapsed_ms);
  }
  if (clear_elapsed_ms_ >= kBatteryClearMs) {
    state_.below_threshold = false;
    clear_tracking_ = false;
    clear_elapsed_ms_ = 0;
    alarm_has_fired_ = false;
  }
}

BatteryThresholdUpdate BatteryMonitor::SetLowThreshold(
    std::uint16_t threshold_mv) {
  const Result result = ValidateBatteryThreshold(threshold_mv);
  if (!result.ok()) {
    return {result, state_.low_threshold_mv};
  }
  if (threshold_mv != state_.low_threshold_mv) {
    state_.low_threshold_mv = threshold_mv;
    ResetDebounce();
  }
  return {OkResult(), state_.low_threshold_mv};
}

void BatteryMonitor::ResetDebounce() {
  low_elapsed_ms_ = 0;
  clear_elapsed_ms_ = 0;
  low_tracking_ = false;
  clear_tracking_ = false;
}

}  // namespace mentor_pi::mcu
