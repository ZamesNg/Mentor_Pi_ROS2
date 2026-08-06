#include "mentor_pi_mcu/domain/pattern_controller.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {

void BinaryPattern::Configure(std::uint16_t on_time_ms,
                              std::uint16_t off_time_ms, std::uint16_t repeat,
                              std::uint32_t now_ms) {
  on_time_ms_ = on_time_ms;
  off_time_ms_ = off_time_ms;
  repeat_ = repeat;
  previous_update_ms_ = now_ms;
  elapsed_ms_ = 0;
  initialized_ = true;
}

PatternOutput BinaryPattern::Update(std::uint32_t now_ms) {
  if (!initialized_ || on_time_ms_ == 0U) {
    return {false, true};
  }
  if (off_time_ms_ == 0U) {
    return {true, false};
  }
  AccumulateElapsed(now_ms);
  const std::uint64_t cycle_ms = static_cast<std::uint64_t>(on_time_ms_) +
                                 static_cast<std::uint64_t>(off_time_ms_);
  if (repeat_ != 0U && elapsed_ms_ >= cycle_ms * repeat_) {
    return {false, true};
  }
  return {(elapsed_ms_ % cycle_ms) < on_time_ms_, false};
}

void BinaryPattern::AccumulateElapsed(std::uint32_t now_ms) {
  elapsed_ms_ += now_ms - previous_update_ms_;
  previous_update_ms_ = now_ms;
}

Result LedController::AcceptCommand(const LedCommand& command,
                                    std::uint32_t now_ms) {
  const Result result = ValidateLedCommand(command);
  if (!result.ok()) {
    return result;
  }
  patterns_[static_cast<std::size_t>(command.led_id - 1U)].Configure(
      command.on_time_ms, command.off_time_ms, command.repeat, now_ms);
  return OkResult();
}

std::array<bool, kLedCount> LedController::Update(std::uint32_t now_ms) {
  std::array<bool, kLedCount> outputs{};
  for (std::size_t index = 0; index < kLedCount; ++index) {
    outputs[index] = patterns_[index].Update(now_ms).on;
  }
  return outputs;
}

Result BuzzerController::AcceptHostCommand(const BuzzerCommand& command,
                                           std::uint32_t now_ms) {
  const Result result = ValidateBuzzerCommand(command);
  if (!result.ok()) {
    return result;
  }
  host_command_ = command;
  if (!battery_alarm_active_) {
    StartHostPattern(now_ms);
  }
  return OkResult();
}

void BuzzerController::TriggerBatteryAlarm(std::uint32_t now_ms) {
  battery_alarm_active_ = true;
  battery_pattern_.Configure(kBatteryAlarmCommand.on_time_ms,
                             kBatteryAlarmCommand.off_time_ms,
                             kBatteryAlarmCommand.repeat, now_ms);
}

BuzzerOutput BuzzerController::Update(std::uint32_t now_ms) {
  if (battery_alarm_active_) {
    const PatternOutput battery = battery_pattern_.Update(now_ms);
    if (!battery.complete) {
      return {static_cast<std::uint16_t>(
                  battery.on ? kBatteryAlarmCommand.frequency_hz : 0U),
              true};
    }
    battery_alarm_active_ = false;
    StartHostPattern(now_ms);
  }

  const PatternOutput host = host_pattern_.Update(now_ms);
  const bool forced_off =
      host_command_.frequency_hz == 0U || host_command_.on_time_ms == 0U;
  return {static_cast<std::uint16_t>(
              host.on && !forced_off ? host_command_.frequency_hz : 0U),
          false};
}

void BuzzerController::StartHostPattern(std::uint32_t now_ms) {
  const bool forced_off =
      host_command_.frequency_hz == 0U || host_command_.on_time_ms == 0U;
  host_pattern_.Configure(forced_off ? 0U : host_command_.on_time_ms,
                          host_command_.off_time_ms, host_command_.repeat,
                          now_ms);
}

}  // namespace mentor_pi::mcu
