#ifndef MENTOR_PI_MCU_DOMAIN_BATTERY_MONITOR_H_
#define MENTOR_PI_MCU_DOMAIN_BATTERY_MONITOR_H_

#include <cstdint>

#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

constexpr std::uint16_t kDefaultBatteryThresholdMv = 6300U;
constexpr std::uint32_t kBatteryLowAssertionMs = 10000U;
constexpr std::uint32_t kBatteryClearMs = 2000U;
constexpr std::uint16_t kBatteryClearHysteresisMv = 200U;
constexpr std::uint32_t kBatteryAlarmRepeatMs = 10000U;
constexpr float kBatteryFilterCoefficient = 0.05F;

struct BatteryState {
  std::uint16_t voltage_mv{0};
  std::uint16_t low_threshold_mv{kDefaultBatteryThresholdMv};
  bool valid{false};
  bool below_threshold{false};
};

struct BatteryUpdate {
  BatteryState state{};
  bool request_alarm_pattern{false};
};

struct BatteryThresholdUpdate {
  Result result{};
  std::uint16_t active_threshold_mv{kDefaultBatteryThresholdMv};
};

class BatteryMonitor {
 public:
  BatteryUpdate AddSample(std::uint32_t raw_voltage_mv,
                          bool internal_reference_valid, std::uint32_t now_ms);
  BatteryThresholdUpdate SetLowThreshold(std::uint16_t threshold_mv);

  const BatteryState& state() const { return state_; }

 private:
  bool UpdateLowThresholdDebounce(std::uint32_t valid_elapsed_ms,
                                  std::uint32_t now_ms);
  void UpdateClearThresholdDebounce(std::uint32_t valid_elapsed_ms);
  void ResetDebounce();

  BatteryState state_{};
  float filtered_voltage_mv_{0.0F};
  std::uint32_t low_elapsed_ms_{0};
  std::uint32_t clear_elapsed_ms_{0};
  std::uint32_t previous_sample_ms_{0};
  std::uint32_t last_alarm_ms_{0};
  bool filter_initialized_{false};
  bool previous_sample_valid_{false};
  bool low_tracking_{false};
  bool clear_tracking_{false};
  bool alarm_has_fired_{false};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_BATTERY_MONITOR_H_
