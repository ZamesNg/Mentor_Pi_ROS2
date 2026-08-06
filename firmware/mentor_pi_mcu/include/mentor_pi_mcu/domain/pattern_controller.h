#ifndef MENTOR_PI_MCU_DOMAIN_PATTERN_CONTROLLER_H_
#define MENTOR_PI_MCU_DOMAIN_PATTERN_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

struct PatternOutput {
  bool on{false};
  bool complete{true};
};

class BinaryPattern {
 public:
  void Configure(std::uint16_t on_time_ms, std::uint16_t off_time_ms,
                 std::uint16_t repeat, std::uint32_t now_ms);
  PatternOutput Update(std::uint32_t now_ms);

 private:
  void AccumulateElapsed(std::uint32_t now_ms);

  std::uint16_t on_time_ms_{0};
  std::uint16_t off_time_ms_{0};
  std::uint16_t repeat_{0};
  std::uint32_t previous_update_ms_{0};
  std::uint64_t elapsed_ms_{0};
  bool initialized_{false};
};

class LedController {
 public:
  Result AcceptCommand(const LedCommand& command, std::uint32_t now_ms);
  std::array<bool, kLedCount> Update(std::uint32_t now_ms);

 private:
  std::array<BinaryPattern, kLedCount> patterns_{};
};

struct BuzzerOutput {
  std::uint16_t frequency_hz{0};
  bool battery_alarm_active{false};
};

class BuzzerController {
 public:
  Result AcceptHostCommand(const BuzzerCommand& command, std::uint32_t now_ms);
  void TriggerBatteryAlarm(std::uint32_t now_ms);
  BuzzerOutput Update(std::uint32_t now_ms);

 private:
  static constexpr BuzzerCommand kBatteryAlarmCommand{2100U, 800U, 200U, 5U};

  void StartHostPattern(std::uint32_t now_ms);

  BuzzerCommand host_command_{};
  BinaryPattern host_pattern_{};
  BinaryPattern battery_pattern_{};
  bool battery_alarm_active_{false};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_PATTERN_CONTROLLER_H_
