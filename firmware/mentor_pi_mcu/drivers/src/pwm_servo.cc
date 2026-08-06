#include "mentor_pi_mcu/drivers/pwm_servo.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {

Result BuildPwmServoFramePlan(
    const std::array<std::uint16_t, kPwmServoCount>& pulse_width_us,
    PwmServoFramePlan* plan) {
  if (plan == nullptr) {
    return {ResultCode::kInvalidArgument, 0};
  }
  for (std::size_t servo = 0; servo < kPwmServoCount; ++servo) {
    if (pulse_width_us[servo] < 500U || pulse_width_us[servo] > 2500U) {
      return {ResultCode::kOutOfRange, static_cast<std::uint16_t>(servo + 1U)};
    }
  }

  PwmServoFramePlan built{};
  for (std::size_t servo = 0; servo < kPwmServoCount; ++servo) {
    const std::uint16_t at_us = pulse_width_us[servo];
    const auto pin = static_cast<std::uint8_t>(1U << servo);
    std::size_t edge = 0;
    while (edge < built.edge_count && built.edges[edge].at_us < at_us) {
      ++edge;
    }
    if (edge < built.edge_count && built.edges[edge].at_us == at_us) {
      built.edges[edge].clear_mask =
          static_cast<std::uint8_t>(built.edges[edge].clear_mask | pin);
      continue;
    }
    for (std::size_t move = built.edge_count; move > edge; --move) {
      built.edges[move] = built.edges[move - 1U];
    }
    built.edges[edge] = {at_us, pin};
    ++built.edge_count;
  }
  *plan = built;
  return OkResult();
}

void PwmServoFrameDriver::BeginFrame() {
  active_ = staged_;
  next_edge_ = 0U;
  hardware_.SetPinsHigh(active_.initial_high_mask);
  if (active_.edge_count == 0U) {
    hardware_.DisableCompare();
    return;
  }
  hardware_.ArmCompareUs(active_.edges[0].at_us);
}

void PwmServoFrameDriver::HandleCompare() {
  if (next_edge_ >= active_.edge_count) {
    hardware_.DisableCompare();
    return;
  }
  hardware_.SetPinsLow(active_.edges[next_edge_].clear_mask);
  ++next_edge_;
  if (next_edge_ < active_.edge_count) {
    hardware_.ArmCompareUs(active_.edges[next_edge_].at_us);
  } else {
    hardware_.DisableCompare();
  }
}

void PwmServoFrameDriver::Stop() {
  hardware_.SetPinsLow(kAllPwmServoMask);
  hardware_.DisableCompare();
  next_edge_ = 0U;
}

}  // namespace mentor_pi::mcu::drivers
