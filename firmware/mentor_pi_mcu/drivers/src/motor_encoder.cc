#include "mentor_pi_mcu/drivers/motor_encoder.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {
namespace {

constexpr std::int16_t kMaximumPermille = 1000;
constexpr std::array<std::uint8_t, kMotorCount> kEncoderBits{32U, 32U, 16U,
                                                             16U};

}  // namespace

void MotorEncoderDriver::InitializeSafe() {
  hardware_.EnableOutputs(false);
  for (std::size_t motor = 0; motor < kMotorCount; ++motor) {
    hardware_.WriteDrive(motor, 0U, 0U);
    previous_encoder_[motor] = hardware_.ReadEncoder(motor);
  }
  encoder_initialized_ = true;
}

void MotorEncoderDriver::Enable() {
  EmergencyStop();
  hardware_.EnableOutputs(true);
}

void MotorEncoderDriver::EmergencyStop() {
  for (std::size_t motor = 0; motor < kMotorCount; ++motor) {
    hardware_.WriteDrive(motor, 0U, 0U);
  }
}

void MotorEncoderDriver::ApplyPermille(
    const std::array<std::int16_t, kMotorCount>& output) {
  for (std::size_t motor = 0; motor < kMotorCount; ++motor) {
    const std::int32_t bounded = std::clamp<std::int32_t>(
        output[motor], -kMaximumPermille, kMaximumPermille);
    const std::uint32_t magnitude =
        static_cast<std::uint32_t>(bounded < 0 ? -bounded : bounded);
    const std::uint32_t period = hardware_.PwmPeriodTicks(motor);
    const std::uint32_t ticks = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(period) * magnitude + 500U) / 1000U);
    if (bounded > 0) {
      hardware_.WriteDrive(motor, ticks, 0U);
    } else if (bounded < 0) {
      hardware_.WriteDrive(motor, 0U, ticks);
    } else {
      hardware_.WriteDrive(motor, 0U, 0U);
    }
  }
}

std::array<std::int32_t, kMotorCount>
MotorEncoderDriver::SampleEncoderDeltas() {
  std::array<std::int32_t, kMotorCount> deltas{};
  for (std::size_t motor = 0; motor < kMotorCount; ++motor) {
    const std::uint32_t current = hardware_.ReadEncoder(motor);
    if (encoder_initialized_) {
      deltas[motor] =
          ModularDelta(current, previous_encoder_[motor], kEncoderBits[motor]);
    }
    previous_encoder_[motor] = current;
  }
  encoder_initialized_ = true;
  return deltas;
}

std::int32_t MotorEncoderDriver::ModularDelta(std::uint32_t current,
                                              std::uint32_t previous,
                                              std::uint8_t bits) {
  const std::uint64_t modulus = std::uint64_t{1} << bits;
  const std::uint32_t mask = bits == 32U ? 0xffffffffU : 0xffffU;
  const std::uint64_t delta =
      static_cast<std::uint64_t>((current - previous) & mask);
  const std::uint64_t half = modulus / 2U;
  if (delta < half) {
    return static_cast<std::int32_t>(delta);
  }
  return static_cast<std::int32_t>(-static_cast<std::int64_t>(modulus - delta));
}

}  // namespace mentor_pi::mcu::drivers
