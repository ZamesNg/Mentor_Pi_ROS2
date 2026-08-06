#ifndef MENTOR_PI_MCU_DRIVERS_MOTOR_ENCODER_H_
#define MENTOR_PI_MCU_DRIVERS_MOTOR_ENCODER_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"

namespace mentor_pi::mcu::drivers {

class MotorHardware {
 public:
  virtual std::uint32_t PwmPeriodTicks(std::size_t motor) const = 0;
  virtual void WriteDrive(std::size_t motor, std::uint32_t positive_ticks,
                          std::uint32_t negative_ticks) = 0;
  virtual std::uint32_t ReadEncoder(std::size_t motor) const = 0;
  virtual void EnableOutputs(bool enabled) = 0;

 protected:
  ~MotorHardware() = default;
};

// Adapts signed permille output and the mixed 32/16-bit RRCLite encoders to a
// platform implementation that owns the exact TIM channel mapping.
class MotorEncoderDriver {
 public:
  explicit MotorEncoderDriver(MotorHardware& hardware) : hardware_(hardware) {}

  void InitializeSafe();
  void Enable();
  void EmergencyStop();
  void ApplyPermille(const std::array<std::int16_t, kMotorCount>& output);
  std::array<std::int32_t, kMotorCount> SampleEncoderDeltas();

 private:
  static std::int32_t ModularDelta(std::uint32_t current,
                                   std::uint32_t previous, std::uint8_t bits);

  MotorHardware& hardware_;
  std::array<std::uint32_t, kMotorCount> previous_encoder_{};
  bool encoder_initialized_{false};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_MOTOR_ENCODER_H_
