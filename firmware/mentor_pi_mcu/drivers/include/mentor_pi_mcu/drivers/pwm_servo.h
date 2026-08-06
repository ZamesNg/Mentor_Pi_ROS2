#ifndef MENTOR_PI_MCU_DRIVERS_PWM_SERVO_H_
#define MENTOR_PI_MCU_DRIVERS_PWM_SERVO_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu::drivers {

constexpr std::uint16_t kPwmServoFrameUs = 20000U;

struct PwmServoEdge {
  std::uint16_t at_us{0};
  std::uint8_t clear_mask{0};
};

struct PwmServoFramePlan {
  std::array<PwmServoEdge, kPwmServoCount> edges{};
  std::uint8_t edge_count{0};
  std::uint8_t initial_high_mask{kAllPwmServoMask};
};

class PwmServoFrameHardware {
 public:
  virtual void SetPinsHigh(std::uint8_t mask) = 0;
  virtual void SetPinsLow(std::uint8_t mask) = 0;
  virtual void ArmCompareUs(std::uint16_t frame_offset_us) = 0;
  virtual void DisableCompare() = 0;

 protected:
  ~PwmServoFrameHardware() = default;
};

Result BuildPwmServoFramePlan(
    const std::array<std::uint16_t, kPwmServoCount>& pulse_width_us,
    PwmServoFramePlan* plan);

// The TIM13 ISR calls BeginFrame and HandleCompare only. LoadPlan must run in a
// caller-provided critical section, outside the ISR.
class PwmServoFrameDriver {
 public:
  explicit PwmServoFrameDriver(PwmServoFrameHardware& hardware)
      : hardware_(hardware) {}

  void LoadPlan(const PwmServoFramePlan& plan) { staged_ = plan; }
  void BeginFrame();
  void HandleCompare();
  void Stop();

 private:
  PwmServoFrameHardware& hardware_;
  PwmServoFramePlan staged_{};
  PwmServoFramePlan active_{};
  std::uint8_t next_edge_{0};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_PWM_SERVO_H_
