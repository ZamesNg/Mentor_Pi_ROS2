#ifndef MENTOR_PI_MCU_DOMAIN_MOTOR_CONTROLLER_H_
#define MENTOR_PI_MCU_DOMAIN_MOTOR_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

constexpr std::uint32_t kMotorLeaseExpiryUs = 198000U;
constexpr std::uint32_t kMotorControlPeriodUs = 10000U;
constexpr std::int16_t kMotorOutputLimitPermille = 1000;
constexpr std::int16_t kMotorOutputDeadbandPermille = 250;
constexpr float kMotorCommissioningMaximumRps = 0.25F;
constexpr float kMotorCommissioningClosedLoopMaximumRps = 6.0F;
constexpr std::int16_t kMotorCommissioningOutputLimitPermille = 1000;
constexpr std::int16_t kMotorDirectionCheckDutyPermille = 250;
constexpr float kMotorDirectionCheckOverspeedRps = 0.5F;
constexpr float kMotorPidUpdateMaximumMeasuredRps = 0.01F;
constexpr float kMotorDefaultPidProportionalGain = 250.0F;
constexpr float kMotorDefaultPidIntegralGain = 0.1F;
constexpr float kMotorDefaultPidDerivativeGain = 0.5F;
constexpr float kMotorDefaultVelocityFilterNewWeight = 0.5F;

struct PidCalibration {
  float proportional_gain{kMotorDefaultPidProportionalGain};
  float integral_gain{kMotorDefaultPidIntegralGain};
  float derivative_gain{kMotorDefaultPidDerivativeGain};
  float velocity_filter_new_weight{kMotorDefaultVelocityFilterNewWeight};
};

struct MotorProfile {
  MotorModel model{MotorModel::kJga27};
  std::uint32_t ticks_per_revolution{1040};
  float max_rps{6.0F};
  PidCalibration pid{};
};

const MotorProfile& GetMotorProfile(MotorModel model);

struct MotorChannelState {
  float target_rps{0.0F};
  float measured_rps{0.0F};
  std::int64_t encoder_count{0};
  std::int16_t output_permille{0};
  bool armed{false};
  bool watchdog_stopped{false};
};

struct MotorModelChange {
  Result result{};
  MotorProfile active_profile{};
};

struct MotorPidUpdate {
  Result result{};
  std::uint8_t applied_mask{0U};
};

enum class MotorControlMode : std::uint8_t {
  kLocked = 0,
  kDirectionCheck,
  kClosedLoop,
};

struct MotorControlConfiguration {
  // RRCLite: M1/TIM5 and M2/TIM2 are 32-bit; M3/TIM4 and M4/TIM3
  // are 16-bit. A hardware adapter supplies channel wiring signs established
  // by HIL. The controller multiplies them by the provisional per-model
  // polarity derived from the legacy controller evidence.
  std::array<std::uint8_t, kMotorCount> counter_bits{32, 32, 16, 16};
  std::array<std::int8_t, kMotorCount> channel_wiring_sign{1, 1, 1, 1};
  MotorControlMode mode{MotorControlMode::kLocked};
  float maximum_accepted_rps{0.0F};
  std::int16_t output_limit_permille{0};
};

// These are the only target build profiles before motor HIL qualification.
// Selecting between them remains a compile-time decision in target/stm32;
// neither ROS nor another runtime input can construct new motor authority.
constexpr MotorControlConfiguration LockedMotorControlConfiguration() {
  return {};
}

constexpr MotorControlConfiguration CommissioningMotorControlConfiguration() {
  MotorControlConfiguration configuration{};
  configuration.mode = MotorControlMode::kDirectionCheck;
  configuration.maximum_accepted_rps = kMotorCommissioningMaximumRps;
  configuration.output_limit_permille = kMotorCommissioningOutputLimitPermille;
  return configuration;
}

static_assert(LockedMotorControlConfiguration().mode ==
              MotorControlMode::kLocked);
static_assert(LockedMotorControlConfiguration().maximum_accepted_rps == 0.0F);
static_assert(LockedMotorControlConfiguration().output_limit_permille == 0);
static_assert(CommissioningMotorControlConfiguration().mode ==
              MotorControlMode::kDirectionCheck);
static_assert(CommissioningMotorControlConfiguration().maximum_accepted_rps ==
              0.25F);
static_assert(CommissioningMotorControlConfiguration().output_limit_permille ==
              kMotorCommissioningOutputLimitPermille);
static_assert(kMotorCommissioningOutputLimitPermille <=
              kMotorOutputLimitPermille);
static_assert(kMotorDirectionCheckDutyPermille <=
              kMotorCommissioningOutputLimitPermille);

class MotorController {
 public:
  explicit MotorController(MotorControlConfiguration configuration = {});

  void SetSessionActive(bool active);
  bool session_active() const { return session_active_; }

  Result AcceptCommand(const MotorCommand& command, std::uint32_t now_us);
  MotorModelChange SetModel(MotorModel model);
  MotorPidUpdate SetPid(const SetMotorPidCommand& command);

  // Called by the independent 1 kHz safety release.
  void EvaluateLeases(std::uint32_t now_us);

  // Called every 10 ms. Raw counters are sampled together by the hardware
  // owner; modular deltas and direction normalization happen here.
  std::array<std::int16_t, kMotorCount> ControlStep(
      const std::array<std::uint32_t, kMotorCount>& raw_encoder_counters,
      std::uint32_t period_us = kMotorControlPeriodUs);

  // Immediate software-side part of transport/supervisor emergency stop.
  // The hardware adapter must independently zero and disable PWM registers.
  void DisarmAll(bool record_watchdog_stop);

  const std::array<MotorChannelState, kMotorCount>& channels() const {
    return channels_;
  }
  const MotorProfile& profile() const { return *profile_; }
  const MotorControlConfiguration& configuration() const {
    return configuration_;
  }
  bool nonzero_motion_enabled() const {
    return configuration_.mode != MotorControlMode::kLocked;
  }
  float maximum_accepted_rps() const;
  std::uint8_t watchdog_stop_mask() const;
  std::uint32_t lease_expiry_count(std::size_t motor_index) const;
  std::uint32_t command_rejection_count(std::size_t motor_index) const;
  // Records one atomically rejected command for every selected motor when the
  // mask itself is valid. A zero or malformed mask has no valid selection and
  // is accounted only by the subscription-level rejection counter. The
  // command gateway calls this only when it rejects before mailbox
  // consumption; AcceptCommand records its own downstream rejections.
  void RecordRejectedCommand(std::uint8_t update_mask);

  static std::int32_t SignedCounterDelta(std::uint32_t current,
                                         std::uint32_t previous,
                                         std::uint8_t counter_bits);
  static std::int8_t ProvisionalModelEncoderPolarity(MotorModel model);

 private:
  struct PidState {
    float accumulated_error{0.0F};
    float previous_error{0.0F};
  };

  struct MotorPidOverride {
    bool active{false};
    PidCalibration gains{};
  };

  void ResetPid(std::size_t index);
  void StopChannel(std::size_t index, bool watchdog_stop);

  MotorControlConfiguration configuration_{};
  const MotorProfile* profile_;
  std::array<MotorChannelState, kMotorCount> channels_{};
  std::array<PidState, kMotorCount> pid_state_{};
  std::array<MotorPidOverride, kMotorCount> pid_overrides_{};
  std::array<std::uint32_t, kMotorCount> last_command_us_{};
  std::array<std::uint32_t, kMotorCount> previous_counter_{};
  std::array<SaturatingCounter<std::uint32_t>, kMotorCount>
      lease_expiry_count_{};
  std::array<SaturatingCounter<std::uint32_t>, kMotorCount>
      command_rejection_count_{};
  bool session_active_{false};
  bool encoder_initialized_{false};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_MOTOR_CONTROLLER_H_
