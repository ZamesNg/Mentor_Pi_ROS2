#ifndef MENTOR_PI_MCU_DOMAIN_PWM_SERVO_CONTROLLER_H_
#define MENTOR_PI_MCU_DOMAIN_PWM_SERVO_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

constexpr std::uint16_t kPwmFramePeriodMs = 20U;
constexpr std::uint16_t kPwmMinimumPulseUs = 500U;
constexpr std::uint16_t kPwmMaximumPulseUs = 2500U;
constexpr std::uint16_t kPwmResetPulseUs = 1500U;

struct PwmServoState {
  std::array<std::uint16_t, kPwmServoCount> target_pulse_width_us{};
  std::array<std::uint16_t, kPwmServoCount> output_pulse_width_us{};
  std::array<std::int16_t, kPwmServoCount> offset_us{};
  std::uint8_t moving_mask{0};
};

class PwmServoController;

// A complete frame shadow prepared by PwmServoController.  The public state is
// the state that becomes observable if the timer ISR commits this shadow.  The
// private interpolation snapshot lets the controller advance only after that
// physical commit; merely writing a hardware shadow never advances time.
class PwmFrameUpdate {
 public:
  const PwmServoState& state() const { return state_; }
  const std::array<std::uint16_t, kPwmServoCount>& output_pulse_width_us()
      const {
    return state_.output_pulse_width_us;
  }
  std::uint8_t offset_commit_mask() const { return offset_commit_mask_; }

 private:
  friend class PwmServoController;

  PwmServoState state_{};
  std::array<std::int32_t, kPwmServoCount> trajectory_start_pulse_us_{};
  std::array<std::uint16_t, kPwmServoCount> trajectory_total_steps_{};
  std::array<std::uint16_t, kPwmServoCount> trajectory_completed_steps_{};
  std::array<std::uint16_t, kPwmServoCount> logical_output_us_{};
  std::uint8_t offset_commit_mask_{0};
  std::uint8_t applied_pending_command_mask_{0};
  std::uint8_t applied_pending_offset_change_mask_{0};
};

class PwmServoController {
 public:
  PwmServoController();

  Result AcceptCommand(const PwmServoCommand& command);

  // A non-idempotent offset update is staged and must be committed by the
  // next common frame boundary before its service returns OK.
  Result StageOffsets(const PwmServoOffsetCommand& command);
  bool offset_commit_pending() const {
    return pending_offset_response_mask_ != 0U;
  }

  // True when newly accepted work must replace the already submitted hardware
  // shadow before the next timer boundary.
  bool frame_prepare_pending() const {
    return pending_command_mask_ != 0U || pending_offset_change_mask_ != 0U;
  }

  // Freezes every channel at the supplied physical pulse currently active in
  // hardware and discards commands and offset changes that have not reached a
  // frame boundary. Active offsets remain unchanged. The peripheral owner
  // supplies its committed-shadow snapshot when ROS session authority is lost.
  void HoldCurrentOutputAndCancelPending(
      const std::array<std::uint16_t, kPwmServoCount>& active_output_us,
      const std::array<std::int16_t, kPwmServoCount>& active_offset_us);

  // Prepares the ordinary frame following the physically committed state.
  // This may contain the next interpolation step, but does not advance the
  // committed state by itself.
  PwmFrameUpdate PrepareFollowingFrame() const;

  // Applies newly accepted commands/offsets to an already submitted shadow.
  // New trajectories begin from the physically committed logical pulse, so a
  // replacement before B0 cannot accidentally start from an uncommitted B1.
  PwmFrameUpdate PreparePendingFrame(
      const PwmFrameUpdate& submitted_frame) const;

  // Called only after PreparePendingFrame's output was copied successfully to
  // the hardware shadow.  The caller serializes this with AcceptCommand and
  // StageOffsets.
  void ConfirmPendingFrameSubmitted(const PwmFrameUpdate& submitted_frame);

  // Called only after the timer ISR's monotonically increasing frame sequence
  // proves that submitted_frame became the physical active frame.
  void CommitFrame(const PwmFrameUpdate& submitted_frame);
  const PwmServoState& state() const { return state_; }

 private:
  struct Trajectory {
    std::int32_t start_pulse_us{kPwmResetPulseUs};
    std::uint16_t total_steps{0};
    std::uint16_t completed_steps{0};
  };

  static std::uint16_t ApplyOffset(std::uint16_t logical_pulse_us,
                                   std::int16_t offset_us);
  static std::uint16_t Interpolate(const Trajectory& trajectory,
                                   std::uint16_t target_pulse_us);
  static void RefreshMovingMask(PwmFrameUpdate* frame);
  static Trajectory FrameTrajectory(const PwmFrameUpdate& frame,
                                    std::size_t index);
  static void SetFrameTrajectory(PwmFrameUpdate* frame, std::size_t index,
                                 const Trajectory& trajectory);

  PwmServoState state_{};
  std::array<Trajectory, kPwmServoCount> trajectories_{};
  std::array<std::uint16_t, kPwmServoCount> trajectory_target_us_{};
  std::array<std::uint16_t, kPwmServoCount> logical_output_us_{};
  std::array<std::uint16_t, kPwmServoCount> pending_target_us_{};
  std::array<std::uint16_t, kPwmServoCount> pending_duration_ms_{};
  std::array<std::int16_t, kPwmServoCount> pending_offset_us_{};
  std::uint8_t pending_command_mask_{0};
  std::uint8_t pending_offset_change_mask_{0};
  std::uint8_t pending_offset_response_mask_{0};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_PWM_SERVO_CONTROLLER_H_
