#include "mentor_pi_mcu/domain/pwm_servo_controller.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {

PwmServoController::PwmServoController() {
  state_.target_pulse_width_us.fill(kPwmResetPulseUs);
  state_.output_pulse_width_us.fill(kPwmResetPulseUs);
  trajectory_target_us_.fill(kPwmResetPulseUs);
  logical_output_us_.fill(kPwmResetPulseUs);
  pending_target_us_.fill(kPwmResetPulseUs);
}

Result PwmServoController::AcceptCommand(const PwmServoCommand& command) {
  const Result result = ValidatePwmServoCommand(command);
  if (!result.ok()) {
    return result;
  }
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U) {
      pending_target_us_[index] = command.pulse_width_us[index];
      pending_duration_ms_[index] = command.duration_ms;
      pending_command_mask_ =
          static_cast<std::uint8_t>(pending_command_mask_ | bit);
      state_.target_pulse_width_us[index] = command.pulse_width_us[index];
    }
  }
  return OkResult();
}

Result PwmServoController::StageOffsets(const PwmServoOffsetCommand& command) {
  const Result result = ValidatePwmServoOffsets(command);
  if (!result.ok()) {
    return result;
  }
  if (pending_offset_response_mask_ != 0U) {
    return {ResultCode::kBusy, 0};
  }

  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U &&
        command.offset_us[index] != state_.offset_us[index]) {
      pending_offset_us_[index] = command.offset_us[index];
      pending_offset_change_mask_ =
          static_cast<std::uint8_t>(pending_offset_change_mask_ | bit);
    }
  }
  if (pending_offset_change_mask_ != 0U) {
    pending_offset_response_mask_ = command.update_mask;
  }
  return OkResult();
}

void PwmServoController::HoldCurrentOutputAndCancelPending(
    const std::array<std::uint16_t, kPwmServoCount>& active_output_us,
    const std::array<std::int16_t, kPwmServoCount>& active_offset_us) {
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const std::int32_t active =
        std::max(static_cast<std::int32_t>(kPwmMinimumPulseUs),
                 std::min(static_cast<std::int32_t>(kPwmMaximumPulseUs),
                          static_cast<std::int32_t>(active_output_us[index])));
    state_.offset_us[index] = active_offset_us[index];
    const std::int32_t logical = std::max(
        static_cast<std::int32_t>(kPwmMinimumPulseUs),
        std::min(static_cast<std::int32_t>(kPwmMaximumPulseUs),
                 active - static_cast<std::int32_t>(state_.offset_us[index])));
    logical_output_us_[index] = static_cast<std::uint16_t>(logical);
    state_.target_pulse_width_us[index] = logical_output_us_[index];
    state_.output_pulse_width_us[index] = static_cast<std::uint16_t>(active);
    pending_target_us_[index] = logical_output_us_[index];
    pending_duration_ms_[index] = 0U;
    pending_offset_us_[index] = state_.offset_us[index];
    trajectories_[index] = {logical_output_us_[index], 0U, 0U};
    trajectory_target_us_[index] = logical_output_us_[index];
  }
  pending_command_mask_ = 0U;
  pending_offset_change_mask_ = 0U;
  pending_offset_response_mask_ = 0U;
  state_.moving_mask = 0U;
}

PwmFrameUpdate PwmServoController::PrepareFollowingFrame() const {
  PwmFrameUpdate update{};
  update.state_ = state_;
  update.state_.target_pulse_width_us = trajectory_target_us_;
  update.logical_output_us_ = logical_output_us_;
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    Trajectory trajectory = trajectories_[index];
    if (trajectory.completed_steps < trajectory.total_steps) {
      ++trajectory.completed_steps;
      update.logical_output_us_[index] =
          Interpolate(trajectory, trajectory_target_us_[index]);
    }
    SetFrameTrajectory(&update, index, trajectory);
    update.state_.output_pulse_width_us[index] = ApplyOffset(
        update.logical_output_us_[index], update.state_.offset_us[index]);
  }
  RefreshMovingMask(&update);
  return update;
}

PwmFrameUpdate PwmServoController::PreparePendingFrame(
    const PwmFrameUpdate& submitted_frame) const {
  PwmFrameUpdate update = submitted_frame;
  update.applied_pending_command_mask_ = pending_command_mask_;
  update.applied_pending_offset_change_mask_ = pending_offset_change_mask_;
  update.offset_commit_mask_ = static_cast<std::uint8_t>(
      update.offset_commit_mask_ | pending_offset_response_mask_);

  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((pending_offset_change_mask_ & bit) != 0U) {
      update.state_.offset_us[index] = pending_offset_us_[index];
    }
    if ((pending_command_mask_ & bit) != 0U) {
      update.state_.target_pulse_width_us[index] = pending_target_us_[index];
      update.logical_output_us_[index] = logical_output_us_[index];
      Trajectory trajectory{};
      trajectory.start_pulse_us = logical_output_us_[index];
      trajectory.total_steps = static_cast<std::uint16_t>(
          (pending_duration_ms_[index] + kPwmFramePeriodMs - 1U) /
          kPwmFramePeriodMs);
      trajectory.completed_steps = 0U;
      SetFrameTrajectory(&update, index, trajectory);
    }
    update.state_.output_pulse_width_us[index] = ApplyOffset(
        update.logical_output_us_[index], update.state_.offset_us[index]);
  }
  RefreshMovingMask(&update);
  return update;
}

void PwmServoController::ConfirmPendingFrameSubmitted(
    const PwmFrameUpdate& submitted_frame) {
  pending_command_mask_ = static_cast<std::uint8_t>(
      pending_command_mask_ &
      static_cast<std::uint8_t>(
          ~submitted_frame.applied_pending_command_mask_));
  pending_offset_change_mask_ = static_cast<std::uint8_t>(
      pending_offset_change_mask_ &
      static_cast<std::uint8_t>(
          ~submitted_frame.applied_pending_offset_change_mask_));
}

void PwmServoController::CommitFrame(const PwmFrameUpdate& submitted_frame) {
  state_ = submitted_frame.state_;
  logical_output_us_ = submitted_frame.logical_output_us_;
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    trajectories_[index] = FrameTrajectory(submitted_frame, index);
    trajectory_target_us_[index] =
        submitted_frame.state_.target_pulse_width_us[index];
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((pending_command_mask_ & bit) != 0U) {
      // A newer callback may have been accepted after this hardware shadow was
      // submitted. Preserve its immediate logical target until its own B0.
      state_.target_pulse_width_us[index] = pending_target_us_[index];
    }
  }
  pending_offset_response_mask_ = static_cast<std::uint8_t>(
      pending_offset_response_mask_ &
      static_cast<std::uint8_t>(~submitted_frame.offset_commit_mask_));
}

std::uint16_t PwmServoController::ApplyOffset(std::uint16_t logical_pulse_us,
                                              std::int16_t offset_us) {
  const std::int32_t adjusted = static_cast<std::int32_t>(logical_pulse_us) +
                                static_cast<std::int32_t>(offset_us);
  return static_cast<std::uint16_t>(std::max(
      static_cast<std::int32_t>(kPwmMinimumPulseUs),
      std::min(static_cast<std::int32_t>(kPwmMaximumPulseUs), adjusted)));
}

std::uint16_t PwmServoController::Interpolate(const Trajectory& trajectory,
                                              std::uint16_t target_pulse_us) {
  if (trajectory.total_steps == 0U ||
      trajectory.completed_steps >= trajectory.total_steps) {
    return target_pulse_us;
  }
  const std::int32_t delta =
      static_cast<std::int32_t>(target_pulse_us) - trajectory.start_pulse_us;
  const std::int32_t product =
      delta * static_cast<std::int32_t>(trajectory.completed_steps);
  const std::int32_t magnitude = product < 0 ? -product : product;
  const std::int32_t rounded =
      (magnitude + static_cast<std::int32_t>(trajectory.total_steps / 2U)) /
      static_cast<std::int32_t>(trajectory.total_steps);
  const std::int32_t signed_rounded = product < 0 ? -rounded : rounded;
  return static_cast<std::uint16_t>(trajectory.start_pulse_us + signed_rounded);
}

void PwmServoController::RefreshMovingMask(PwmFrameUpdate* frame) {
  frame->state_.moving_mask = 0U;
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    if (frame->trajectory_completed_steps_[index] <
        frame->trajectory_total_steps_[index]) {
      frame->state_.moving_mask = static_cast<std::uint8_t>(
          frame->state_.moving_mask | static_cast<std::uint8_t>(1U << index));
    }
  }
}

PwmServoController::Trajectory PwmServoController::FrameTrajectory(
    const PwmFrameUpdate& frame, std::size_t index) {
  return {frame.trajectory_start_pulse_us_[index],
          frame.trajectory_total_steps_[index],
          frame.trajectory_completed_steps_[index]};
}

void PwmServoController::SetFrameTrajectory(PwmFrameUpdate* frame,
                                            std::size_t index,
                                            const Trajectory& trajectory) {
  frame->trajectory_start_pulse_us_[index] = trajectory.start_pulse_us;
  frame->trajectory_total_steps_[index] = trajectory.total_steps;
  frame->trajectory_completed_steps_[index] = trajectory.completed_steps;
}

}  // namespace mentor_pi::mcu
