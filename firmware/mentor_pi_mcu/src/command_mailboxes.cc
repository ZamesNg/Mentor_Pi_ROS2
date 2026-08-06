#include "mentor_pi_mcu/domain/command_mailboxes.h"

#include <cstddef>
#include <cstdint>
#include <limits>

#include "mentor_pi_mcu/domain/state_merger.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {
namespace {

std::uint32_t NextGeneration(std::uint32_t generation) {
  return generation == std::numeric_limits<std::uint32_t>::max()
             ? 1U
             : generation + 1U;
}

CommandAdmission PublishResult(bool overwrote, std::uint32_t generation,
                               SaturatingCounter<std::uint32_t>* counter) {
  if (overwrote && counter != nullptr) {
    counter->Increment();
  }
  return {OkResult(), overwrote, generation};
}

}  // namespace

CommandAdmission MotorCommandMailbox::Publish(const MotorCommand& command,
                                              float max_rps,
                                              std::uint32_t accepted_at_us) {
  const Result result = ValidateMotorCommand(command, max_rps);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.generation = NextGeneration(shadow_.generation);
  for (std::size_t index = 0; index < kMotorCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U) {
      shadow_.target_rps[index] = command.target_rps[index];
      shadow_.accepted_at_us[index] = accepted_at_us;
      shadow_.field_generation[index] = shadow_.generation;
    }
  }
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

void MotorCommandMailbox::ResetMergedFields() {
  static_cast<void>(mailbox_.DiscardLatest());
  const std::uint32_t generation = shadow_.generation;
  shadow_ = {};
  shadow_.generation = generation;
}

CommandAdmission PwmCommandMailbox::Publish(const PwmServoCommand& command) {
  const Result result = ValidatePwmServoCommand(command);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.generation = NextGeneration(shadow_.generation);
  for (std::size_t index = 0; index < kPwmServoCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U) {
      shadow_.pulse_width_us[index] = command.pulse_width_us[index];
      shadow_.duration_ms[index] = command.duration_ms;
      shadow_.field_generation[index] = shadow_.generation;
    }
  }
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

void PwmCommandMailbox::ResetMergedFields() {
  static_cast<void>(mailbox_.DiscardLatest());
  const std::uint32_t generation = shadow_.generation;
  shadow_ = {};
  shadow_.generation = generation;
}

CommandAdmission BusMotionMailbox::Publish(const BusServoCommand& command) {
  const Result result = ValidateBusServoCommand(command);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.command = command;
  shadow_.generation = NextGeneration(shadow_.generation);
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

CommandAdmission LedCommandMailbox::Publish(const LedCommand& command) {
  const Result result = ValidateLedCommand(command);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.generation = NextGeneration(shadow_.generation);
  const std::size_t index = static_cast<std::size_t>(command.led_id - 1U);
  shadow_.commands[index] = command;
  shadow_.field_generation[index] = shadow_.generation;
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

void LedCommandMailbox::ResetMergedFields() {
  static_cast<void>(mailbox_.DiscardLatest());
  const std::uint32_t generation = shadow_.generation;
  shadow_ = {};
  shadow_.generation = generation;
}

CommandAdmission BuzzerCommandMailbox::Publish(const BuzzerCommand& command) {
  const Result result = ValidateBuzzerCommand(command);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.command = command;
  shadow_.generation = NextGeneration(shadow_.generation);
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

CommandAdmission RgbCommandMailbox::Publish(const RgbCommand& command) {
  const Result result = MergeRgbCommand(command, &shadow_.state);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.generation = NextGeneration(shadow_.generation);
  for (std::size_t pixel = 0U; pixel < kRgbPixelCount; ++pixel) {
    const auto bit = static_cast<std::uint8_t>(1U << pixel);
    if ((command.update_mask & bit) != 0U) {
      shadow_.field_generation[pixel] = shadow_.generation;
    }
  }
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

void RgbCommandMailbox::ResetMergedFields() {
  static_cast<void>(mailbox_.DiscardLatest());
  const std::uint32_t generation = shadow_.generation;
  shadow_ = {};
  shadow_.generation = generation;
}

CommandAdmission OledCommandMailbox::Publish(const OledCommand& command) {
  const Result result = MergeOledCommand(command, &shadow_.state);
  if (!result.ok()) {
    return {result, false, shadow_.generation};
  }
  shadow_.generation = NextGeneration(shadow_.generation);
  for (std::size_t line = 0U; line < kOledHostLineCount; ++line) {
    const auto bit = static_cast<std::uint8_t>(1U << line);
    if ((command.update_mask & bit) != 0U) {
      shadow_.field_generation[line] = shadow_.generation;
    }
  }
  const bool overwrote = mailbox_.Publish(shadow_);
  return PublishResult(overwrote, shadow_.generation, &overwrite_count_);
}

void OledCommandMailbox::ResetMergedFields() {
  static_cast<void>(mailbox_.DiscardLatest());
  const std::uint32_t generation = shadow_.generation;
  shadow_ = {};
  shadow_.generation = generation;
}

}  // namespace mentor_pi::mcu
