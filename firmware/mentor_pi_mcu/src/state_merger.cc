#include "mentor_pi_mcu/domain/state_merger.h"

#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi::mcu {

Result MergeRgbCommand(const RgbCommand& command, RgbState* state) {
  if (state == nullptr) {
    return {ResultCode::kInvalidArgument, 0};
  }
  const Result result = ValidateRgbCommand(command);
  if (!result.ok()) {
    return result;
  }
  RgbState merged = *state;
  for (std::size_t index = 0; index < kRgbPixelCount; ++index) {
    const auto bit = static_cast<std::uint8_t>(1U << index);
    if ((command.update_mask & bit) != 0U) {
      merged.red[index] = command.red[index];
      merged.green[index] = command.green[index];
      merged.blue[index] = command.blue[index];
    }
  }
  *state = merged;
  return OkResult();
}

Result MergeOledCommand(const OledCommand& command, OledState* state) {
  if (state == nullptr) {
    return {ResultCode::kInvalidArgument, 0};
  }
  const Result result = ValidateOledCommand(command);
  if (!result.ok()) {
    return result;
  }
  OledState merged = *state;
  for (std::size_t line = 0; line < kOledHostLineCount; ++line) {
    const auto bit = static_cast<std::uint8_t>(1U << line);
    if ((command.update_mask & bit) != 0U) {
      merged.lines[line] = command.lines[line];
    }
  }
  *state = merged;
  return OkResult();
}

}  // namespace mentor_pi::mcu
