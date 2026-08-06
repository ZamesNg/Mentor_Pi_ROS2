#include "mentor_pi_mcu/domain/button_controller.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu {
namespace {

bool DeadlineReached(std::uint32_t now_ms, std::uint32_t deadline_ms) {
  return static_cast<std::int32_t>(now_ms - deadline_ms) >= 0;
}

}  // namespace

void ButtonController::Sample(const std::array<bool, kButtonCount>& raw_pressed,
                              std::uint32_t now_ms) {
  for (std::size_t index = 0; index < kButtonCount; ++index) {
    ProcessButton(index, raw_pressed[index], now_ms);
  }
}

void ButtonController::ProcessButton(std::size_t index, bool raw_pressed,
                                     std::uint32_t now_ms) {
  ButtonState& state = buttons_[index];
  if (!state.debounced_pressed && state.click_count != 0U &&
      now_ms - state.last_short_release_ms > kButtonMultiClickMs) {
    state.click_count = 0;
    state.triple_emitted = false;
  }

  if (raw_pressed == state.debounced_pressed) {
    state.candidate_samples = 0;
  } else {
    if (raw_pressed != state.candidate_pressed) {
      state.candidate_pressed = raw_pressed;
      state.candidate_samples = 1;
    } else if (state.candidate_samples < 2U) {
      ++state.candidate_samples;
    }
    if (state.candidate_samples >= 2U) {
      state.debounced_pressed = raw_pressed;
      state.candidate_samples = 0;
      if (raw_pressed) {
        OnPressed(index, now_ms);
      } else {
        OnReleased(index, now_ms);
      }
    }
  }

  if (state.debounced_pressed) {
    ProcessHeld(index, now_ms);
  }
}

void ButtonController::OnPressed(std::size_t index, std::uint32_t now_ms) {
  ButtonState& state = buttons_[index];
  state.pressed_at_ms = now_ms;
  state.long_press_emitted = false;
  state.triple_emitted = false;
  Emit(index, ButtonEventType::kPressed, now_ms);

  if (state.click_count != 0U &&
      now_ms - state.last_short_release_ms <= kButtonMultiClickMs) {
    ++state.click_count;
    if (state.click_count == 2U) {
      Emit(index, ButtonEventType::kDoubleClick, now_ms);
    } else if (state.click_count == 3U) {
      Emit(index, ButtonEventType::kTripleClick, now_ms);
      state.triple_emitted = true;
    }
  } else {
    state.click_count = 0;
  }
}

void ButtonController::OnReleased(std::size_t index, std::uint32_t now_ms) {
  ButtonState& state = buttons_[index];
  if (state.long_press_emitted) {
    Emit(index, ButtonEventType::kReleaseFromLongPress, now_ms);
    state.click_count = 0;
    state.triple_emitted = false;
    return;
  }

  Emit(index, ButtonEventType::kReleaseFromShortPress, now_ms);
  Emit(index, ButtonEventType::kClick, now_ms);
  if (state.triple_emitted) {
    state.click_count = 0;
    state.triple_emitted = false;
  } else if (state.click_count == 0U) {
    state.click_count = 1;
  }
  state.last_short_release_ms = now_ms;
}

void ButtonController::ProcessHeld(std::size_t index, std::uint32_t now_ms) {
  ButtonState& state = buttons_[index];
  if (!state.long_press_emitted &&
      now_ms - state.pressed_at_ms >= kButtonLongPressMs) {
    state.long_press_emitted = true;
    state.click_count = 0;
    state.triple_emitted = false;
    state.next_repeat_ms =
        state.pressed_at_ms + kButtonLongPressMs + kButtonLongRepeatMs;
    Emit(index, ButtonEventType::kLongPress, now_ms);
    return;
  }
  if (state.long_press_emitted &&
      DeadlineReached(now_ms, state.next_repeat_ms)) {
    state.next_repeat_ms += kButtonLongRepeatMs;
    Emit(index, ButtonEventType::kLongPressRepeat, now_ms);
  }
}

void ButtonController::Emit(std::size_t index, ButtonEventType event,
                            std::uint32_t now_ms) {
  events_.PushDropOldest(
      {now_ms, static_cast<std::uint8_t>(index + 1U), event});
}

}  // namespace mentor_pi::mcu
