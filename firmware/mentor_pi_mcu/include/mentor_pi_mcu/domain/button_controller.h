#ifndef MENTOR_PI_MCU_DOMAIN_BUTTON_CONTROLLER_H_
#define MENTOR_PI_MCU_DOMAIN_BUTTON_CONTROLLER_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/fixed_containers.h"

namespace mentor_pi::mcu {

constexpr std::uint32_t kButtonScanPeriodMs = 30U;
constexpr std::uint32_t kButtonLongPressMs = 1500U;
constexpr std::uint32_t kButtonLongRepeatMs = 400U;
constexpr std::uint32_t kButtonMultiClickMs = 300U;
constexpr std::size_t kButtonCount = 2U;

enum class ButtonEventType : std::uint8_t {
  kPressed = 1,
  kLongPress = 2,
  kLongPressRepeat = 4,
  kReleaseFromLongPress = 8,
  kReleaseFromShortPress = 16,
  kClick = 32,
  kDoubleClick = 64,
  kTripleClick = 128,
};

struct ButtonEvent {
  std::uint32_t timestamp_ms{0};
  std::uint8_t button_id{0};
  ButtonEventType event{ButtonEventType::kPressed};
};

using ButtonEventQueue = DropOldestQueue<ButtonEvent, 16>;

class ButtonController {
 public:
  // raw_pressed uses logical active state; the GPIO adapter converts the
  // board's active-low input before calling this method every 30 ms.
  void Sample(const std::array<bool, kButtonCount>& raw_pressed,
              std::uint32_t now_ms);

  bool PopEvent(ButtonEvent* event) { return events_.TryPop(event); }
  std::uint32_t dropped_event_count() const { return events_.dropped_count(); }

 private:
  struct ButtonState {
    bool debounced_pressed{false};
    bool candidate_pressed{false};
    std::uint8_t candidate_samples{0};
    bool long_press_emitted{false};
    bool triple_emitted{false};
    std::uint8_t click_count{0};
    std::uint32_t pressed_at_ms{0};
    std::uint32_t next_repeat_ms{0};
    std::uint32_t last_short_release_ms{0};
  };

  void ProcessButton(std::size_t index, bool raw_pressed, std::uint32_t now_ms);
  void OnPressed(std::size_t index, std::uint32_t now_ms);
  void OnReleased(std::size_t index, std::uint32_t now_ms);
  void ProcessHeld(std::size_t index, std::uint32_t now_ms);
  void Emit(std::size_t index, ButtonEventType event, std::uint32_t now_ms);

  std::array<ButtonState, kButtonCount> buttons_{};
  ButtonEventQueue events_{};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_BUTTON_CONTROLLER_H_
