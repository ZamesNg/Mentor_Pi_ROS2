#ifndef MENTOR_PI_MCU_DRIVERS_SSD1306_H_
#define MENTOR_PI_MCU_DRIVERS_SSD1306_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi::mcu::drivers {

constexpr std::uint8_t kSsd1306Address = 0x3cU;
constexpr std::size_t kSsd1306Width = 128U;
constexpr std::size_t kSsd1306Height = 32U;
constexpr std::size_t kSsd1306FramebufferSize =
    kSsd1306Width * kSsd1306Height / 8U;

class Ssd1306Driver {
 public:
  explicit Ssd1306Driver(RawI2c& i2c) : i2c_(i2c) {}

  Result Initialize(std::uint32_t deadline_ms);
  Result Render(const OledState& host_lines, std::uint16_t battery_mv,
                std::uint32_t deadline_ms);
  const std::array<std::uint8_t, kSsd1306FramebufferSize>& framebuffer() const {
    return framebuffer_;
  }

 private:
  void Clear();
  void DrawText(std::size_t page, const char* text, std::size_t size);
  Result Flush(std::uint32_t deadline_ms);

  RawI2c& i2c_;
  std::array<std::uint8_t, kSsd1306FramebufferSize> framebuffer_{};
  bool initialized_{false};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_SSD1306_H_
