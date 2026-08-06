#ifndef MENTOR_PI_MCU_DRIVERS_RGB_SPI_H_
#define MENTOR_PI_MCU_DRIVERS_RGB_SPI_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi::mcu::drivers {

constexpr std::uint8_t kRgbEncodedZero = 0xc0U;
constexpr std::uint8_t kRgbEncodedOne = 0xf8U;
constexpr std::size_t kRgbBytesPerPixel = 24U;
constexpr std::size_t kRgbResetBytes = 24U;
constexpr std::size_t kRgbEncodedSize =
    (kRgbPixelCount * kRgbBytesPerPixel) + kRgbResetBytes;

using RgbEncodedFrame = std::array<std::uint8_t, kRgbEncodedSize>;

RgbEncodedFrame EncodeRgbFrame(const RgbState& state);

class RgbSpiDriver {
 public:
  explicit RgbSpiDriver(AsyncSpi& spi) : spi_(spi) {}

  Result Begin(const RgbState& state, std::uint32_t deadline_us);
  Result Poll(std::uint32_t now_us);
  void Cancel();
  bool busy() const { return busy_; }

 private:
  AsyncSpi& spi_;
  RgbEncodedFrame frame_{};
  bool busy_{false};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_RGB_SPI_H_
