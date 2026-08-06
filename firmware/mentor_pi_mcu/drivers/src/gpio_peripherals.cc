#include "mentor_pi_mcu/drivers/gpio_peripherals.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {
namespace {

constexpr std::array<bool, kLedCount> kLedActiveHigh{false, false, true};

}  // namespace

Result GpioPeripheralDriver::InitializeSafe() {
  for (std::size_t led = 0; led < kLedCount; ++led) {
    SetLed(led, false);
  }
  return hardware_.SetBuzzerTone(0U, false);
}

bool GpioPeripheralDriver::ButtonPressed(std::size_t button) const {
  // PE1 KEY1 and PE0 KEY2 are both active-low.
  return button < 2U && !hardware_.ReadButtonPin(button);
}

void GpioPeripheralDriver::SetLed(std::size_t led, bool on) {
  if (led >= kLedCount) {
    return;
  }
  hardware_.WriteLedPin(led, on == kLedActiveHigh[led]);
}

Result GpioPeripheralDriver::SetBuzzer(std::uint16_t frequency_hz, bool on) {
  return hardware_.SetBuzzerTone(frequency_hz, on && frequency_hz != 0U);
}

}  // namespace mentor_pi::mcu::drivers
