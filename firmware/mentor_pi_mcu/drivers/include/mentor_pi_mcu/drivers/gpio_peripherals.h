#ifndef MENTOR_PI_MCU_DRIVERS_GPIO_PERIPHERALS_H_
#define MENTOR_PI_MCU_DRIVERS_GPIO_PERIPHERALS_H_

#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu::drivers {

class PeripheralHardware {
 public:
  virtual bool ReadButtonPin(std::size_t button) const = 0;
  virtual void WriteLedPin(std::size_t led, bool high) = 0;
  virtual Result SetBuzzerTone(std::uint16_t frequency_hz, bool enabled) = 0;

 protected:
  ~PeripheralHardware() = default;
};

class GpioPeripheralDriver {
 public:
  explicit GpioPeripheralDriver(PeripheralHardware& hardware)
      : hardware_(hardware) {}

  Result InitializeSafe();
  bool ButtonPressed(std::size_t button) const;
  void SetLed(std::size_t led, bool on);
  Result SetBuzzer(std::uint16_t frequency_hz, bool on);

 private:
  PeripheralHardware& hardware_;
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_GPIO_PERIPHERALS_H_
