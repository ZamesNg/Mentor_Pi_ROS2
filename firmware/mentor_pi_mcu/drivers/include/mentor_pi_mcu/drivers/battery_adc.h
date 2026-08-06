#ifndef MENTOR_PI_MCU_DRIVERS_BATTERY_ADC_H_
#define MENTOR_PI_MCU_DRIVERS_BATTERY_ADC_H_

#include <cstdint>

namespace mentor_pi::mcu::drivers {

struct BatteryAdcCalibration {
  std::uint16_t internal_reference_mv{1210U};
  std::uint16_t divider_numerator{11U};
  std::uint16_t divider_denominator{1U};
  std::uint16_t maximum_valid_mv{20000U};
};

struct BatteryAdcReading {
  std::uint16_t voltage_mv{0};
  bool valid{false};
};

// The DMA scan order is VREFINT followed by ADC1 channel 8 (PB0). This ratio
// conversion cancels ADC full-scale and VDDA error.
BatteryAdcReading ConvertBatteryAdc(std::uint16_t vrefint_raw,
                                    std::uint16_t battery_raw,
                                    const BatteryAdcCalibration& calibration);

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_BATTERY_ADC_H_
