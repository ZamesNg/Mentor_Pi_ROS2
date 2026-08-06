#include "mentor_pi_mcu/drivers/battery_adc.h"

#include <cstdint>
#include <limits>

namespace mentor_pi::mcu::drivers {

BatteryAdcReading ConvertBatteryAdc(std::uint16_t vrefint_raw,
                                    std::uint16_t battery_raw,
                                    const BatteryAdcCalibration& calibration) {
  if (vrefint_raw == 0U || vrefint_raw == 4095U ||
      calibration.internal_reference_mv == 0U ||
      calibration.divider_denominator == 0U) {
    return {};
  }
  const std::uint64_t numerator =
      static_cast<std::uint64_t>(calibration.internal_reference_mv) *
      battery_raw * calibration.divider_numerator;
  const std::uint64_t denominator =
      static_cast<std::uint64_t>(vrefint_raw) * calibration.divider_denominator;
  const std::uint64_t voltage_mv = (numerator + denominator / 2U) / denominator;
  if (voltage_mv == 0U || voltage_mv > calibration.maximum_valid_mv ||
      voltage_mv > std::numeric_limits<std::uint16_t>::max()) {
    return {};
  }
  return {static_cast<std::uint16_t>(voltage_mv), true};
}

}  // namespace mentor_pi::mcu::drivers
