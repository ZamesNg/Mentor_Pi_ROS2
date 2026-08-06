#ifndef MENTOR_PI_MCU_DRIVERS_QMI8658_H_
#define MENTOR_PI_MCU_DRIVERS_QMI8658_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi::mcu::drivers {

struct SignedAxis {
  std::uint8_t source{0};
  std::int8_t sign{1};
};

struct AxisTransform {
  std::array<SignedAxis, 3> output{{{0, 1}, {1, 1}, {2, 1}}};
  bool verified{false};
};

struct ImuSample {
  std::array<float, 3> acceleration_mps2{};
  std::array<float, 3> angular_velocity_rps{};
};

class Qmi8658Driver {
 public:
  explicit Qmi8658Driver(RegisterI2c& i2c) : i2c_(i2c) {}

  Result Initialize(std::uint32_t deadline_us);
  // Reads sensor-frame values in SI units without applying a board transform.
  // This is intended for safe axis characterization and driver-level tests.
  Result ReadRawSample(std::uint32_t deadline_us, ImuSample* sample);
  Result ReadSample(std::uint32_t deadline_us, const AxisTransform& transform,
                    ImuSample* sample);
  bool DataReady(std::uint32_t deadline_us, Result* result);

  bool initialized() const { return initialized_; }
  std::uint8_t address() const { return address_; }
  std::uint8_t revision() const { return revision_; }

 private:
  Result WriteRegister(std::uint8_t reg, std::uint8_t value,
                       std::uint32_t deadline_us);
  static bool ValidateTransform(const AxisTransform& transform);

  RegisterI2c& i2c_;
  std::uint8_t address_{0};
  std::uint8_t revision_{0};
  bool initialized_{false};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_QMI8658_H_
