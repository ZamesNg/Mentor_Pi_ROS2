#ifndef MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_
#define MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace mentor_pi::hardware {

inline constexpr std::size_t kWheelCount = 4U;
inline constexpr double kTwoPi = 6.28318530717958647692;

enum class Wheel : std::size_t {
  kFrontLeft = 0U,
  kFrontRight = 1U,
  kRearLeft = 2U,
  kRearRight = 3U,
};

// The ROS-facing logical wheel order is FL, FR, RL, RR. Firmware arrays are
// connector ordered: M1=FL, M2=RL, M3=FR, M4=RR.
inline constexpr std::array<std::size_t, kWheelCount> kMcuMotorIndexByWheel{
    {0U, 2U, 1U, 3U}};

constexpr std::size_t WheelIndex(Wheel wheel) {
  return static_cast<std::size_t>(wheel);
}

constexpr std::size_t McuMotorIndex(Wheel wheel) {
  return kMcuMotorIndexByWheel[WheelIndex(wheel)];
}

constexpr std::uint8_t McuMotorMask(Wheel wheel) {
  return static_cast<std::uint8_t>(1U << McuMotorIndex(wheel));
}

double RpsToRadiansPerSecond(double rps);
double EncoderCountToRadians(std::int64_t count,
                             std::uint32_t ticks_per_revolution);
std::optional<float> RadiansPerSecondToRps(double radians_per_second,
                                           double maximum_rps);
std::optional<double> MotorMaximumRps(std::uint8_t model);
std::optional<std::uint32_t> MotorTicksPerRevolution(std::uint8_t model);

struct SteeringCalibration {
  std::size_t servo_index{2U};
  std::uint16_t minimum_pulse_us{500U};
  std::uint16_t center_pulse_us{1500U};
  std::uint16_t maximum_pulse_us{2500U};
  double minimum_angle_rad{-1.5};
  double maximum_angle_rad{1.5};
  bool inverted{true};
};

bool IsValidSteeringCalibration(const SteeringCalibration& calibration);
std::optional<std::uint16_t> SteeringAngleToPulse(
    double angle_rad, const SteeringCalibration& calibration);
std::optional<double> SteeringPulseToAngle(
    std::uint16_t pulse_us, const SteeringCalibration& calibration);

}  // namespace mentor_pi::hardware

#endif  // MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_
