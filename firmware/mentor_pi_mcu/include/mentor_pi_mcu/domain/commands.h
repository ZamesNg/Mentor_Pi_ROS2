#ifndef MENTOR_PI_MCU_DOMAIN_COMMANDS_H_
#define MENTOR_PI_MCU_DOMAIN_COMMANDS_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu {

constexpr std::size_t kMotorCount = 4;
constexpr std::size_t kPwmServoCount = 4;
constexpr std::size_t kBusServoBatchCapacity = 16;
constexpr std::size_t kLedCount = 3;
// LED1 is the firmware system heartbeat; LED2 and LED3 are host-controlled.
// The public command keeps its wire shape.
constexpr std::uint8_t kFirstHostLedId = 2U;
constexpr std::uint8_t kLastHostLedId = 3U;
constexpr std::size_t kSystemHeartbeatLedIndex = 0U;
constexpr std::size_t kRgbPixelCount = 2;
constexpr std::size_t kOledHostLineCount = 2;
constexpr std::size_t kOledLineCapacity = 23;

constexpr std::uint8_t kAllMotorMask = 0x0fU;
constexpr float kMotorAdrcMaximumInputGain = 1000.0F;
constexpr float kMotorAdrcMaximumObserverBandwidthRadS = 50.0F;
constexpr float kMotorAdrcMaximumKnownVelocityDecayRateSInverse = 50.0F;
constexpr float kMotorAdrcMinimumFalExponent = 0.1F;
constexpr float kMotorAdrcMaximumFalExponent = 1.0F;
constexpr float kMotorAdrcMinimumFalThresholdRps = 0.001F;
constexpr float kMotorAdrcMaximumFalThresholdRps = 6.0F;
constexpr float kMotorAdrcMaximumDisturbanceLeakageSInverse = 50.0F;
constexpr std::uint16_t kMotorAdrcMaximumMinimumDrivePermille = 250U;
constexpr float kMotorAdrcHardOutputLimitPermille = 1000.0F;
constexpr std::uint8_t kAllPwmServoMask = 0x0fU;
constexpr std::uint8_t kAllRgbPixelMask = 0x03U;
// RGB1 is owned by firmware status indication. The public message keeps its
// original two-pixel wire shape, but host commands may select only RGB2.
constexpr std::uint8_t kHostRgbPixelMask = 0x02U;
constexpr std::size_t kHostRgbPixelIndex = 1U;
constexpr std::size_t kStatusRgbPixelIndex = 0U;
constexpr std::uint8_t kAllOledLineMask = 0x03U;

enum class MotorModel : std::uint8_t {
  kJgb520 = 0,
  kJgb37 = 1,
  kJga27 = 2,
  kJgb528 = 3,
};

struct MotorCommand {
  std::uint8_t update_mask{0};
  std::array<float, kMotorCount> target_rps{};
};

struct PwmServoCommand {
  std::uint8_t update_mask{0};
  std::uint16_t duration_ms{0};
  std::array<std::uint16_t, kPwmServoCount> pulse_width_us{};
};

struct PwmServoOffsetCommand {
  std::uint8_t update_mask{0};
  std::array<std::int16_t, kPwmServoCount> offset_us{};
};

struct BusServoCommand {
  std::uint8_t count{0};
  std::array<std::uint8_t, kBusServoBatchCapacity> servo_id{};
  std::array<std::uint16_t, kBusServoBatchCapacity> position{};
  std::uint16_t duration_ms{0};
};

struct SetMotorAdrcCommand {
  std::uint8_t update_mask{0};
  std::array<float, kMotorCount> known_velocity_decay_rate_s_inverse{};
  std::array<float, kMotorCount> input_gain_rps_per_second_per_permille{};
  std::array<float, kMotorCount> controller_bandwidth_rad_s{};
  std::array<float, kMotorCount> controller_fal_exponent{};
  std::array<float, kMotorCount> controller_fal_threshold_rps{};
  std::array<float, kMotorCount> observer_bandwidth_rad_s{};
  std::array<float, kMotorCount> observer_velocity_fal_exponent{};
  std::array<float, kMotorCount> observer_disturbance_fal_exponent{};
  std::array<float, kMotorCount> observer_fal_threshold_rps{};
  std::array<float, kMotorCount> disturbance_leakage_s_inverse{};
  std::array<float, kMotorCount> disturbance_estimate_limit_rps_per_second{};
  std::array<float, kMotorCount> velocity_filter_new_weight{};
  std::array<std::uint16_t, kMotorCount> positive_minimum_drive_permille{};
  std::array<std::uint16_t, kMotorCount> negative_minimum_drive_permille{};
};

struct StopBusServosCommand {
  std::uint8_t count{0};
  std::array<std::uint8_t, kBusServoBatchCapacity> servo_id{};
};

struct ConfigureBusServoCommand {
  static constexpr std::uint16_t kSetId = 1U;
  static constexpr std::uint16_t kSetOffset = 2U;
  static constexpr std::uint16_t kSaveOffset = 4U;
  static constexpr std::uint16_t kSetPositionLimits = 8U;
  static constexpr std::uint16_t kSetVoltageLimits = 16U;
  static constexpr std::uint16_t kSetTemperatureLimit = 32U;
  static constexpr std::uint16_t kSetTorque = 64U;
  static constexpr std::uint16_t kAllUpdates = 127U;

  std::uint8_t servo_id{0};
  std::uint16_t update_mask{0};
  std::uint8_t new_id{0};
  std::int8_t offset{0};
  std::uint16_t position_min{0};
  std::uint16_t position_max{0};
  std::uint16_t voltage_min_mv{0};
  std::uint16_t voltage_max_mv{0};
  std::uint8_t temperature_limit_c{0};
  bool torque_enabled{false};
};

struct GetBusServoStateCommand {
  static constexpr std::uint16_t kFieldId = 1U;
  static constexpr std::uint16_t kAllFields = 511U;

  std::uint8_t servo_id{0};
  std::uint16_t fields{0};
};

struct LedCommand {
  std::uint8_t led_id{0};
  std::uint16_t on_time_ms{0};
  std::uint16_t off_time_ms{0};
  std::uint16_t repeat{0};
};

struct BuzzerCommand {
  std::uint16_t frequency_hz{0};
  std::uint16_t on_time_ms{0};
  std::uint16_t off_time_ms{0};
  std::uint16_t repeat{0};
};

struct RgbCommand {
  std::uint8_t update_mask{0};
  std::array<std::uint8_t, kRgbPixelCount> red{};
  std::array<std::uint8_t, kRgbPixelCount> green{};
  std::array<std::uint8_t, kRgbPixelCount> blue{};
};

struct BoundedText {
  std::array<char, kOledLineCapacity + 1U> bytes{};
  std::uint8_t size{0};
};

struct OledCommand {
  std::uint8_t update_mask{0};
  std::array<BoundedText, kOledHostLineCount> lines{};
};

struct RgbState {
  std::array<std::uint8_t, kRgbPixelCount> red{};
  std::array<std::uint8_t, kRgbPixelCount> green{};
  std::array<std::uint8_t, kRgbPixelCount> blue{};
};

struct OledState {
  std::array<BoundedText, kOledHostLineCount> lines{};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_COMMANDS_H_
