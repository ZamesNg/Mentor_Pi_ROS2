#ifndef MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_
#define MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace mentor_pi::hardware {

inline constexpr std::size_t kWheelCount = 4U;
inline constexpr double kTwoPi = 6.28318530717958647692;
inline constexpr char kVehicleHardwareNodeName[] = "vehicle_hardware";

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

// Positive ROS wheel rotation rolls the chassis toward +X. Firmware publishes
// raw signed encoder direction and consumes signed targets directly. This is
// the only ROS<->MCU chassis-direction map; it is ordered FL, FR, RL, RR. Since
// every entry is +/-1, the same map is its own inverse for feedback.
inline constexpr std::array<std::int8_t, kWheelCount>
    kChassisDirectionSignByWheel{{-1, 1, -1, 1}};

constexpr std::size_t WheelIndex(Wheel wheel) {
  return static_cast<std::size_t>(wheel);
}

constexpr std::size_t McuMotorIndex(Wheel wheel) {
  return kMcuMotorIndexByWheel[WheelIndex(wheel)];
}

constexpr std::uint8_t McuMotorMask(Wheel wheel) {
  return static_cast<std::uint8_t>(1U << McuMotorIndex(wheel));
}

constexpr std::int8_t ChassisDirectionSign(Wheel wheel) {
  return kChassisDirectionSignByWheel[WheelIndex(wheel)];
}

double RpsToRadiansPerSecond(double rps);
double EncoderCountToRadians(std::int64_t count,
                             std::uint32_t ticks_per_revolution);
std::optional<float> RadiansPerSecondToRps(double radians_per_second,
                                           double maximum_rps);
std::optional<double> MotorMaximumRps(std::uint8_t model);
std::optional<std::uint32_t> MotorTicksPerRevolution(std::uint8_t model);

enum class FeedbackStream : std::uint8_t {
  kMotor = 0x01U,
  kImu = 0x02U,
  kPwm = 0x04U,
};

constexpr std::uint8_t FeedbackMask(FeedbackStream stream) {
  return static_cast<std::uint8_t>(stream);
}

enum class RecoveryReason : std::uint8_t {
  kInitial,
  kFeedbackMissing,
  kFeedbackInvalid,
  kFeedbackStale,
  kHeartbeatMissing,
  kHeartbeatNotReady,
  kHeartbeatStale,
  kAuthorizationPublisherInvalid,
  kAuthorizationInvalid,
  kSessionChanged,
  kUptimeRegressed,
};

const char* RecoveryReasonName(RecoveryReason reason);

struct ReconnectStatus {
  bool ready{false};
  bool transition{false};
  RecoveryReason reason{RecoveryReason::kInitial};
  std::uint32_t session_id{0U};
  std::uint32_t uptime_ms{0U};
  std::uint32_t authorization_generation{0U};
};

// Thread-safe host-side reconnect gate. Callbacks record observations while the
// ros2_control loop evaluates whether cached controller references may reach
// the firmware. A recovery always requires observations newer than its entry
// snapshot, and an MCU/Agent restart also requires a new supervisor generation.
class ReconnectGate {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  static constexpr std::chrono::milliseconds kHeartbeatTimeout{1500};

  bool Configure(std::uint8_t required_feedback_mask,
                 std::chrono::milliseconds motor_timeout,
                 std::chrono::milliseconds imu_timeout,
                 std::chrono::milliseconds pwm_timeout);
  void Reset(TimePoint now);
  void ObserveHeartbeat(std::uint32_t session_id, std::uint32_t uptime_ms,
                        bool ready, TimePoint now);
  void ObserveAuthorization(std::uint64_t authorization, TimePoint now);
  void ObserveFeedback(FeedbackStream stream, bool valid, TimePoint now);
  ReconnectStatus Evaluate(bool authorization_publisher_valid, TimePoint now);

 private:
  struct FeedbackObservation {
    bool seen{false};
    bool valid{false};
    TimePoint received_at{};
    std::uint64_t valid_sequence{0U};
    std::chrono::milliseconds timeout{100};
  };

  static std::size_t FeedbackIndex(FeedbackStream stream);
  void ResetLocked(TimePoint now);
  void EnterRecoveryLocked(RecoveryReason reason, bool require_new_generation,
                           bool force_new_snapshot = false);
  ReconnectStatus StatusLocked(bool transition) const;

  mutable std::mutex mutex_;
  std::array<FeedbackObservation, 3U> feedback_{};
  std::array<std::uint64_t, 3U> recovery_feedback_sequence_{};
  std::uint8_t required_feedback_mask_{0U};
  std::uint64_t heartbeat_sequence_{0U};
  std::uint64_t recovery_heartbeat_sequence_{0U};
  std::uint64_t authorization_{0U};
  TimePoint last_heartbeat_{};
  std::uint32_t heartbeat_session_id_{0U};
  std::uint32_t heartbeat_uptime_ms_{0U};
  std::uint32_t last_connected_generation_{0U};
  RecoveryReason recovery_reason_{RecoveryReason::kInitial};
  bool has_heartbeat_{false};
  bool heartbeat_ready_{false};
  bool recovering_{true};
  bool recovery_cycle_observed_{false};
  bool require_new_generation_{false};
  bool transition_pending_{true};
};

class FirstOrderLowPass {
 public:
  bool Configure(double cutoff_hz);
  void Reset();
  std::optional<double> Update(double measurement, double period_seconds);

 private:
  double cutoff_hz_{5.0};
  double output_{0.0};
  bool initialized_{false};
};

class FirstOrderLadrc {
 public:
  bool Configure(double controller_bandwidth_rad_s,
                 double observer_bandwidth_rad_s);
  void Reset();
  std::optional<double> Update(double reference, double measurement,
                               double applied_control, double input_gain,
                               double period_seconds);

 private:
  double controller_bandwidth_rad_s_{1.0};
  double observer_bandwidth_rad_s_{3.0};
  double observed_output_{0.0};
  double observed_disturbance_{0.0};
  bool initialized_{false};
};

struct SteeringCalibration {
  std::size_t servo_index{2U};
  std::uint16_t minimum_pulse_us{500U};
  std::uint16_t center_pulse_us{1500U};
  std::uint16_t maximum_pulse_us{2500U};
  double minimum_angle_rad{-0.6};
  double maximum_angle_rad{0.6};
  bool inverted{true};
};

bool IsValidSteeringCalibration(const SteeringCalibration& calibration);
std::optional<std::uint16_t> SteeringAngleToPulse(
    double angle_rad, const SteeringCalibration& calibration);
std::optional<double> SteeringPulseToAngle(
    std::uint16_t pulse_us, const SteeringCalibration& calibration);

}  // namespace mentor_pi::hardware

#endif  // MENTOR_PI_HARDWARES__HARDWARE_COMMON_HPP_
