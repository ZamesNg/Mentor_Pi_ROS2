#ifndef MENTOR_PI_HARDWARES__ACKERMANN_HARDWARE_HPP_
#define MENTOR_PI_HARDWARES__ACKERMANN_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <map>
#include <memory>
#include <mentor_pi_hardwares/hardware_common.hpp>
#include <mentor_pi_hardwares/joint.hpp>
#include <mentor_pi_interfaces/msg/heartbeat.hpp>
#include <mentor_pi_interfaces/msg/imu_state.hpp>
#include <mentor_pi_interfaces/msg/motor_command.hpp>
#include <mentor_pi_interfaces/msg/motor_state.hpp>
#include <mentor_pi_interfaces/msg/pwm_servo_command.hpp>
#include <mentor_pi_interfaces/msg/pwm_servo_state.hpp>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <string>
#include <thread>

namespace mentor_pi {

class AckermannHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AckermannHardware)

  ~AckermannHardware() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  using SteadyClock = std::chrono::steady_clock;

  void MotorStateCallback(
      const mentor_pi_interfaces::msg::MotorState::SharedPtr message);
  void ImuStateCallback(
      const mentor_pi_interfaces::msg::ImuState::SharedPtr message);
  void PwmServoStateCallback(
      const mentor_pi_interfaces::msg::PwmServoState::SharedPtr message);
  void HeartbeatCallback(
      const mentor_pi_interfaces::msg::Heartbeat::SharedPtr message);
  void MotionAuthorizationCallback(
      const std_msgs::msg::UInt64::SharedPtr message);
  bool StartExecutor();
  void StopExecutor();
  hardware_interface::CallbackReturn Teardown();
  bool AuthorizationPublisherIsValid() const;
  hardware::ReconnectStatus EvaluateReconnect(SteadyClock::time_point now);
  void LogReconnectTransition(const hardware::ReconnectStatus& status) const;
  void ResetChassisAdrc();
  void SendZeroCommands();
  bool SendDriveAndSteeringCommands(double period_seconds);
  std::size_t ParseWheelSlot(const std::string& joint_name) const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<mentor_pi_interfaces::msg::MotorCommand>::SharedPtr
      motor_command_publisher_;
  rclcpp::Publisher<mentor_pi_interfaces::msg::PwmServoCommand>::SharedPtr
      pwm_command_publisher_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::MotorState>::SharedPtr
      motor_state_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::ImuState>::SharedPtr
      imu_state_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::PwmServoState>::SharedPtr
      pwm_state_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::Heartbeat>::SharedPtr
      heartbeat_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      motion_authorization_subscription_;

  std::array<std::string, hardware::kWheelCount> wheel_names_{};
  std::array<bool, hardware::kWheelCount> wheel_seen_{};
  std::array<std::string, 2U> steering_names_{};
  std::array<bool, 2U> steering_seen_{};
  std::map<std::string, Joint> joints_;

  mutable std::mutex feedback_mutex_;
  std::array<double, hardware::kWheelCount> velocity_rad_s_{};
  std::array<double, hardware::kWheelCount> position_rad_{};
  double steering_position_rad_{0.0};
  double yaw_rate_rad_s_{0.0};
  double maximum_rps_{0.0};
  std::chrono::milliseconds feedback_timeout_{100};
  std::chrono::milliseconds imu_timeout_{100};
  hardware::ReconnectGate reconnect_gate_{};
  hardware::SteeringCalibration steering_calibration_{};
  hardware::FirstOrderLadrc linear_adrc_{};
  hardware::FirstOrderLadrc yaw_adrc_{};
  hardware::FirstOrderLowPass linear_measurement_lpf_{};
  hardware::FirstOrderLowPass yaw_measurement_lpf_{};
  double applied_linear_correction_m_s_{0.0};
  double applied_steering_correction_rad_{0.0};
  double rear_wheel_radius_m_{0.0325};
  double wheelbase_m_{0.135};
  double linear_adrc_input_gain_per_second_{5.0};
  double yaw_adrc_input_gain_per_mps_{30.0};
  double yaw_adrc_minimum_speed_mps_{0.1};
  std::uint16_t steering_duration_ms_{20U};
  std::atomic<bool> executor_failed_{false};
  std::atomic<bool> executor_stop_requested_{false};
  bool active_{false};
  std::string robot_name_{"mentor_pi"};
};

}  // namespace mentor_pi

#endif  // MENTOR_PI_HARDWARES__ACKERMANN_HARDWARE_HPP_
