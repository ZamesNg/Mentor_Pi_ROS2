#ifndef MENTOR_PI_HARDWARES__MECANUM_HARDWARE_HPP_
#define MENTOR_PI_HARDWARES__MECANUM_HARDWARE_HPP_

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
#include <mentor_pi_interfaces/msg/motor_command.hpp>
#include <mentor_pi_interfaces/msg/motor_state.hpp>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <string>
#include <thread>

namespace mentor_pi {

class MecanumHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumHardware)

  ~MecanumHardware() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
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
  void HeartbeatCallback(
      const mentor_pi_interfaces::msg::Heartbeat::SharedPtr message);
  void MotionAuthorizationCallback(
      const std_msgs::msg::UInt64::SharedPtr message);
  bool StartExecutor();
  void StopExecutor();
  bool FeedbackIsFresh(SteadyClock::time_point now) const;
  bool MotionIsAuthorized() const;
  void SendZeroMotorCommand();
  bool SendMotorCommand();
  std::size_t ParseWheelSlot(const std::string& joint_name) const;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<mentor_pi_interfaces::msg::MotorCommand>::SharedPtr
      motor_command_publisher_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::MotorState>::SharedPtr
      motor_state_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::Heartbeat>::SharedPtr
      heartbeat_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      motion_authorization_subscription_;

  std::array<std::string, hardware::kWheelCount> joint_names_{};
  std::array<bool, hardware::kWheelCount> joint_seen_{};
  std::map<std::string, Joint> joints_;

  mutable std::mutex feedback_mutex_;
  mutable std::mutex authorization_mutex_;
  std::array<double, hardware::kWheelCount> velocity_rad_s_{};
  std::array<double, hardware::kWheelCount> position_rad_{};
  double maximum_rps_{0.0};
  SteadyClock::time_point last_motor_state_{};
  SteadyClock::time_point activated_at_{};
  std::chrono::milliseconds feedback_timeout_{100};
  std::uint64_t motion_authorization_{0U};
  std::uint32_t heartbeat_session_id_{0U};
  bool heartbeat_ready_{false};
  bool has_heartbeat_{false};
  std::atomic<bool> executor_failed_{false};
  bool has_motor_state_{false};
  bool active_{false};
  std::string robot_name_{"mentor_pi"};
};

}  // namespace mentor_pi

#endif  // MENTOR_PI_HARDWARES__MECANUM_HARDWARE_HPP_
