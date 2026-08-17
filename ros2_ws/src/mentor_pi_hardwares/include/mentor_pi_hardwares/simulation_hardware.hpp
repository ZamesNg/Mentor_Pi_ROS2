#ifndef MENTOR_PI_HARDWARES__SIMULATION_HARDWARE_HPP_
#define MENTOR_PI_HARDWARES__SIMULATION_HARDWARE_HPP_

#include <array>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

#include "mentor_pi_hardwares/hardware_common.hpp"

namespace mentor_pi::simulation {

struct VelocityActuator {
  double command{0.0};
  double target{0.0};
  double position{0.0};
  double velocity{0.0};
};

bool StepVelocityActuator(VelocityActuator* actuator, double period_seconds,
                          double speed_limit_rad_s,
                          double acceleration_limit_rad_s2);

class MecanumSimulationHardware final
    : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSimulationHardware)

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
  void Stop();

  std::array<std::string, hardware::kWheelCount> joint_names_{};
  std::array<VelocityActuator, hardware::kWheelCount> actuators_{};
  rclcpp::Node::SharedPtr node_;
  double speed_limit_rad_s_{37.69911184307752};
  double acceleration_limit_rad_s2_{188.4955592153876};
  bool active_{false};
};

class AckermannSimulationHardware final
    : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AckermannSimulationHardware)

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
  void Stop();

  std::array<std::string, 2U> steering_names_{};
  std::array<std::string, 2U> rear_wheel_names_{};
  std::array<double, 2U> steering_commands_{};
  std::array<VelocityActuator, 2U> rear_actuators_{};
  rclcpp::Node::SharedPtr node_;
  double steering_target_rad_{0.0};
  double steering_position_rad_{0.0};
  double speed_limit_rad_s_{37.69911184307752};
  double acceleration_limit_rad_s2_{188.4955592153876};
  double steering_minimum_rad_{-0.6};
  double steering_maximum_rad_{0.6};
  double steering_rate_limit_rad_s_{60.0};
  bool active_{false};
};

}  // namespace mentor_pi::simulation

#endif  // MENTOR_PI_HARDWARES__SIMULATION_HARDWARE_HPP_
