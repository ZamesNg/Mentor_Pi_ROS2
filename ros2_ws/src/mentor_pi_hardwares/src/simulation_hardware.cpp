#include "mentor_pi_hardwares/simulation_hardware.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mentor_pi::simulation {
namespace {

constexpr std::array<const char*, hardware::kWheelCount> kMecanumJointNames{{
    "wheel_left_front_joint",
    "wheel_right_front_joint",
    "wheel_left_rear_joint",
    "wheel_right_rear_joint",
}};
constexpr std::array<const char*, 2U> kSteeringJointNames{{
    "wheel_left_front_joint",
    "wheel_right_front_joint",
}};
constexpr std::array<const char*, 2U> kRearJointNames{{
    "wheel_left_rear_joint",
    "wheel_right_rear_joint",
}};

std::string HardwareParameter(const hardware_interface::HardwareInfo& info,
                              const std::string& name,
                              const std::string& fallback) {
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ? fallback : found->second;
}

bool ParseFinite(const std::string& text, double* output) {
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(value)) {
    return false;
  }
  *output = value;
  return true;
}

bool ParsePositive(const hardware_interface::HardwareInfo& info,
                   const std::string& name, const std::string& fallback,
                   double* output) {
  return ParseFinite(HardwareParameter(info, name, fallback), output) &&
         *output > 0.0;
}

bool IsValidRobotName(const std::string& name) {
  return !name.empty() && name.front() != '/' && name.back() != '/' &&
         name.find("//") == std::string::npos;
}

bool HasOnlyInterface(
    const std::vector<hardware_interface::InterfaceInfo>& interfaces,
    const std::string& name) {
  return interfaces.size() == 1U && interfaces.front().name == name;
}

bool HasWheelStateInterfaces(
    const std::vector<hardware_interface::InterfaceInfo>& interfaces) {
  if (interfaces.size() != 2U) {
    return false;
  }
  bool has_position = false;
  bool has_velocity = false;
  for (const auto& interface : interfaces) {
    has_position |= interface.name == hardware_interface::HW_IF_POSITION;
    has_velocity |= interface.name == hardware_interface::HW_IF_VELOCITY;
  }
  return has_position && has_velocity;
}

const hardware_interface::ComponentInfo* FindJoint(
    const hardware_interface::HardwareInfo& info, const std::string& name) {
  const auto found =
      std::find_if(info.joints.begin(), info.joints.end(),
                   [&name](const auto& joint) { return joint.name == name; });
  return found == info.joints.end() ? nullptr : &*found;
}

bool ValidateVelocityJoint(const hardware_interface::ComponentInfo* joint) {
  return joint != nullptr &&
         HasOnlyInterface(joint->command_interfaces,
                          hardware_interface::HW_IF_VELOCITY) &&
         HasWheelStateInterfaces(joint->state_interfaces);
}

bool ValidateSteeringJoint(const hardware_interface::ComponentInfo* joint) {
  return joint != nullptr &&
         HasOnlyInterface(joint->command_interfaces,
                          hardware_interface::HW_IF_POSITION) &&
         HasOnlyInterface(joint->state_interfaces,
                          hardware_interface::HW_IF_POSITION);
}

}  // namespace

bool StepVelocityActuator(VelocityActuator* actuator, double period_seconds,
                          double speed_limit_rad_s,
                          double acceleration_limit_rad_s2) {
  if (actuator == nullptr || !std::isfinite(period_seconds) ||
      period_seconds < 0.0 || !std::isfinite(speed_limit_rad_s) ||
      speed_limit_rad_s <= 0.0 || !std::isfinite(acceleration_limit_rad_s2) ||
      acceleration_limit_rad_s2 <= 0.0 || !std::isfinite(actuator->target) ||
      !std::isfinite(actuator->position) ||
      !std::isfinite(actuator->velocity)) {
    return false;
  }
  const double old_velocity = actuator->velocity;
  const double target =
      std::clamp(actuator->target, -speed_limit_rad_s, speed_limit_rad_s);
  const double maximum_change = acceleration_limit_rad_s2 * period_seconds;
  actuator->velocity =
      old_velocity +
      std::clamp(target - old_velocity, -maximum_change, maximum_change);
  actuator->position +=
      0.5 * (old_velocity + actuator->velocity) * period_seconds;
  return std::isfinite(actuator->position) && std::isfinite(actuator->velocity);
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string robot_name =
      HardwareParameter(info, "robot_name", "mentor_pi_sim");
  if (!IsValidRobotName(robot_name) ||
      !ParsePositive(info, "wheel_angular_speed_limit_rad_s",
                     "37.69911184307752", &speed_limit_rad_s_) ||
      !ParsePositive(info, "wheel_angular_acceleration_limit_rad_s2",
                     "188.4955592153876", &acceleration_limit_rad_s2_) ||
      info.joints.size() != kMecanumJointNames.size()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0U; index < kMecanumJointNames.size(); ++index) {
    const auto* joint = FindJoint(info, kMecanumJointNames[index]);
    if (!ValidateVelocityJoint(joint)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_[index] = joint->name;
  }
  node_ = std::make_shared<rclcpp::Node>(hardware::kVehicleHardwareNodeName,
                                         "/" + robot_name);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  actuators_ = {};
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  actuators_ = {};
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_error(
    const rclcpp_lifecycle::State&) {
  Stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  for (auto& actuator : actuators_) {
    actuator.command = 0.0;
    actuator.target = 0.0;
    actuator.velocity = 0.0;
  }
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSimulationHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  Stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecanumSimulationHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(hardware::kWheelCount * 2U);
  for (std::size_t index = 0U; index < hardware::kWheelCount; ++index) {
    interfaces.emplace_back(joint_names_[index],
                            hardware_interface::HW_IF_POSITION,
                            &actuators_[index].position);
    interfaces.emplace_back(joint_names_[index],
                            hardware_interface::HW_IF_VELOCITY,
                            &actuators_[index].velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MecanumSimulationHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(hardware::kWheelCount);
  for (std::size_t index = 0U; index < hardware::kWheelCount; ++index) {
    interfaces.emplace_back(joint_names_[index],
                            hardware_interface::HW_IF_VELOCITY,
                            &actuators_[index].command);
  }
  return interfaces;
}

hardware_interface::return_type MecanumSimulationHardware::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const double period_seconds = period.seconds();
  for (auto& actuator : actuators_) {
    if (!StepVelocityActuator(&actuator, period_seconds, speed_limit_rad_s_,
                              acceleration_limit_rad_s2_)) {
      Stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumSimulationHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  for (const auto& actuator : actuators_) {
    if (!std::isfinite(actuator.command)) {
      Stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  for (auto& actuator : actuators_) {
    actuator.target =
        std::clamp(actuator.command, -speed_limit_rad_s_, speed_limit_rad_s_);
  }
  return hardware_interface::return_type::OK;
}

void MecanumSimulationHardware::Stop() {
  active_ = false;
  for (auto& actuator : actuators_) {
    actuator.command = 0.0;
    actuator.target = 0.0;
    actuator.velocity = 0.0;
  }
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string robot_name =
      HardwareParameter(info, "robot_name", "mentor_pi_sim");
  if (!IsValidRobotName(robot_name) ||
      !ParsePositive(info, "wheel_angular_speed_limit_rad_s",
                     "37.69911184307752", &speed_limit_rad_s_) ||
      !ParsePositive(info, "wheel_angular_acceleration_limit_rad_s2",
                     "188.4955592153876", &acceleration_limit_rad_s2_) ||
      !ParseFinite(HardwareParameter(info, "steering_angle_min_rad", "-0.6"),
                   &steering_minimum_rad_) ||
      !ParseFinite(HardwareParameter(info, "steering_angle_max_rad", "0.6"),
                   &steering_maximum_rad_) ||
      steering_minimum_rad_ >= steering_maximum_rad_ ||
      !ParsePositive(info, "steering_rate_limit_rad_s", "60.0",
                     &steering_rate_limit_rad_s_) ||
      info.joints.size() != 4U) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0U; index < kSteeringJointNames.size(); ++index) {
    const auto* joint = FindJoint(info, kSteeringJointNames[index]);
    if (!ValidateSteeringJoint(joint)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    steering_names_[index] = joint->name;
  }
  for (std::size_t index = 0U; index < kRearJointNames.size(); ++index) {
    const auto* joint = FindJoint(info, kRearJointNames[index]);
    if (!ValidateVelocityJoint(joint)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    rear_wheel_names_[index] = joint->name;
  }
  node_ = std::make_shared<rclcpp::Node>(hardware::kVehicleHardwareNodeName,
                                         "/" + robot_name);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  steering_commands_ = {};
  rear_actuators_ = {};
  steering_target_rad_ = 0.0;
  steering_position_rad_ = 0.0;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  steering_commands_ = {};
  rear_actuators_ = {};
  steering_target_rad_ = 0.0;
  steering_position_rad_ = 0.0;
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_error(
    const rclcpp_lifecycle::State&) {
  Stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  steering_commands_.fill(steering_position_rad_);
  steering_target_rad_ = steering_position_rad_;
  for (auto& actuator : rear_actuators_) {
    actuator.command = 0.0;
    actuator.target = 0.0;
    actuator.velocity = 0.0;
  }
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannSimulationHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  Stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AckermannSimulationHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(6U);
  for (const auto& name : steering_names_) {
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &steering_position_rad_);
  }
  for (std::size_t index = 0U; index < rear_wheel_names_.size(); ++index) {
    interfaces.emplace_back(rear_wheel_names_[index],
                            hardware_interface::HW_IF_POSITION,
                            &rear_actuators_[index].position);
    interfaces.emplace_back(rear_wheel_names_[index],
                            hardware_interface::HW_IF_VELOCITY,
                            &rear_actuators_[index].velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AckermannSimulationHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4U);
  for (std::size_t index = 0U; index < steering_names_.size(); ++index) {
    interfaces.emplace_back(steering_names_[index],
                            hardware_interface::HW_IF_POSITION,
                            &steering_commands_[index]);
  }
  for (std::size_t index = 0U; index < rear_wheel_names_.size(); ++index) {
    interfaces.emplace_back(rear_wheel_names_[index],
                            hardware_interface::HW_IF_VELOCITY,
                            &rear_actuators_[index].command);
  }
  return interfaces;
}

hardware_interface::return_type AckermannSimulationHardware::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds < 0.0) {
    Stop();
    return hardware_interface::return_type::ERROR;
  }
  for (auto& actuator : rear_actuators_) {
    if (!StepVelocityActuator(&actuator, period_seconds, speed_limit_rad_s_,
                              acceleration_limit_rad_s2_)) {
      Stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  const double maximum_change = steering_rate_limit_rad_s_ * period_seconds;
  steering_position_rad_ +=
      std::clamp(steering_target_rad_ - steering_position_rad_, -maximum_change,
                 maximum_change);
  if (!std::isfinite(steering_position_rad_)) {
    Stop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannSimulationHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  for (const double command : steering_commands_) {
    if (!std::isfinite(command)) {
      Stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto& actuator : rear_actuators_) {
    if (!std::isfinite(actuator.command)) {
      Stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  steering_target_rad_ =
      std::clamp(0.5 * (steering_commands_[0] + steering_commands_[1]),
                 steering_minimum_rad_, steering_maximum_rad_);
  for (auto& actuator : rear_actuators_) {
    actuator.target =
        std::clamp(actuator.command, -speed_limit_rad_s_, speed_limit_rad_s_);
  }
  return hardware_interface::return_type::OK;
}

void AckermannSimulationHardware::Stop() {
  active_ = false;
  steering_commands_.fill(steering_position_rad_);
  steering_target_rad_ = steering_position_rad_;
  for (auto& actuator : rear_actuators_) {
    actuator.command = 0.0;
    actuator.target = 0.0;
    actuator.velocity = 0.0;
  }
}

}  // namespace mentor_pi::simulation

PLUGINLIB_EXPORT_CLASS(mentor_pi::simulation::MecanumSimulationHardware,
                       hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mentor_pi::simulation::AckermannSimulationHardware,
                       hardware_interface::SystemInterface)
