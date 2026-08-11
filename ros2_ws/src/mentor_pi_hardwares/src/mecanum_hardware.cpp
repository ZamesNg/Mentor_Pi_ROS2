#include "mentor_pi_hardwares/mecanum_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <functional>
#include <pluginlib/class_list_macros.hpp>
#include <system_error>

namespace mentor_pi {
namespace {

constexpr char kMotionAuthorizationTopic[] =
    "/mentor_pi/configuration/motion_authorization";
constexpr char kConfigurationSupervisorName[] = "configuration_supervisor";
constexpr char kConfigurationSupervisorNamespace[] = "/mentor_pi";

std::string ToLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

bool HasToken(const std::string& source, const std::string& token) {
  return source.find(token) != std::string::npos;
}

std::string HardwareParameter(const hardware_interface::HardwareInfo& info,
                              const std::string& name,
                              const std::string& fallback) {
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ? fallback : found->second;
}

bool ParsePositiveMilliseconds(const std::string& text,
                               std::chrono::milliseconds* output) {
  std::uint32_t value = 0U;
  const auto conversion =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (conversion.ec != std::errc{} ||
      conversion.ptr != text.data() + text.size() || value == 0U ||
      value > 10000U) {
    return false;
  }
  *output = std::chrono::milliseconds(value);
  return true;
}

bool IsValidRobotName(const std::string& name) {
  return !name.empty() && name.front() != '/' && name.back() != '/' &&
         name.find("//") == std::string::npos;
}

}  // namespace

MecanumHardware::~MecanumHardware() { StopExecutor(); }

hardware_interface::CallbackReturn MecanumHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_name_ = HardwareParameter(info, "robot_name", "mentor_pi");
  if (!IsValidRobotName(robot_name_) ||
      !ParsePositiveMilliseconds(
          HardwareParameter(info, "feedback_timeout_ms", "100"),
          &feedback_timeout_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  node_ = std::make_shared<rclcpp::Node>("mecanum_hardware", "/" + robot_name_);

  if (info.joints.size() != hardware::kWheelCount) {
    RCLCPP_FATAL(node_->get_logger(), "Expected four mecanum wheel joints");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto& joint : info.joints) {
    const std::size_t slot = ParseWheelSlot(joint.name);
    if (slot >= hardware::kWheelCount || joint_seen_[slot]) {
      RCLCPP_FATAL(node_->get_logger(), "Invalid or duplicate wheel joint: %s",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto has_interface = [](const auto& interfaces,
                                  const std::string& name) {
      return std::any_of(
          interfaces.begin(), interfaces.end(),
          [&name](const auto& interface) { return interface.name == name; });
    };
    if (joint.command_interfaces.size() != 1U ||
        !has_interface(joint.command_interfaces,
                       hardware_interface::HW_IF_VELOCITY) ||
        joint.state_interfaces.size() != 2U ||
        !has_interface(joint.state_interfaces,
                       hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint.state_interfaces,
                       hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL(
          node_->get_logger(),
          "Wheel %s requires velocity command and position/velocity state",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_seen_[slot] = true;
    joint_names_[slot] = joint.name;
    joints_.emplace(joint.name, Joint(joint.name));
  }
  if (!std::all_of(joint_seen_.begin(), joint_seen_.end(),
                   [](bool seen) { return seen; })) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!node_ || executor_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  motor_command_publisher_ =
      node_->create_publisher<mentor_pi_interfaces::msg::MotorCommand>(
          "/mentor_pi/motors/command", qos);
  motor_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::MotorState>(
          "/mentor_pi/motors/state", qos,
          std::bind(&MecanumHardware::MotorStateCallback, this,
                    std::placeholders::_1));
  heartbeat_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          std::bind(&MecanumHardware::HeartbeatCallback, this,
                    std::placeholders::_1));
  motion_authorization_subscription_ =
      node_->create_subscription<std_msgs::msg::UInt64>(
          "/mentor_pi/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&MecanumHardware::MotionAuthorizationCallback, this,
                    std::placeholders::_1));
  for (auto& entry : joints_) {
    entry.second.state = {};
    entry.second.command = {};
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    velocity_rad_s_.fill(0.0);
    position_rad_.fill(0.0);
    maximum_rps_ = 0.0;
    has_motor_state_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(authorization_mutex_);
    motion_authorization_ = 0U;
    heartbeat_session_id_ = 0U;
    heartbeat_ready_ = false;
    has_heartbeat_ = false;
  }
  executor_failed_.store(false, std::memory_order_release);
  if (!StartExecutor()) {
    SendZeroMotorCommand();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  SendZeroMotorCommand();
  StopExecutor();
  motion_authorization_subscription_.reset();
  heartbeat_subscription_.reset();
  motor_state_subscription_.reset();
  motor_command_publisher_.reset();
  {
    std::lock_guard<std::mutex> lock(authorization_mutex_);
    motion_authorization_ = 0U;
    heartbeat_session_id_ = 0U;
    heartbeat_ready_ = false;
    has_heartbeat_ = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  if (executor_failed_.load(std::memory_order_acquire)) {
    SendZeroMotorCommand();
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto& entry : joints_) {
    entry.second.command.velocity = 0.0;
  }
  activated_at_ = SteadyClock::now();
  active_ = true;
  SendZeroMotorCommand();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  SendZeroMotorCommand();
  for (auto& entry : joints_) {
    entry.second.command.velocity = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecanumHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(hardware::kWheelCount * 2U);
  for (const auto& name : joint_names_) {
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &joints_.at(name).state.position);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &joints_.at(name).state.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MecanumHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(hardware::kWheelCount);
  for (const auto& name : joint_names_) {
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &joints_.at(name).command.velocity);
  }
  return interfaces;
}

hardware_interface::return_type MecanumHardware::read(const rclcpp::Time&,
                                                      const rclcpp::Duration&) {
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    fresh = FeedbackIsFresh(SteadyClock::now());
    velocity = velocity_rad_s_;
    position = position_rad_;
  }
  if (active_ && !fresh) {
    SendZeroMotorCommand();
    return MotionIsAuthorized() ? hardware_interface::return_type::ERROR
                                : hardware_interface::return_type::OK;
  }
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    auto& joint = joints_.at(joint_names_[wheel]);
    joint.state.velocity = velocity[wheel];
    joint.state.position = position[wheel];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (executor_failed_.load(std::memory_order_acquire)) {
    SendZeroMotorCommand();
    return hardware_interface::return_type::ERROR;
  }
  if (!MotionIsAuthorized()) {
    SendZeroMotorCommand();
    return hardware_interface::return_type::OK;
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!FeedbackIsFresh(SteadyClock::now())) {
      SendZeroMotorCommand();
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!SendMotorCommand()) {
    SendZeroMotorCommand();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void MecanumHardware::MotorStateCallback(
    const mentor_pi_interfaces::msg::MotorState::SharedPtr message) {
  const auto maximum_rps = hardware::MotorMaximumRps(message->motor_model);
  const auto ticks = hardware::MotorTicksPerRevolution(message->motor_model);
  if (!maximum_rps || !ticks) {
    return;
  }
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    const auto logical = static_cast<hardware::Wheel>(wheel);
    const std::size_t motor = hardware::McuMotorIndex(logical);
    velocity[wheel] = hardware::RpsToRadiansPerSecond(
        static_cast<double>(message->measured_rps[motor]));
    position[wheel] =
        hardware::EncoderCountToRadians(message->encoder_count[motor], *ticks);
    if (!std::isfinite(velocity[wheel]) || !std::isfinite(position[wheel])) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  velocity_rad_s_ = velocity;
  position_rad_ = position;
  maximum_rps_ = *maximum_rps;
  last_motor_state_ = SteadyClock::now();
  has_motor_state_ = true;
}

void MecanumHardware::HeartbeatCallback(
    const mentor_pi_interfaces::msg::Heartbeat::SharedPtr message) {
  const bool ready =
      message->agent_session_id != 0U &&
      (message->state == mentor_pi_interfaces::msg::Heartbeat::READY ||
       message->state == mentor_pi_interfaces::msg::Heartbeat::DEGRADED);
  std::lock_guard<std::mutex> lock(authorization_mutex_);
  heartbeat_session_id_ = message->agent_session_id;
  heartbeat_ready_ = ready;
  has_heartbeat_ = true;
}

void MecanumHardware::MotionAuthorizationCallback(
    const std_msgs::msg::UInt64::SharedPtr message) {
  std::lock_guard<std::mutex> lock(authorization_mutex_);
  motion_authorization_ = message->data;
}

bool MecanumHardware::StartExecutor() {
  if (!node_ || executor_) {
    return false;
  }
  try {
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (...) {
        executor_failed_.store(true, std::memory_order_release);
        SendZeroMotorCommand();
      }
    });
  } catch (...) {
    if (executor_ && node_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    executor_failed_.store(true, std::memory_order_release);
    return false;
  }
  return true;
}

void MecanumHardware::StopExecutor() {
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  executor_.reset();
}

bool MecanumHardware::FeedbackIsFresh(SteadyClock::time_point now) const {
  const SteadyClock::time_point reference =
      has_motor_state_ ? last_motor_state_ : activated_at_;
  return now - reference <= feedback_timeout_;
}

bool MecanumHardware::MotionIsAuthorized() const {
  if (!node_ || !motion_authorization_subscription_) {
    return false;
  }
  const auto publisher_information =
      node_->get_publishers_info_by_topic(kMotionAuthorizationTopic);
  if (publisher_information.size() != std::size_t{1} ||
      publisher_information.front().node_name() !=
          kConfigurationSupervisorName ||
      publisher_information.front().node_namespace() !=
          kConfigurationSupervisorNamespace) {
    return false;
  }
  std::lock_guard<std::mutex> lock(authorization_mutex_);
  const std::uint64_t authorization = motion_authorization_;
  const std::uint32_t configuration_generation =
      static_cast<std::uint32_t>(authorization >> 32U);
  const std::uint32_t authorized_session =
      static_cast<std::uint32_t>(authorization);
  return heartbeat_ready_ && has_heartbeat_ && configuration_generation != 0U &&
         authorized_session != 0U &&
         authorized_session == heartbeat_session_id_;
}

void MecanumHardware::SendZeroMotorCommand() {
  if (!motor_command_publisher_) {
    return;
  }
  mentor_pi_interfaces::msg::MotorCommand command;
  command.update_mask = mentor_pi_interfaces::msg::MotorCommand::ALL_MOTORS;
  command.target_rps.fill(0.0F);
  motor_command_publisher_->publish(command);
}

bool MecanumHardware::SendMotorCommand() {
  if (!motor_command_publisher_) {
    return false;
  }
  double maximum_rps = 0.0;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    maximum_rps = maximum_rps_;
  }
  mentor_pi_interfaces::msg::MotorCommand command;
  command.update_mask = mentor_pi_interfaces::msg::MotorCommand::ALL_MOTORS;
  command.target_rps.fill(0.0F);
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    const auto rps = hardware::RadiansPerSecondToRps(
        joints_.at(joint_names_[wheel]).command.velocity, maximum_rps);
    if (!rps) {
      return false;
    }
    const auto logical = static_cast<hardware::Wheel>(wheel);
    command.target_rps[hardware::McuMotorIndex(logical)] = *rps;
  }
  motor_command_publisher_->publish(command);
  return true;
}

std::size_t MecanumHardware::ParseWheelSlot(
    const std::string& joint_name) const {
  const std::string normalized = ToLower(joint_name);
  const bool front = HasToken(normalized, "front");
  const bool rear = HasToken(normalized, "rear");
  const bool left = HasToken(normalized, "left");
  const bool right = HasToken(normalized, "right");
  if (front && left) {
    return hardware::WheelIndex(hardware::Wheel::kFrontLeft);
  }
  if (front && right) {
    return hardware::WheelIndex(hardware::Wheel::kFrontRight);
  }
  if (rear && left) {
    return hardware::WheelIndex(hardware::Wheel::kRearLeft);
  }
  if (rear && right) {
    return hardware::WheelIndex(hardware::Wheel::kRearRight);
  }
  return hardware::kWheelCount;
}

}  // namespace mentor_pi

PLUGINLIB_EXPORT_CLASS(mentor_pi::MecanumHardware,
                       hardware_interface::SystemInterface)
