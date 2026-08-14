#include "mentor_pi_hardwares/ackermann_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <system_error>

namespace mentor_pi {
namespace {

constexpr char kConfigurationSupervisorName[] = "configuration_supervisor";

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

template <typename Integer>
bool ParseInteger(const std::string& text, Integer minimum, Integer maximum,
                  Integer* output) {
  Integer value{};
  const auto conversion =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (conversion.ec != std::errc{} ||
      conversion.ptr != text.data() + text.size() || value < minimum ||
      value > maximum) {
    return false;
  }
  *output = value;
  return true;
}

bool ParseDouble(const std::string& text, double* output) {
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(value)) {
    return false;
  }
  *output = value;
  return true;
}

bool ParsePositiveDouble(const std::string& text, double* output) {
  double value = 0.0;
  if (!ParseDouble(text, &value) || value <= 0.0) {
    return false;
  }
  *output = value;
  return true;
}

bool ParseBoolean(const std::string& text, bool* output) {
  const std::string normalized = ToLower(text);
  if (normalized == "true" || normalized == "1") {
    *output = true;
    return true;
  }
  if (normalized == "false" || normalized == "0") {
    *output = false;
    return true;
  }
  return false;
}

bool IsValidRobotName(const std::string& name) {
  return !name.empty() && name.front() != '/' && name.back() != '/' &&
         name.find("//") == std::string::npos;
}

}  // namespace

AckermannHardware::~AckermannHardware() { StopExecutor(); }

hardware_interface::CallbackReturn AckermannHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_name_ = HardwareParameter(info, "robot_name", "mentor_pi");
  std::uint32_t timeout_ms = 0U;
  std::uint32_t imu_timeout_ms = 0U;
  std::uint16_t servo_channel = 0U;
  double linear_controller_bandwidth = 0.0;
  double linear_observer_bandwidth = 0.0;
  double yaw_controller_bandwidth = 0.0;
  double yaw_observer_bandwidth = 0.0;
  if (!IsValidRobotName(robot_name_) ||
      !ParseInteger(HardwareParameter(info, "feedback_timeout_ms", "500"), 1U,
                    10000U, &timeout_ms) ||
      !ParseInteger(HardwareParameter(info, "imu_timeout_ms", "500"), 1U,
                    10000U, &imu_timeout_ms) ||
      !ParseInteger(HardwareParameter(info, "steering_pwm_channel", "3"),
                    static_cast<std::uint16_t>(1U),
                    static_cast<std::uint16_t>(4U), &servo_channel) ||
      !ParseInteger(HardwareParameter(info, "steering_pwm_min_us", "500"),
                    static_cast<std::uint16_t>(500U),
                    static_cast<std::uint16_t>(2500U),
                    &steering_calibration_.minimum_pulse_us) ||
      !ParseInteger(HardwareParameter(info, "steering_pwm_center_us", "1500"),
                    static_cast<std::uint16_t>(500U),
                    static_cast<std::uint16_t>(2500U),
                    &steering_calibration_.center_pulse_us) ||
      !ParseInteger(HardwareParameter(info, "steering_pwm_max_us", "2500"),
                    static_cast<std::uint16_t>(500U),
                    static_cast<std::uint16_t>(2500U),
                    &steering_calibration_.maximum_pulse_us) ||
      !ParseDouble(HardwareParameter(info, "steering_angle_min_rad", "-0.6"),
                   &steering_calibration_.minimum_angle_rad) ||
      !ParseDouble(HardwareParameter(info, "steering_angle_max_rad", "0.6"),
                   &steering_calibration_.maximum_angle_rad) ||
      !ParseBoolean(HardwareParameter(info, "steering_inverted", "true"),
                    &steering_calibration_.inverted) ||
      !ParseInteger(HardwareParameter(info, "steering_duration_ms", "20"),
                    static_cast<std::uint16_t>(20U),
                    static_cast<std::uint16_t>(30000U),
                    &steering_duration_ms_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "rear_wheel_radius_m", "0.0325"),
          &rear_wheel_radius_m_) ||
      !ParsePositiveDouble(HardwareParameter(info, "wheelbase_m", "0.135"),
                           &wheelbase_m_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "linear_adrc_input_gain_per_second", "5.0"),
          &linear_adrc_input_gain_per_second_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "linear_adrc_controller_bandwidth_rad_s",
                            "1.0"),
          &linear_controller_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "linear_adrc_observer_bandwidth_rad_s",
                            "3.0"),
          &linear_observer_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_input_gain_per_mps", "30.0"),
          &yaw_adrc_input_gain_per_mps_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_controller_bandwidth_rad_s", "1.0"),
          &yaw_controller_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_observer_bandwidth_rad_s", "3.0"),
          &yaw_observer_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_minimum_speed_mps", "0.1"),
          &yaw_adrc_minimum_speed_mps_) ||
      !linear_adrc_.Configure(linear_controller_bandwidth,
                              linear_observer_bandwidth) ||
      !yaw_adrc_.Configure(yaw_controller_bandwidth, yaw_observer_bandwidth)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  feedback_timeout_ = std::chrono::milliseconds(timeout_ms);
  imu_timeout_ = std::chrono::milliseconds(imu_timeout_ms);
  steering_calibration_.servo_index =
      static_cast<std::size_t>(servo_channel - 1U);
  if (!hardware::IsValidSteeringCalibration(steering_calibration_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ =
      std::make_shared<rclcpp::Node>("ackermann_hardware", "/" + robot_name_);
  if (info.joints.size() != hardware::kWheelCount) {
    RCLCPP_FATAL(node_->get_logger(), "Expected four Ackermann joints");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto& joint : info.joints) {
    const std::size_t slot = ParseWheelSlot(joint.name);
    if (slot >= hardware::kWheelCount) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool front =
        slot == hardware::WheelIndex(hardware::Wheel::kFrontLeft) ||
        slot == hardware::WheelIndex(hardware::Wheel::kFrontRight);
    const auto has_interface = [](const auto& interfaces,
                                  const std::string& name) {
      return std::any_of(
          interfaces.begin(), interfaces.end(),
          [&name](const auto& interface) { return interface.name == name; });
    };
    if (front) {
      const std::size_t steering =
          slot == hardware::WheelIndex(hardware::Wheel::kFrontLeft) ? 0U : 1U;
      if (steering_seen_[steering] || joint.command_interfaces.size() != 1U ||
          !has_interface(joint.command_interfaces,
                         hardware_interface::HW_IF_POSITION) ||
          joint.state_interfaces.size() != 1U ||
          !has_interface(joint.state_interfaces,
                         hardware_interface::HW_IF_POSITION)) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      steering_seen_[steering] = true;
      steering_names_[steering] = joint.name;
    } else {
      if (wheel_seen_[slot] || joint.command_interfaces.size() != 1U ||
          !has_interface(joint.command_interfaces,
                         hardware_interface::HW_IF_VELOCITY) ||
          joint.state_interfaces.size() != 2U ||
          !has_interface(joint.state_interfaces,
                         hardware_interface::HW_IF_POSITION) ||
          !has_interface(joint.state_interfaces,
                         hardware_interface::HW_IF_VELOCITY)) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      wheel_seen_[slot] = true;
      wheel_names_[slot] = joint.name;
    }
    joints_.emplace(joint.name, Joint(joint.name));
  }
  if (!std::all_of(steering_seen_.begin(), steering_seen_.end(),
                   [](bool seen) { return seen; }) ||
      !wheel_seen_[hardware::WheelIndex(hardware::Wheel::kRearLeft)] ||
      !wheel_seen_[hardware::WheelIndex(hardware::Wheel::kRearRight)]) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardware::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!node_ || executor_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  motor_command_publisher_ =
      node_->create_publisher<mentor_pi_interfaces::msg::MotorCommand>(
          "motors/command", qos);
  pwm_command_publisher_ =
      node_->create_publisher<mentor_pi_interfaces::msg::PwmServoCommand>(
          "pwm_servos/command", qos);
  motor_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::MotorState>(
          "motors/state", qos,
          std::bind(&AckermannHardware::MotorStateCallback, this,
                    std::placeholders::_1));
  imu_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::ImuState>(
          "imu", qos,
          std::bind(&AckermannHardware::ImuStateCallback, this,
                    std::placeholders::_1));
  pwm_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::PwmServoState>(
          "pwm_servos/state", qos,
          std::bind(&AckermannHardware::PwmServoStateCallback, this,
                    std::placeholders::_1));
  heartbeat_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::Heartbeat>(
          "heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          std::bind(&AckermannHardware::HeartbeatCallback, this,
                    std::placeholders::_1));
  motion_authorization_subscription_ =
      node_->create_subscription<std_msgs::msg::UInt64>(
          "configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&AckermannHardware::MotionAuthorizationCallback, this,
                    std::placeholders::_1));
  for (auto& entry : joints_) {
    entry.second.state = {};
    entry.second.command = {};
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    velocity_rad_s_.fill(0.0);
    position_rad_.fill(0.0);
    steering_position_rad_ = 0.0;
    yaw_rate_rad_s_ = 0.0;
    maximum_rps_ = 0.0;
    has_motor_state_ = false;
    has_pwm_state_ = false;
    has_imu_state_ = false;
    imu_valid_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(authorization_mutex_);
    motion_authorization_ = 0U;
    authorization_changed_at_ = SteadyClock::now();
    heartbeat_session_id_ = 0U;
    heartbeat_ready_ = false;
    has_heartbeat_ = false;
  }
  executor_failed_.store(false, std::memory_order_release);
  if (!StartExecutor()) {
    SendZeroCommands();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  SendZeroCommands();
  StopExecutor();
  motion_authorization_subscription_.reset();
  heartbeat_subscription_.reset();
  imu_state_subscription_.reset();
  motor_state_subscription_.reset();
  pwm_state_subscription_.reset();
  motor_command_publisher_.reset();
  pwm_command_publisher_.reset();
  {
    std::lock_guard<std::mutex> lock(authorization_mutex_);
    motion_authorization_ = 0U;
    heartbeat_session_id_ = 0U;
    heartbeat_ready_ = false;
    has_heartbeat_ = false;
  }
  ResetChassisAdrc();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardware::on_activate(
    const rclcpp_lifecycle::State&) {
  if (executor_failed_.load(std::memory_order_acquire)) {
    SendZeroCommands();
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto& entry : joints_) {
    entry.second.command = {};
  }
  ResetChassisAdrc();
  active_ = true;
  SendZeroCommands();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  ResetChassisAdrc();
  SendZeroCommands();
  for (auto& entry : joints_) {
    entry.second.command = {};
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AckermannHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (const auto& name : steering_names_) {
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &joints_.at(name).state.position);
  }
  for (hardware::Wheel wheel :
       {hardware::Wheel::kRearLeft, hardware::Wheel::kRearRight}) {
    const auto& name = wheel_names_[hardware::WheelIndex(wheel)];
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &joints_.at(name).state.position);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &joints_.at(name).state.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
AckermannHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (const auto& name : steering_names_) {
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION,
                            &joints_.at(name).command.position);
  }
  for (hardware::Wheel wheel :
       {hardware::Wheel::kRearLeft, hardware::Wheel::kRearRight}) {
    const auto& name = wheel_names_[hardware::WheelIndex(wheel)];
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY,
                            &joints_.at(name).command.velocity);
  }
  return interfaces;
}

hardware_interface::return_type AckermannHardware::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  double steering = 0.0;
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    fresh = FeedbackIsFresh(SteadyClock::now());
    velocity = velocity_rad_s_;
    position = position_rad_;
    steering = steering_position_rad_;
  }
  if (active_ && !fresh) {
    SendZeroCommands();
    if (!MotionIsAuthorized() || FeedbackCanSettle(SteadyClock::now())) {
      return hardware_interface::return_type::OK;
    }
    return hardware_interface::return_type::ERROR;
  }
  for (const auto& name : steering_names_) {
    joints_.at(name).state.position = steering;
  }
  for (hardware::Wheel wheel :
       {hardware::Wheel::kRearLeft, hardware::Wheel::kRearRight}) {
    const std::size_t index = hardware::WheelIndex(wheel);
    auto& joint = joints_.at(wheel_names_[index]);
    joint.state.velocity = velocity[index];
    joint.state.position = position[index];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannHardware::write(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (executor_failed_.load(std::memory_order_acquire)) {
    ResetChassisAdrc();
    SendZeroCommands();
    return hardware_interface::return_type::ERROR;
  }
  if (!MotionIsAuthorized()) {
    ResetChassisAdrc();
    SendZeroCommands();
    return hardware_interface::return_type::OK;
  }
  bool feedback_fresh = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_fresh = FeedbackIsFresh(SteadyClock::now());
  }
  if (!feedback_fresh) {
    ResetChassisAdrc();
    SendZeroCommands();
    if (FeedbackCanSettle(SteadyClock::now())) {
      return hardware_interface::return_type::OK;
    }
    return hardware_interface::return_type::ERROR;
  }
  if (!SendDriveAndSteeringCommands(period.seconds())) {
    ResetChassisAdrc();
    SendZeroCommands();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void AckermannHardware::MotorStateCallback(
    const mentor_pi_interfaces::msg::MotorState::SharedPtr message) {
  const auto maximum_rps = hardware::MotorMaximumRps(message->motor_model);
  const auto ticks = hardware::MotorTicksPerRevolution(message->motor_model);
  if (!maximum_rps || !ticks) {
    return;
  }
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  for (hardware::Wheel wheel :
       {hardware::Wheel::kRearLeft, hardware::Wheel::kRearRight}) {
    const std::size_t logical = hardware::WheelIndex(wheel);
    const std::size_t motor = hardware::McuMotorIndex(wheel);
    const double direction =
        static_cast<double>(hardware::ChassisDirectionSign(wheel));
    velocity[logical] = hardware::RpsToRadiansPerSecond(
        direction * static_cast<double>(message->measured_rps[motor]));
    position[logical] = direction * hardware::EncoderCountToRadians(
                                        message->encoder_count[motor], *ticks);
    if (!std::isfinite(velocity[logical]) ||
        !std::isfinite(position[logical])) {
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

void AckermannHardware::ImuStateCallback(
    const mentor_pi_interfaces::msg::ImuState::SharedPtr message) {
  const double yaw_rate =
      static_cast<double>(message->angular_velocity_rad_s[2]);
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_imu_state_ = SteadyClock::now();
  has_imu_state_ = true;
  imu_valid_ = message->valid && std::isfinite(yaw_rate);
  yaw_rate_rad_s_ = imu_valid_ ? yaw_rate : 0.0;
}

void AckermannHardware::PwmServoStateCallback(
    const mentor_pi_interfaces::msg::PwmServoState::SharedPtr message) {
  const auto angle = hardware::SteeringPulseToAngle(
      message->output_pulse_width_us[steering_calibration_.servo_index],
      steering_calibration_);
  if (!angle) {
    return;
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  steering_position_rad_ = *angle;
  last_pwm_state_ = SteadyClock::now();
  has_pwm_state_ = true;
}

void AckermannHardware::HeartbeatCallback(
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

void AckermannHardware::MotionAuthorizationCallback(
    const std_msgs::msg::UInt64::SharedPtr message) {
  std::lock_guard<std::mutex> lock(authorization_mutex_);
  if (message->data != motion_authorization_) {
    authorization_changed_at_ = SteadyClock::now();
  }
  motion_authorization_ = message->data;
}

bool AckermannHardware::StartExecutor() {
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
        SendZeroCommands();
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

void AckermannHardware::StopExecutor() {
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

bool AckermannHardware::FeedbackIsFresh(SteadyClock::time_point now) const {
  return has_motor_state_ && has_pwm_state_ && has_imu_state_ && imu_valid_ &&
         now - last_motor_state_ <= feedback_timeout_ &&
         now - last_pwm_state_ <= feedback_timeout_ &&
         now - last_imu_state_ <= imu_timeout_;
}

bool AckermannHardware::FeedbackCanSettle(SteadyClock::time_point now) const {
  bool waiting_for_motor = false;
  bool waiting_for_pwm = false;
  bool waiting_for_imu = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    waiting_for_motor = !has_motor_state_;
    waiting_for_pwm = !has_pwm_state_;
    waiting_for_imu = !has_imu_state_;
    if ((!waiting_for_motor && now - last_motor_state_ > feedback_timeout_) ||
        (!waiting_for_pwm && now - last_pwm_state_ > feedback_timeout_) ||
        (!waiting_for_imu &&
         (!imu_valid_ || now - last_imu_state_ > imu_timeout_)) ||
        (!waiting_for_motor && !waiting_for_pwm && !waiting_for_imu)) {
      return false;
    }
  }
  std::lock_guard<std::mutex> lock(authorization_mutex_);
  const auto settling_time = now - authorization_changed_at_;
  return (!waiting_for_motor || settling_time <= feedback_timeout_) &&
         (!waiting_for_pwm || settling_time <= feedback_timeout_) &&
         (!waiting_for_imu || settling_time <= imu_timeout_);
}

bool AckermannHardware::MotionIsAuthorized() const {
  if (!node_ || !motion_authorization_subscription_) {
    return false;
  }
  const auto publisher_information =
      node_->get_publishers_info_by_topic("configuration/motion_authorization");
  if (publisher_information.size() != std::size_t{1} ||
      publisher_information.front().node_name() !=
          kConfigurationSupervisorName ||
      publisher_information.front().node_namespace() != "/" + robot_name_) {
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

void AckermannHardware::SendZeroCommands() {
  if (motor_command_publisher_) {
    mentor_pi_interfaces::msg::MotorCommand command;
    command.update_mask = static_cast<std::uint8_t>(
        hardware::McuMotorMask(hardware::Wheel::kRearLeft) |
        hardware::McuMotorMask(hardware::Wheel::kRearRight));
    command.target_rps.fill(0.0F);
    motor_command_publisher_->publish(command);
  }
  if (pwm_command_publisher_) {
    mentor_pi_interfaces::msg::PwmServoCommand command;
    command.update_mask =
        static_cast<std::uint8_t>(1U << steering_calibration_.servo_index);
    command.duration_ms = steering_duration_ms_;
    command.pulse_width_us.fill(1500U);
    command.pulse_width_us[steering_calibration_.servo_index] =
        steering_calibration_.center_pulse_us;
    pwm_command_publisher_->publish(command);
  }
}

void AckermannHardware::ResetChassisAdrc() {
  linear_adrc_.Reset();
  yaw_adrc_.Reset();
  applied_linear_correction_m_s_ = 0.0;
  applied_steering_correction_rad_ = 0.0;
}

bool AckermannHardware::SendDriveAndSteeringCommands(double period_seconds) {
  if (!motor_command_publisher_ || !pwm_command_publisher_) {
    return false;
  }
  const double left = joints_.at(steering_names_[0]).command.position;
  const double right = joints_.at(steering_names_[1]).command.position;
  if (!std::isfinite(left) || !std::isfinite(right)) {
    return false;
  }
  std::array<double, 2U> reference_rear_velocity{
      joints_.at(wheel_names_[hardware::WheelIndex(hardware::Wheel::kRearLeft)])
          .command.velocity,
      joints_
          .at(wheel_names_[hardware::WheelIndex(hardware::Wheel::kRearRight)])
          .command.velocity};
  if (!std::isfinite(reference_rear_velocity[0]) ||
      !std::isfinite(reference_rear_velocity[1])) {
    return false;
  }
  if (reference_rear_velocity[0] == 0.0 && reference_rear_velocity[1] == 0.0) {
    ResetChassisAdrc();
    SendZeroCommands();
    return true;
  }

  double maximum_rps = 0.0;
  std::array<double, hardware::kWheelCount> measured_wheel_velocity{};
  double measured_yaw_rate = 0.0;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    maximum_rps = maximum_rps_;
    measured_wheel_velocity = velocity_rad_s_;
    measured_yaw_rate = yaw_rate_rad_s_;
  }
  const double reference_speed_m_s =
      rear_wheel_radius_m_ *
      (reference_rear_velocity[0] + reference_rear_velocity[1]) * 0.5;
  const double measured_speed_m_s =
      rear_wheel_radius_m_ *
      (measured_wheel_velocity[hardware::WheelIndex(
           hardware::Wheel::kRearLeft)] +
       measured_wheel_velocity[hardware::WheelIndex(
           hardware::Wheel::kRearRight)]) *
      0.5;
  const auto linear_correction = linear_adrc_.Update(
      reference_speed_m_s, measured_speed_m_s, applied_linear_correction_m_s_,
      linear_adrc_input_gain_per_second_, period_seconds);
  if (!linear_correction) {
    return false;
  }
  std::array<double, 2U> target_rear_velocity{
      reference_rear_velocity[0] + *linear_correction / rear_wheel_radius_m_,
      reference_rear_velocity[1] + *linear_correction / rear_wheel_radius_m_};
  const double maximum_wheel_velocity = maximum_rps * hardware::kTwoPi;
  const double largest_wheel_velocity = std::max(
      std::fabs(target_rear_velocity[0]), std::fabs(target_rear_velocity[1]));
  if (!std::isfinite(maximum_wheel_velocity) || maximum_wheel_velocity <= 0.0 ||
      !std::isfinite(largest_wheel_velocity)) {
    return false;
  }
  if (largest_wheel_velocity > maximum_wheel_velocity) {
    const double scale = maximum_wheel_velocity / largest_wheel_velocity;
    target_rear_velocity[0] *= scale;
    target_rear_velocity[1] *= scale;
  }
  applied_linear_correction_m_s_ =
      rear_wheel_radius_m_ *
          (target_rear_velocity[0] + target_rear_velocity[1]) * 0.5 -
      reference_speed_m_s;

  const double feedforward_steering =
      std::clamp((left + right) * 0.5, steering_calibration_.minimum_angle_rad,
                 steering_calibration_.maximum_angle_rad);
  double steering_command = feedforward_steering;
  if (std::fabs(measured_speed_m_s) < yaw_adrc_minimum_speed_mps_) {
    yaw_adrc_.Reset();
    applied_steering_correction_rad_ = 0.0;
  } else {
    const double reference_yaw_rate =
        reference_speed_m_s * std::tan(feedforward_steering) / wheelbase_m_;
    const double yaw_input_gain =
        yaw_adrc_input_gain_per_mps_ * measured_speed_m_s;
    const auto steering_correction = yaw_adrc_.Update(
        reference_yaw_rate, measured_yaw_rate, applied_steering_correction_rad_,
        yaw_input_gain, period_seconds);
    if (!steering_correction) {
      return false;
    }
    steering_command = std::clamp(feedforward_steering + *steering_correction,
                                  steering_calibration_.minimum_angle_rad,
                                  steering_calibration_.maximum_angle_rad);
    applied_steering_correction_rad_ = steering_command - feedforward_steering;
  }
  const auto pulse =
      hardware::SteeringAngleToPulse(steering_command, steering_calibration_);
  if (!pulse) {
    return false;
  }

  mentor_pi_interfaces::msg::MotorCommand motor_command;
  motor_command.update_mask = static_cast<std::uint8_t>(
      hardware::McuMotorMask(hardware::Wheel::kRearLeft) |
      hardware::McuMotorMask(hardware::Wheel::kRearRight));
  motor_command.target_rps.fill(0.0F);
  std::size_t rear_index = 0U;
  for (hardware::Wheel wheel :
       {hardware::Wheel::kRearLeft, hardware::Wheel::kRearRight}) {
    const double direction =
        static_cast<double>(hardware::ChassisDirectionSign(wheel));
    const auto rps = hardware::RadiansPerSecondToRps(
        direction * target_rear_velocity[rear_index], maximum_rps);
    if (!rps) {
      return false;
    }
    motor_command.target_rps[hardware::McuMotorIndex(wheel)] = *rps;
    ++rear_index;
  }

  mentor_pi_interfaces::msg::PwmServoCommand pwm_command;
  pwm_command.update_mask =
      static_cast<std::uint8_t>(1U << steering_calibration_.servo_index);
  pwm_command.duration_ms = steering_duration_ms_;
  pwm_command.pulse_width_us.fill(1500U);
  pwm_command.pulse_width_us[steering_calibration_.servo_index] = *pulse;
  pwm_command_publisher_->publish(pwm_command);
  motor_command_publisher_->publish(motor_command);
  return true;
}

std::size_t AckermannHardware::ParseWheelSlot(
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

PLUGINLIB_EXPORT_CLASS(mentor_pi::AckermannHardware,
                       hardware_interface::SystemInterface)
