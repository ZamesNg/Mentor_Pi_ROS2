#include "mentor_pi_hardwares/mecanum_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <pluginlib/class_list_macros.hpp>
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

bool ParsePositiveDouble(const std::string& text, double* output) {
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(value) ||
      value <= 0.0) {
    return false;
  }
  *output = value;
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
  double linear_controller_bandwidth = 0.0;
  double linear_observer_bandwidth = 0.0;
  double yaw_controller_bandwidth = 0.0;
  double yaw_observer_bandwidth = 0.0;
  double linear_measurement_lpf_cutoff_hz = 0.0;
  double yaw_measurement_lpf_cutoff_hz = 0.0;
  if (!IsValidRobotName(robot_name_) ||
      !ParsePositiveMilliseconds(
          HardwareParameter(info, "feedback_timeout_ms", "100"),
          &feedback_timeout_) ||
      !ParsePositiveMilliseconds(
          HardwareParameter(info, "imu_timeout_ms", "100"), &imu_timeout_) ||
      !ParsePositiveDouble(HardwareParameter(info, "wheel_radius_m", "0.0325"),
                           &wheel_radius_m_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "wheel_projection_sum_m", "0.14"),
          &wheel_projection_sum_m_) ||
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
          HardwareParameter(info, "linear_adrc_measurement_lpf_cutoff_hz",
                            "5.0"),
          &linear_measurement_lpf_cutoff_hz) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_input_gain_per_second", "5.0"),
          &yaw_adrc_input_gain_per_second_) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_controller_bandwidth_rad_s", "1.0"),
          &yaw_controller_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_observer_bandwidth_rad_s", "3.0"),
          &yaw_observer_bandwidth) ||
      !ParsePositiveDouble(
          HardwareParameter(info, "yaw_adrc_measurement_lpf_cutoff_hz", "5.0"),
          &yaw_measurement_lpf_cutoff_hz) ||
      !chassis_adrc_[0].Configure(linear_controller_bandwidth,
                                  linear_observer_bandwidth) ||
      !chassis_adrc_[1].Configure(linear_controller_bandwidth,
                                  linear_observer_bandwidth) ||
      !chassis_adrc_[2].Configure(yaw_controller_bandwidth,
                                  yaw_observer_bandwidth) ||
      !measurement_lpf_[0].Configure(linear_measurement_lpf_cutoff_hz) ||
      !measurement_lpf_[1].Configure(linear_measurement_lpf_cutoff_hz) ||
      !measurement_lpf_[2].Configure(yaw_measurement_lpf_cutoff_hz)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!reconnect_gate_.Configure(
          static_cast<std::uint8_t>(
              hardware::FeedbackMask(hardware::FeedbackStream::kMotor) |
              hardware::FeedbackMask(hardware::FeedbackStream::kImu)),
          feedback_timeout_, imu_timeout_, feedback_timeout_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  node_ = std::make_shared<rclcpp::Node>(hardware::kVehicleHardwareNodeName,
                                        "/" + robot_name_);

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
          "motors/command", qos);
  motor_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::MotorState>(
          "motors/state", qos,
          std::bind(&MecanumHardware::MotorStateCallback, this,
                    std::placeholders::_1));
  imu_state_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::ImuState>(
          "imu", qos,
          std::bind(&MecanumHardware::ImuStateCallback, this,
                    std::placeholders::_1));
  heartbeat_subscription_ =
      node_->create_subscription<mentor_pi_interfaces::msg::Heartbeat>(
          "heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          std::bind(&MecanumHardware::HeartbeatCallback, this,
                    std::placeholders::_1));
  motion_authorization_subscription_ =
      node_->create_subscription<std_msgs::msg::UInt64>(
          "configuration/motion_authorization",
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
    yaw_rate_rad_s_ = 0.0;
    maximum_rps_ = 0.0;
  }
  reconnect_gate_.Reset(SteadyClock::now());
  executor_failed_.store(false, std::memory_order_release);
  if (!StartExecutor()) {
    SendZeroMotorCommand();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_cleanup(
    const rclcpp_lifecycle::State&) {
  return Teardown();
}

hardware_interface::CallbackReturn MecanumHardware::on_error(
    const rclcpp_lifecycle::State&) {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ros2_control entered the Mecanum hardware error path");
  }
  return Teardown();
}

hardware_interface::CallbackReturn MecanumHardware::Teardown() {
  active_ = false;
  ResetChassisAdrc();
  SendZeroMotorCommand();
  StopExecutor();
  motion_authorization_subscription_.reset();
  heartbeat_subscription_.reset();
  imu_state_subscription_.reset();
  motor_state_subscription_.reset();
  motor_command_publisher_.reset();
  for (auto& entry : joints_) {
    entry.second.state = {};
    entry.second.command = {};
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    velocity_rad_s_.fill(0.0);
    position_rad_.fill(0.0);
    yaw_rate_rad_s_ = 0.0;
    maximum_rps_ = 0.0;
  }
  reconnect_gate_.Reset(SteadyClock::now());
  executor_failed_.store(false, std::memory_order_release);
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
  ResetChassisAdrc();
  active_ = true;
  SendZeroMotorCommand();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardware::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  ResetChassisAdrc();
  reconnect_gate_.Reset(SteadyClock::now());
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
  if (active_) {
    if (executor_failed_.load(std::memory_order_acquire)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Mecanum read failed because the private ROS executor "
                   "stopped unexpectedly");
      ResetChassisAdrc();
      SendZeroMotorCommand();
      return hardware_interface::return_type::ERROR;
    }
    const auto reconnect = EvaluateReconnect(SteadyClock::now());
    if (!reconnect.ready) {
      ResetChassisAdrc();
      SendZeroMotorCommand();
      for (const auto& name : joint_names_) {
        joints_.at(name).state.velocity = 0.0;
      }
      return hardware_interface::return_type::OK;
    }
  }
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    velocity = velocity_rad_s_;
    position = position_rad_;
  }
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    auto& joint = joints_.at(joint_names_[wheel]);
    joint.state.velocity = velocity[wheel];
    joint.state.position = position[wheel];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumHardware::write(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (executor_failed_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Mecanum write failed because the private ROS executor "
                 "stopped unexpectedly");
    ResetChassisAdrc();
    SendZeroMotorCommand();
    return hardware_interface::return_type::ERROR;
  }
  const auto reconnect = EvaluateReconnect(SteadyClock::now());
  if (!reconnect.ready) {
    ResetChassisAdrc();
    SendZeroMotorCommand();
    return hardware_interface::return_type::OK;
  }
  if (!SendMotorCommand(period.seconds())) {
    ResetChassisAdrc();
    SendZeroMotorCommand();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void MecanumHardware::MotorStateCallback(
    const mentor_pi_interfaces::msg::MotorState::SharedPtr message) {
  const auto now = SteadyClock::now();
  const auto maximum_rps = hardware::MotorMaximumRps(message->motor_model);
  const auto ticks = hardware::MotorTicksPerRevolution(message->motor_model);
  if (!maximum_rps || !ticks) {
    reconnect_gate_.ObserveFeedback(hardware::FeedbackStream::kMotor, false,
                                    now);
    return;
  }
  std::array<double, hardware::kWheelCount> velocity{};
  std::array<double, hardware::kWheelCount> position{};
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    const auto logical = static_cast<hardware::Wheel>(wheel);
    const std::size_t motor = hardware::McuMotorIndex(logical);
    const double direction =
        static_cast<double>(hardware::ChassisDirectionSign(logical));
    velocity[wheel] = hardware::RpsToRadiansPerSecond(
        direction * static_cast<double>(message->measured_rps[motor]));
    position[wheel] = direction * hardware::EncoderCountToRadians(
                                      message->encoder_count[motor], *ticks);
    if (!std::isfinite(velocity[wheel]) || !std::isfinite(position[wheel])) {
      reconnect_gate_.ObserveFeedback(hardware::FeedbackStream::kMotor, false,
                                      now);
      return;
    }
  }
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    velocity_rad_s_ = velocity;
    position_rad_ = position;
    maximum_rps_ = *maximum_rps;
  }
  reconnect_gate_.ObserveFeedback(hardware::FeedbackStream::kMotor, true, now);
}

void MecanumHardware::ImuStateCallback(
    const mentor_pi_interfaces::msg::ImuState::SharedPtr message) {
  const auto now = SteadyClock::now();
  const double yaw_rate =
      static_cast<double>(message->angular_velocity_rad_s[2]);
  const bool valid = message->valid && std::isfinite(yaw_rate);
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    yaw_rate_rad_s_ = valid ? yaw_rate : 0.0;
  }
  reconnect_gate_.ObserveFeedback(hardware::FeedbackStream::kImu, valid, now);
}

void MecanumHardware::HeartbeatCallback(
    const mentor_pi_interfaces::msg::Heartbeat::SharedPtr message) {
  const bool ready =
      message->agent_session_id != 0U &&
      (message->state == mentor_pi_interfaces::msg::Heartbeat::READY ||
       message->state == mentor_pi_interfaces::msg::Heartbeat::DEGRADED);
  reconnect_gate_.ObserveHeartbeat(
      message->agent_session_id, message->uptime_ms, ready, SteadyClock::now());
}

void MecanumHardware::MotionAuthorizationCallback(
    const std_msgs::msg::UInt64::SharedPtr message) {
  reconnect_gate_.ObserveAuthorization(message->data, SteadyClock::now());
}

bool MecanumHardware::StartExecutor() {
  if (!node_ || executor_) {
    return false;
  }
  try {
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_stop_requested_.store(false, std::memory_order_release);
    executor_thread_ = std::thread([this]() {
      try {
        while (!executor_stop_requested_.load(std::memory_order_acquire)) {
          executor_->spin_once(std::chrono::milliseconds(100));
        }
      } catch (const std::exception& error) {
        if (!executor_stop_requested_.load(std::memory_order_acquire)) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Mecanum private ROS executor exception: %s",
                       error.what());
          executor_failed_.store(true, std::memory_order_release);
          SendZeroMotorCommand();
        }
      } catch (...) {
        if (!executor_stop_requested_.load(std::memory_order_acquire)) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Mecanum private ROS executor failed with an unknown "
                       "exception");
          executor_failed_.store(true, std::memory_order_release);
          SendZeroMotorCommand();
        }
      }
    });
  } catch (const std::exception& error) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Mecanum private ROS executor setup failed: %s",
                   error.what());
    }
    if (executor_ && node_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    executor_failed_.store(true, std::memory_order_release);
    return false;
  } catch (...) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Mecanum private ROS executor setup failed with an "
                   "unknown exception");
    }
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
  executor_stop_requested_.store(true, std::memory_order_release);
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

bool MecanumHardware::AuthorizationPublisherIsValid() const {
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
  return true;
}

hardware::ReconnectStatus MecanumHardware::EvaluateReconnect(
    SteadyClock::time_point now) {
  const auto status =
      reconnect_gate_.Evaluate(AuthorizationPublisherIsValid(), now);
  LogReconnectTransition(status);
  return status;
}

void MecanumHardware::LogReconnectTransition(
    const hardware::ReconnectStatus& status) const {
  if (!node_ || !status.transition) {
    return;
  }
  if (status.ready) {
    RCLCPP_INFO(node_->get_logger(),
                "reconnect recovery complete: reason=%s session=%u "
                "uptime_ms=%u authorization_generation=%u",
                hardware::RecoveryReasonName(status.reason), status.session_id,
                status.uptime_ms, status.authorization_generation);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "motion inhibited for reconnect recovery: reason=%s "
                "session=%u uptime_ms=%u authorization_generation=%u",
                hardware::RecoveryReasonName(status.reason), status.session_id,
                status.uptime_ms, status.authorization_generation);
  }
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

void MecanumHardware::ResetChassisAdrc() {
  for (auto& controller : chassis_adrc_) {
    controller.Reset();
  }
  for (auto& filter : measurement_lpf_) {
    filter.Reset();
  }
  applied_correction_.fill(0.0);
}

bool MecanumHardware::SendMotorCommand(double period_seconds) {
  if (!motor_command_publisher_) {
    return false;
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
  std::array<double, hardware::kWheelCount> reference_wheel_velocity{};
  bool all_zero = true;
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    reference_wheel_velocity[wheel] =
        joints_.at(joint_names_[wheel]).command.velocity;
    if (!std::isfinite(reference_wheel_velocity[wheel])) {
      return false;
    }
    all_zero = all_zero && reference_wheel_velocity[wheel] == 0.0;
  }
  if (all_zero) {
    ResetChassisAdrc();
    SendZeroMotorCommand();
    return true;
  }

  const auto inverse_kinematics = [this](const std::array<double, 4U>& wheel) {
    const double scale = wheel_radius_m_ * 0.25;
    return std::array<double, 3U>{
        scale * (wheel[0] + wheel[1] + wheel[2] + wheel[3]),
        scale * (-wheel[0] + wheel[1] + wheel[2] - wheel[3]),
        scale * (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) /
            wheel_projection_sum_m_};
  };
  const auto reference = inverse_kinematics(reference_wheel_velocity);
  auto measured = inverse_kinematics(measured_wheel_velocity);
  measured[2] = measured_yaw_rate;
  for (std::size_t axis = 0U; axis < measured.size(); ++axis) {
    const auto filtered =
        measurement_lpf_[axis].Update(measured[axis], period_seconds);
    if (!filtered) {
      return false;
    }
    measured[axis] = *filtered;
  }

  std::array<double, 3U> correction{};
  for (std::size_t axis = 0U; axis < correction.size(); ++axis) {
    const double input_gain = axis < 2U ? linear_adrc_input_gain_per_second_
                                        : yaw_adrc_input_gain_per_second_;
    const auto update = chassis_adrc_[axis].Update(
        reference[axis], measured[axis], applied_correction_[axis], input_gain,
        period_seconds);
    if (!update) {
      return false;
    }
    correction[axis] = *update;
  }
  const std::array<double, 3U> corrected{reference[0] + correction[0],
                                         reference[1] + correction[1],
                                         reference[2] + correction[2]};
  std::array<double, hardware::kWheelCount> target_wheel_velocity{
      (corrected[0] - corrected[1] - wheel_projection_sum_m_ * corrected[2]) /
          wheel_radius_m_,
      (corrected[0] + corrected[1] + wheel_projection_sum_m_ * corrected[2]) /
          wheel_radius_m_,
      (corrected[0] + corrected[1] - wheel_projection_sum_m_ * corrected[2]) /
          wheel_radius_m_,
      (corrected[0] - corrected[1] + wheel_projection_sum_m_ * corrected[2]) /
          wheel_radius_m_};
  const double maximum_wheel_velocity = maximum_rps * hardware::kTwoPi;
  double largest_wheel_velocity = 0.0;
  for (const double value : target_wheel_velocity) {
    largest_wheel_velocity = std::max(largest_wheel_velocity, std::fabs(value));
  }
  if (!std::isfinite(maximum_wheel_velocity) || maximum_wheel_velocity <= 0.0 ||
      !std::isfinite(largest_wheel_velocity)) {
    return false;
  }
  if (largest_wheel_velocity > maximum_wheel_velocity) {
    const double scale = maximum_wheel_velocity / largest_wheel_velocity;
    for (double& value : target_wheel_velocity) {
      value *= scale;
    }
  }
  const auto applied = inverse_kinematics(target_wheel_velocity);
  for (std::size_t axis = 0U; axis < applied_correction_.size(); ++axis) {
    applied_correction_[axis] = applied[axis] - reference[axis];
  }

  mentor_pi_interfaces::msg::MotorCommand command;
  command.update_mask = mentor_pi_interfaces::msg::MotorCommand::ALL_MOTORS;
  command.target_rps.fill(0.0F);
  for (std::size_t wheel = 0U; wheel < hardware::kWheelCount; ++wheel) {
    const auto logical = static_cast<hardware::Wheel>(wheel);
    const double direction =
        static_cast<double>(hardware::ChassisDirectionSign(logical));
    const auto rps = hardware::RadiansPerSecondToRps(
        direction * target_wheel_velocity[wheel], maximum_rps);
    if (!rps) {
      return false;
    }
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
