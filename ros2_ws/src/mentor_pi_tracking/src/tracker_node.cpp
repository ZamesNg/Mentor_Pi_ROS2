// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mentor_pi_interfaces/motor_profile_contract.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mentor_pi::tracking {
namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr auto kControlPeriod = std::chrono::nanoseconds(33'333'333);
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(100);
constexpr auto kSolveDeadline = std::chrono::milliseconds(25);
constexpr auto kMinimumStartLead = std::chrono::milliseconds(250);
constexpr auto kMaximumStartLead = std::chrono::seconds(60);
constexpr double kTwoPi = 2.0 * std::acos(-1.0);
constexpr char kMotionAuthorizationTopic[] =
    "/mentor_pi/configuration/motion_authorization";
constexpr char kConfigurationSupervisorName[] = "configuration_supervisor";
constexpr char kConfigurationSupervisorNamespace[] = "/mentor_pi";

struct ScheduledTrajectory {
  std::shared_ptr<const PolynomialTrajectory> trajectory;
  SteadyClock::time_point start;
};

struct WorkItem {
  std::uint64_t sequence{};
  SteadyClock::time_point deadline;
  MpcConfiguration configuration;
  MpcRequest request;
  std::shared_ptr<const PolynomialTrajectory> trajectory;
};

struct WorkResult {
  std::uint64_t sequence{};
  std::string trajectory_id;
  MpcCommand command;
  std::chrono::steady_clock::duration duration{};
  SteadyClock::time_point completed;
};

class TrackerNode final : public rclcpp::Node {
 public:
  TrackerNode(VehicleType vehicle, const rclcpp::NodeOptions& options)
      : Node(vehicle == VehicleType::kMecanum ? "mecanum_mpc_tracker"
                                              : "ackermann_mpc_tracker",
             "/mentor_pi", options) {
    configuration_.vehicle = vehicle;
    configuration_.horizon = declare_parameter<int>("horizon", 10);
    configuration_.prediction_step =
        declare_parameter<double>("prediction_step", 0.1);
    configuration_.wheelbase = declare_parameter<double>("wheelbase", 0.145);
    wheel_radius_ = declare_parameter<double>(
        "wheel_radius", vehicle == VehicleType::kMecanum ? 0.0325 : 0.0333);
    mecanum_radius_sum_ = declare_parameter<double>("mecanum_radius_sum", 0.14);
    configuration_.mecanum_radius_sum = mecanum_radius_sum_;
    configured_steering_limit_ =
        declare_parameter<double>("max_steering_angle", 0.5);
    if (configuration_.horizon != 10 ||
        std::abs(configuration_.prediction_step - 0.1) > 1.0e-9 ||
        configuration_.wheelbase <= 0.0 || wheel_radius_ <= 0.0 ||
        mecanum_radius_sum_ <= 0.0 || configured_steering_limit_ <= 0.0) {
      throw std::invalid_argument("invalid fixed tracking configuration");
    }
    configuration_.max_steering_angle = configured_steering_limit_;

    const std::string controller = vehicle == VehicleType::kMecanum
                                       ? "mecanum_drive_controller"
                                       : "ackermann_steering_controller";
    const std::string tracker = vehicle == VehicleType::kMecanum
                                    ? "mecanum_mpc_tracker"
                                    : "ackermann_mpc_tracker";
    command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mentor_pi/" + controller + "/reference",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    safety_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions safety_subscription_options;
    safety_subscription_options.callback_group = safety_callback_group_;
    trajectory_subscription_ = create_subscription<
        mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
        "/mentor_pi/" + tracker + "/reference_trajectory",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [this](const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory::
                   SharedPtr message) { AcceptTrajectory(*message); },
        safety_subscription_options);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mentor_pi/" + controller + "/odometry", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          const auto& orientation = message->pose.pose.orientation;
          const auto& position = message->pose.pose.position;
          const double quaternion_norm =
              std::hypot(std::hypot(orientation.x, orientation.y),
                         std::hypot(orientation.z, orientation.w));
          if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
              !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9) {
            {
              std::lock_guard<std::mutex> lock(state_mutex_);
              has_odometry_ = false;
              PublishZero();
            }
            PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "non-finite or invalid odometry rejected");
            return;
          }
          const double qx = orientation.x / quaternion_norm;
          const double qy = orientation.y / quaternion_norm;
          const double qz = orientation.z / quaternion_norm;
          const double qw = orientation.w / quaternion_norm;
          const double sin_yaw = 2.0 * (qw * qz + qx * qy);
          const double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);
          std::lock_guard<std::mutex> lock(state_mutex_);
          state_ = {position.x, position.y, std::atan2(sin_yaw, cos_yaw)};
          odometry_time_ = SteadyClock::now();
          has_odometry_ = true;
        },
        safety_subscription_options);
    motor_subscription_ =
        create_subscription<mentor_pi_interfaces::msg::MotorState>(
            "/mentor_pi/motors/state", rclcpp::SensorDataQoS(),
            [this](const mentor_pi_interfaces::msg::MotorState::SharedPtr
                       message) {
              const auto* profile =
                  mentor_pi_interfaces::FindMotorProfileContract(
                      message->motor_model);
              std::lock_guard<std::mutex> lock(state_mutex_);
              if (profile == nullptr) {
                has_motor_profile_ = false;
                PublishZero();
                return;
              }
              const double wheel_speed =
                  std::min(6.0, static_cast<double>(profile->max_rps)) *
                  kTwoPi * wheel_radius_;
              configuration_.max_linear_speed = wheel_speed;
              configuration_.max_lateral_speed = wheel_speed;
              configuration_.max_yaw_rate = wheel_speed / mecanum_radius_sum_;
              motor_time_ = SteadyClock::now();
              has_motor_profile_ = true;
            },
            safety_subscription_options);
    authorization_subscription_ = create_subscription<std_msgs::msg::UInt64>(
        kMotionAuthorizationTopic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](const std_msgs::msg::UInt64::SharedPtr message) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          motion_authorization_ = message->data;
          if (motion_authorization_ == 0U) {
            PublishZero();
          }
        },
        safety_subscription_options);
    heartbeat_subscription_ =
        create_subscription<mentor_pi_interfaces::msg::Heartbeat>(
            "/mentor_pi/heartbeat",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            [this](const mentor_pi_interfaces::msg::Heartbeat::SharedPtr
                       message) {
              const bool ready =
                  message->agent_session_id != 0U &&
                  (message->state ==
                       mentor_pi_interfaces::msg::Heartbeat::READY ||
                   message->state ==
                       mentor_pi_interfaces::msg::Heartbeat::DEGRADED);
              std::lock_guard<std::mutex> lock(state_mutex_);
              heartbeat_session_id_ = message->agent_session_id;
              heartbeat_ready_ = ready;
              has_heartbeat_ = true;
              if (!ready) {
                PublishZero();
              }
            },
            safety_subscription_options);
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
        "/mentor_pi/" + tracker + "/cancel",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active_.reset();
          pending_.reset();
          fallback_started_.reset();
          PublishZero();
          response->success = true;
          response->message = "active and pending trajectories cancelled";
        },
        rmw_qos_profile_services_default, safety_callback_group_);
    timer_ = create_wall_timer(kControlPeriod, [this] { ControlTick(); });
    worker_ = std::thread([this] { WorkerLoop(); });
  }

  ~TrackerNode() override {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      stop_worker_ = true;
    }
    worker_condition_.notify_one();
    result_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void AcceptTrajectory(
      const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory& message) {
    std::string error;
    auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
    if (!trajectory.has_value()) {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        "trajectory rejected: " + error);
      return;
    }
    const rclcpp::Time start_ros(message.header.stamp);
    const rclcpp::Time now_ros = now();
    const auto lead =
        std::chrono::nanoseconds((start_ros - now_ros).nanoseconds());
    if (lead < kMinimumStartLead || lead > kMaximumStartLead) {
      PublishDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "trajectory start must be 0.25..60 seconds in the future");
      return;
    }
    auto accepted =
        std::make_shared<PolynomialTrajectory>(std::move(*trajectory));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if ((active_.has_value() &&
           active_->trajectory->id() == accepted->id()) ||
          (pending_.has_value() &&
           pending_->trajectory->id() == accepted->id())) {
        return;
      }
      pending_ = ScheduledTrajectory{accepted, SteadyClock::now() + lead};
    }
    PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                      "trajectory accepted and staged");
  }

  void ControlTick() {
    const auto current_time = SteadyClock::now();
    const bool motion_authorized = MotionIsAuthorized();
    WorkItem work;
    bool ready = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (pending_.has_value() && current_time >= pending_->start) {
        active_ = std::move(pending_);
        pending_.reset();
        fallback_started_.reset();
      }
      const bool fresh_odometry =
          has_odometry_ && current_time - odometry_time_ <= kFeedbackTimeout;
      const bool fresh_profile =
          has_motor_profile_ && current_time - motor_time_ <= kFeedbackTimeout;
      if (!active_.has_value() || current_time < active_->start ||
          !fresh_odometry || !fresh_profile || !motion_authorized) {
        PublishZero();
        return;
      }
      const double elapsed =
          std::chrono::duration<double>(current_time - active_->start).count();
      if (elapsed >= active_->trajectory->duration()) {
        active_.reset();
        fallback_started_.reset();
        PublishZero();
        PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                          "trajectory completed");
        return;
      }
      work.sequence = ++sequence_;
      work.deadline = current_time + kSolveDeadline;
      work.configuration = configuration_;
      work.trajectory = active_->trajectory;
      work.request = MpcRequest{state_, work.trajectory.get(), elapsed};
      ready = true;
    }
    if (!ready) {
      PublishZero();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_work_ = work;
    }
    worker_condition_.notify_one();

    std::optional<WorkResult> result;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      result_condition_.wait_until(lock, work.deadline, [this, &work] {
        return stop_worker_ || (worker_result_.has_value() &&
                                worker_result_->sequence == work.sequence);
      });
      if (worker_result_.has_value() &&
          worker_result_->sequence == work.sequence) {
        result = std::move(worker_result_);
        worker_result_.reset();
      }
    }
    std::uint8_t diagnostic_level{};
    std::string diagnostic_text;
    const bool post_wait_authorized = MotionIsAuthorized();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto post_wait_time = SteadyClock::now();
      const bool same_trajectory =
          active_.has_value() && active_->trajectory == work.trajectory;
      const bool fresh_odometry =
          has_odometry_ && post_wait_time - odometry_time_ <= kFeedbackTimeout;
      const bool fresh_profile =
          has_motor_profile_ &&
          post_wait_time - motor_time_ <= kFeedbackTimeout;
      const double elapsed =
          same_trajectory
              ? std::chrono::duration<double>(post_wait_time - active_->start)
                    .count()
              : 0.0;
      if (!same_trajectory || post_wait_time < active_->start ||
          !fresh_odometry || !fresh_profile || !post_wait_authorized) {
        PublishZero();
        return;
      }
      if (elapsed >= active_->trajectory->duration()) {
        active_.reset();
        fallback_started_.reset();
        PublishZero();
        diagnostic_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_text = "trajectory completed";
      } else if (result.has_value() && result->sequence == work.sequence &&
                 result->trajectory_id == work.trajectory->id() &&
                 result->command.solved && result->duration <= kSolveDeadline &&
                 result->completed <= work.deadline &&
                 post_wait_time <= work.deadline) {
        fallback_started_.reset();
        PublishCommand(EnforceCommandBounds(configuration_, result->command));
        return;
      } else {
        if (!fallback_started_.has_value()) {
          fallback_started_ = post_wait_time;
        }
        if (post_wait_time - *fallback_started_ <= kFeedbackTimeout) {
          const MpcRequest post_wait_request{state_, active_->trajectory.get(),
                                             elapsed};
          PublishCommand(FeedbackCommand(configuration_, post_wait_request));
          diagnostic_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          diagnostic_text = "ALTO deadline miss; bounded feedback active";
        } else {
          PublishZero();
          diagnostic_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          diagnostic_text = "ALTO unavailable for 100 ms; output inhibited";
        }
      }
    }
    if (!diagnostic_text.empty()) {
      PublishDiagnostic(diagnostic_level, diagnostic_text);
    }
  }

  void WorkerLoop() {
    while (true) {
      WorkItem work;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_condition_.wait(
            lock, [this] { return stop_worker_ || pending_work_.has_value(); });
        if (stop_worker_) {
          return;
        }
        work = std::move(*pending_work_);
        pending_work_.reset();
      }
      const auto started = SteadyClock::now();
      MpcCommand command;
      try {
        command = MpcSolver(work.configuration).Solve(work.request);
      } catch (const std::exception& error) {
        command.detail = std::string("ALTO exception: ") + error.what();
      }
      const auto completed = SteadyClock::now();
      WorkResult result{work.sequence, work.trajectory->id(),
                        std::move(command), completed - started, completed};
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_result_ = std::move(result);
      }
      result_condition_.notify_one();
    }
  }

  bool MotionIsAuthorized() const {
    const auto publisher_information =
        get_publishers_info_by_topic(kMotionAuthorizationTopic);
    if (publisher_information.size() != std::size_t{1} ||
        publisher_information.front().node_name() !=
            kConfigurationSupervisorName ||
        publisher_information.front().node_namespace() !=
            kConfigurationSupervisorNamespace) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::uint32_t configuration_generation =
        static_cast<std::uint32_t>(motion_authorization_ >> 32U);
    const std::uint32_t authorized_session =
        static_cast<std::uint32_t>(motion_authorization_);
    return heartbeat_ready_ && has_heartbeat_ &&
           configuration_generation != 0U && authorized_session != 0U &&
           authorized_session == heartbeat_session_id_;
  }

  void PublishCommand(const MpcCommand& command) {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = "base_footprint";
    message.twist.linear.x = command.linear_x;
    message.twist.linear.y = command.linear_y;
    message.twist.angular.z = command.angular_z;
    command_publisher_->publish(message);
  }

  void PublishZero() { PublishCommand(MpcCommand{}); }

  void PublishDiagnostic(std::uint8_t level, const std::string& text) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    const auto current_time = SteadyClock::now();
    if (text == last_diagnostic_ &&
        current_time - last_diagnostic_time_ < std::chrono::seconds(1)) {
      return;
    }
    last_diagnostic_ = text;
    last_diagnostic_time_ = current_time;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name();
    status.hardware_id = "mentor_pi";
    status.message = text;
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  MpcConfiguration configuration_;
  double wheel_radius_{};
  double mecanum_radius_sum_{};
  double configured_steering_limit_{};

  mutable std::mutex state_mutex_;
  std::array<double, 3> state_{};
  SteadyClock::time_point odometry_time_{};
  SteadyClock::time_point motor_time_{};
  bool has_odometry_{};
  bool has_motor_profile_{};
  std::uint64_t motion_authorization_{};
  std::uint32_t heartbeat_session_id_{};
  bool has_heartbeat_{};
  bool heartbeat_ready_{};
  std::optional<ScheduledTrajectory> active_;
  std::optional<ScheduledTrajectory> pending_;
  std::optional<SteadyClock::time_point> fallback_started_;
  std::uint64_t sequence_{};

  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::condition_variable result_condition_;
  std::optional<WorkItem> pending_work_;
  std::optional<WorkResult> worker_result_;
  bool stop_worker_{};
  std::thread worker_;

  std::mutex diagnostic_mutex_;
  std::string last_diagnostic_;
  SteadyClock::time_point last_diagnostic_time_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<
      mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::MotorState>::SharedPtr
      motor_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      authorization_subscription_;
  rclcpp::Subscription<mentor_pi_interfaces::msg::Heartbeat>::SharedPtr
      heartbeat_subscription_;
  rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

rclcpp::Node::SharedPtr MakeTrackerNode(VehicleType vehicle,
                                        const rclcpp::NodeOptions& options) {
  return std::make_shared<TrackerNode>(vehicle, options);
}

}  // namespace mentor_pi::tracking
