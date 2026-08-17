// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/tracker_node.hpp"

#include <algorithm>
#include <array>
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
#include "mentor_pi_tracking/tracker_plugin.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mentor_pi::tracking {
namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr auto kControlPeriod = std::chrono::nanoseconds(33'333'333);
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(100);
constexpr auto kHeartbeatTimeout = std::chrono::milliseconds(1500);
constexpr auto kControllerDeadline = std::chrono::milliseconds(25);
constexpr auto kMpcFallbackLimit = std::chrono::milliseconds(100);
constexpr auto kMinimumStartLead = std::chrono::milliseconds(250);
constexpr auto kMaximumStartLead = std::chrono::seconds(60);
constexpr double kTwoPi = 6.28318530717958647692;
constexpr char kAuthorizationTopic[] = "configuration/motion_authorization";

struct ScheduledTrajectory {
  std::shared_ptr<const PolynomialTrajectory> trajectory;
  SteadyClock::time_point start;
};

struct WorkItem {
  std::uint64_t sequence{};
  SteadyClock::time_point deadline;
  TrackerRequest request;
};

struct WorkResult {
  std::uint64_t sequence{};
  MpcCommand command;
  SteadyClock::time_point completed;
};

struct ControlSnapshot {
  std::uint64_t generation{};
  std::shared_ptr<const PolynomialTrajectory> trajectory;
  SteadyClock::time_point start;
  TrackerRequest request;
};

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

class TrackerNode final : public rclcpp::Node {
 public:
  explicit TrackerNode(const rclcpp::NodeOptions& options)
      : Node("trajectory_tracker", "/mentor_pi", options),
        loader_("mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin") {
    const std::string vehicle_name =
        declare_parameter<std::string>("vehicle_type", "mecanum");
    if (vehicle_name == "mecanum") {
      vehicle_ = VehicleType::kMecanum;
    } else if (vehicle_name == "ackermann") {
      vehicle_ = VehicleType::kAckermann;
    } else {
      throw std::invalid_argument("vehicle_type must be mecanum or ackermann");
    }

    const std::string algorithm =
        declare_parameter<std::string>("tracking_algorithm", "mpc");
    const std::string expected_plugin =
        "mentor_pi_tracking/" +
        std::string(vehicle_ == VehicleType::kMecanum ? "Mecanum"
                                                      : "Ackermann") +
        (algorithm == "mpc"    ? "Mpc"
         : algorithm == "adrc" ? "Adrc"
                               : "");
    const std::string plugin_name =
        declare_parameter<std::string>("controller_plugin", expected_plugin);
    if ((algorithm != "mpc" && algorithm != "adrc") ||
        plugin_name != expected_plugin) {
      throw std::invalid_argument(
          "controller_plugin must exactly match vehicle_type and "
          "tracking_algorithm");
    }
    algorithm_is_mpc_ = algorithm == "mpc";

    configured_.vehicle = vehicle_;
    configured_.horizon = declare_parameter<int>("horizon", 10);
    configured_.prediction_step =
        declare_parameter<double>("prediction_step", 0.1);
    configured_.wheelbase = declare_parameter<double>("wheelbase", 0.135);
    configured_.wheel_track = declare_parameter<double>("wheel_track", 0.140);
    configured_.geometry_center_offset = declare_parameter<double>(
        "rear_axle_to_geometry_center",
        vehicle_ == VehicleType::kAckermann ? 0.0675 : 0.0);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0325);
    configured_.mecanum_radius_sum =
        declare_parameter<double>("mecanum_radius_sum", 0.14);
    configured_.max_steering_angle =
        declare_parameter<double>("max_steering_angle", 0.6);
    configuration_.mpc = configured_;
    configuration_.position_adrc_input_gain =
        declare_parameter<double>("position_adrc_input_gain", 1.0);
    configuration_.position_adrc_controller_bandwidth_rad_s =
        declare_parameter<double>("position_adrc_controller_bandwidth_rad_s",
                                  1.0);
    configuration_.position_adrc_observer_bandwidth_rad_s =
        declare_parameter<double>("position_adrc_observer_bandwidth_rad_s",
                                  3.0);
    configuration_.yaw_adrc_input_gain =
        declare_parameter<double>("yaw_adrc_input_gain", 1.0);
    configuration_.yaw_adrc_controller_bandwidth_rad_s =
        declare_parameter<double>("yaw_adrc_controller_bandwidth_rad_s", 1.0);
    configuration_.yaw_adrc_observer_bandwidth_rad_s =
        declare_parameter<double>("yaw_adrc_observer_bandwidth_rad_s", 3.0);
    ValidateConfiguration();

    plugin_ = loader_.createSharedInstance(plugin_name);
    if (plugin_->vehicle() != vehicle_ || !plugin_->requires_worker()) {
      throw std::invalid_argument(
          "selected tracker plugin has an invalid execution model");
    }
    {
      std::lock_guard<std::mutex> lock(plugin_mutex_);
      plugin_->Configure(configuration_);
    }

    controller_ = "vehicle";
    command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        controller_ + "/reference", rclcpp::QoS(1).reliable());
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "diagnostics", rclcpp::QoS(10).reliable());

    safety_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions safety_options;
    safety_options.callback_group = safety_group_;
    trajectory_subscription_ = create_subscription<
        mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
        "trajectory_tracker/reference_trajectory",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory::
                   SharedPtr message) { AcceptTrajectory(*message); },
        safety_options);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        controller_ + "/odometry", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          AcceptOdometry(*message);
        },
        safety_options);
    motor_subscription_ =
        create_subscription<mentor_pi_interfaces::msg::MotorState>(
            "motors/state", rclcpp::SensorDataQoS(),
            [this](const mentor_pi_interfaces::msg::MotorState::SharedPtr
                       message) { AcceptMotor(*message); },
            safety_options);
    authorization_subscription_ = create_subscription<std_msgs::msg::UInt64>(
        kAuthorizationTopic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::UInt64::SharedPtr message) {
          AcceptAuthorization(message->data);
        },
        safety_options);
    heartbeat_subscription_ =
        create_subscription<mentor_pi_interfaces::msg::Heartbeat>(
            "heartbeat", rclcpp::QoS(1).reliable(),
            [this](
                const mentor_pi_interfaces::msg::Heartbeat::SharedPtr message) {
              AcceptHeartbeat(*message);
            },
            safety_options);
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
        "trajectory_tracker/cancel",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               const std_srvs::srv::Trigger::Response::SharedPtr response) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_.reset();
            pending_.reset();
            fallback_deadline_.reset();
            ++generation_;
            inhibited_ = true;
            last_control_time_.reset();
          }
          PublishZero();
          ResetPlugin();
          response->success = true;
          response->message = "active and pending trajectories cancelled";
        },
        rmw_qos_profile_services_default, safety_group_);

    // Deliberately use the default callback group: safety callbacks remain
    // executable while the timer is waiting up to 25 ms for controller work.
    timer_ = create_wall_timer(kControlPeriod, [this] { ControlTick(); });
    worker_ = std::thread([this] { WorkerLoop(); });
  }

  ~TrackerNode() override {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      stop_worker_ = true;
      pending_work_.reset();
    }
    worker_condition_.notify_one();
    result_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void ValidateConfiguration() const {
    if (configured_.horizon != 10 ||
        std::abs(configured_.prediction_step - 0.1) > 1.0e-12 ||
        !IsFinitePositive(configured_.wheelbase) ||
        !IsFinitePositive(configured_.wheel_track) ||
        !IsFinitePositive(wheel_radius_) ||
        !IsFinitePositive(configured_.mecanum_radius_sum) ||
        !IsFinitePositive(configured_.max_steering_angle) ||
        (vehicle_ == VehicleType::kAckermann &&
         !IsFinitePositive(configured_.geometry_center_offset))) {
      throw std::invalid_argument(
          "invalid fixed tracking geometry or MPC horizon");
    }
    const std::array<double, 6> adrc_values{
        {configuration_.position_adrc_input_gain,
         configuration_.position_adrc_controller_bandwidth_rad_s,
         configuration_.position_adrc_observer_bandwidth_rad_s,
         configuration_.yaw_adrc_input_gain,
         configuration_.yaw_adrc_controller_bandwidth_rad_s,
         configuration_.yaw_adrc_observer_bandwidth_rad_s}};
    for (const double value : adrc_values) {
      if (!IsFinitePositive(value)) {
        throw std::invalid_argument(
            "ADRC parameters must be finite and positive");
      }
    }
    if (configuration_.position_adrc_observer_bandwidth_rad_s <
            configuration_.position_adrc_controller_bandwidth_rad_s ||
        configuration_.yaw_adrc_observer_bandwidth_rad_s <
            configuration_.yaw_adrc_controller_bandwidth_rad_s) {
      throw std::invalid_argument(
          "ADRC observer bandwidth must be at least controller bandwidth");
    }
    const double nominal_period_seconds =
        std::chrono::duration<double>(kControlPeriod).count();
    if (configuration_.position_adrc_observer_bandwidth_rad_s *
                nominal_period_seconds >
            0.5 ||
        configuration_.yaw_adrc_observer_bandwidth_rad_s *
                nominal_period_seconds >
            0.5) {
      throw std::invalid_argument(
          "ADRC observer bandwidth is unstable at the 30 Hz control period");
    }
  }

  void AcceptTrajectory(
      const mentor_pi_tracking_interfaces::msg::PolynomialTrajectory& message) {
    std::string error;
    std::optional<PolynomialTrajectory> parsed =
        PolynomialTrajectory::FromMessage(message, &error);
    if (!parsed.has_value()) {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        "trajectory rejected: " + error);
      return;
    }
    const auto lead = std::chrono::nanoseconds(
        (rclcpp::Time(message.header.stamp) - now()).nanoseconds());
    if (lead < kMinimumStartLead || lead > kMaximumStartLead) {
      PublishDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "trajectory start must be 0.25..60 seconds in the future");
      return;
    }
    auto trajectory =
        std::make_shared<const PolynomialTrajectory>(std::move(*parsed));
    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if ((!active_ || active_->trajectory->id() != trajectory->id()) &&
          (!pending_ || pending_->trajectory->id() != trajectory->id())) {
        pending_ = ScheduledTrajectory{trajectory, SteadyClock::now() + lead};
        accepted = true;
      }
    }
    if (accepted) {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                        "trajectory accepted and staged");
    }
  }

  void AcceptOdometry(const nav_msgs::msg::Odometry& message) {
    const auto& position = message.pose.pose.position;
    const auto& orientation = message.pose.pose.orientation;
    const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
                                   std::hypot(orientation.z, orientation.w));
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(norm) || norm < 1.0e-9) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_odometry_ = false;
      }
      SafetyInhibit();
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        "non-finite or invalid odometry rejected");
      return;
    }
    const double qx = orientation.x / norm;
    const double qy = orientation.y / norm;
    const double qz = orientation.z / norm;
    const double qw = orientation.w / norm;
    const double raw_yaw =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (vehicle_ == VehicleType::kMecanum && has_raw_yaw_) {
        yaw_unwrapped_ += WrapAngle(raw_yaw - last_raw_yaw_);
      } else {
        yaw_unwrapped_ = raw_yaw;
      }
      last_raw_yaw_ = raw_yaw;
      has_raw_yaw_ = true;
      const double yaw =
          vehicle_ == VehicleType::kMecanum ? yaw_unwrapped_ : raw_yaw;
      state_ = {{position.x +
                     (vehicle_ == VehicleType::kAckermann
                          ? configured_.geometry_center_offset * std::cos(yaw)
                          : 0.0),
                 position.y +
                     (vehicle_ == VehicleType::kAckermann
                          ? configured_.geometry_center_offset * std::sin(yaw)
                          : 0.0),
                 yaw}};
      odometry_time_ = SteadyClock::now();
      has_odometry_ = true;
    }
  }

  void AcceptMotor(const mentor_pi_interfaces::msg::MotorState& message) {
    const auto* profile =
        mentor_pi_interfaces::FindMotorProfileContract(message.motor_model);
    if (profile == nullptr) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_motor_profile_ = false;
      }
      SafetyInhibit();
      return;
    }
    const double wheel_speed =
        std::min(6.0, static_cast<double>(profile->max_rps)) * kTwoPi *
        wheel_radius_;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      live_configuration_ = configured_;
      live_configuration_.max_linear_speed = wheel_speed;
      live_configuration_.max_lateral_speed = wheel_speed;
      live_configuration_.max_yaw_rate =
          vehicle_ == VehicleType::kMecanum
              ? wheel_speed / configured_.mecanum_radius_sum
              : wheel_speed * std::tan(configured_.max_steering_angle) /
                    configured_.wheelbase;
      motor_time_ = SteadyClock::now();
      has_motor_profile_ = true;
    }
  }

  void AcceptAuthorization(std::uint64_t authorization) {
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      motion_authorization_ = authorization;
      valid = AuthorizationStateValidLocked();
    }
    if (!valid) {
      SafetyInhibit();
    }
  }

  void AcceptHeartbeat(const mentor_pi_interfaces::msg::Heartbeat& heartbeat) {
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      heartbeat_session_id_ = heartbeat.agent_session_id;
      heartbeat_ready_ =
          heartbeat.agent_session_id != 0U &&
          (heartbeat.state == mentor_pi_interfaces::msg::Heartbeat::READY ||
           heartbeat.state == mentor_pi_interfaces::msg::Heartbeat::DEGRADED);
      has_heartbeat_ = true;
      heartbeat_time_ = SteadyClock::now();
      valid = AuthorizationStateValidLocked();
    }
    if (!valid) {
      SafetyInhibit();
    }
  }

  bool AuthorizationStateValidLocked() const {
    const std::uint32_t generation =
        static_cast<std::uint32_t>(motion_authorization_ >> 32U);
    const std::uint32_t session =
        static_cast<std::uint32_t>(motion_authorization_);
    return has_heartbeat_ && heartbeat_ready_ &&
           SteadyClock::now() - heartbeat_time_ <= kHeartbeatTimeout &&
           generation != 0U && session != 0U &&
           session == heartbeat_session_id_;
  }

  bool HasExpectedAuthorizationPublisher() const {
    const auto publishers = get_publishers_info_by_topic(kAuthorizationTopic);
    return publishers.size() == 1U &&
           publishers.front().node_name() == "configuration_supervisor" &&
           publishers.front().node_namespace() == get_namespace();
  }

  bool FreshLocked(SteadyClock::time_point now) const {
    return has_odometry_ && has_motor_profile_ &&
           now - odometry_time_ <= kFeedbackTimeout &&
           now - motor_time_ <= kFeedbackTimeout;
  }

  std::optional<ControlSnapshot> MakeSnapshot(SteadyClock::time_point now) {
    bool activated = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (pending_ && now >= pending_->start) {
        active_ = std::move(pending_);
        pending_.reset();
        fallback_deadline_.reset();
        ++generation_;
        last_control_time_.reset();
        inhibited_ = false;
        activated = true;
        if (vehicle_ == VehicleType::kMecanum && has_raw_yaw_) {
          const double reference_yaw = active_->trajectory->Evaluate(0.0).yaw;
          yaw_unwrapped_ =
              last_raw_yaw_ +
              kTwoPi * std::round((reference_yaw - last_raw_yaw_) / kTwoPi);
          state_[2] = yaw_unwrapped_;
        }
      }
    }
    if (activated) {
      ResetPlugin();
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_ || now < active_->start || !FreshLocked(now) ||
        !AuthorizationStateValidLocked()) {
      return std::nullopt;
    }
    const double elapsed =
        std::chrono::duration<double>(now - active_->start).count();
    if (elapsed >= active_->trajectory->duration()) {
      active_.reset();
      fallback_deadline_.reset();
      ++generation_;
      last_control_time_.reset();
      return std::nullopt;
    }
    const double period =
        last_control_time_.has_value()
            ? std::chrono::duration<double>(now - *last_control_time_).count()
            : std::chrono::duration<double>(kControlPeriod).count();
    last_control_time_ = now;
    inhibited_ = false;
    TrackerRequest request;
    request.trajectory = active_->trajectory;
    request.mpc = {state_, request.trajectory.get(), elapsed};
    request.live_configuration = live_configuration_;
    request.measured_period_seconds = period;
    return ControlSnapshot{generation_, active_->trajectory, active_->start,
                           std::move(request)};
  }

  bool CandidateStillValidLocked(const ControlSnapshot& snapshot,
                                 SteadyClock::time_point now,
                                 MpcConfiguration* configuration) const {
    if (inhibited_ || !active_ || active_->trajectory != snapshot.trajectory ||
        generation_ != snapshot.generation || !FreshLocked(now) ||
        !AuthorizationStateValidLocked()) {
      return false;
    }
    const double elapsed =
        std::chrono::duration<double>(now - active_->start).count();
    if (elapsed >= active_->trajectory->duration()) {
      return false;
    }
    *configuration = live_configuration_;
    return true;
  }

  bool ApplyCandidate(const ControlSnapshot& snapshot, MpcCommand command,
                      std::optional<SteadyClock::time_point> deadline,
                      bool is_fallback) {
    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    MpcConfiguration bounds;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      const auto now = SteadyClock::now();
      if (!CandidateStillValidLocked(snapshot, now, &bounds) ||
          (deadline.has_value() && now >= *deadline) ||
          !HasExpectedAuthorizationPublisher()) {
        return false;
      }
      command = EnforceCommandBounds(bounds, std::move(command));
      if (!command.solved) {
        return false;
      }
    }
    if (!algorithm_is_mpc_) {
      std::lock_guard<std::mutex> plugin_lock(plugin_mutex_);
      plugin_->SetAppliedCommand(command);
    }
    // Recheck after the controller result and any observer handoff.  A cancel,
    // replacement activation, profile change, authorization loss, or deadline
    // expiry must never allow a stale command to be published.
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto now = SteadyClock::now();
    if (!CandidateStillValidLocked(snapshot, now, &bounds) ||
        (deadline.has_value() && now >= *deadline) ||
        !HasExpectedAuthorizationPublisher()) {
      return false;
    }
    if (!is_fallback) {
      fallback_deadline_.reset();
    }
    PublishCommandLocked(command);
    return true;
  }

  void ControlTick() {
    std::unique_lock<std::mutex> tick_lock(tick_mutex_, std::try_to_lock);
    if (!tick_lock.owns_lock()) {
      return;
    }
    const auto tick_time = SteadyClock::now();
    std::optional<ControlSnapshot> snapshot = MakeSnapshot(tick_time);
    if (!snapshot.has_value() || !HasExpectedAuthorizationPublisher()) {
      SafetyInhibit();
      PublishZero();
      return;
    }

    std::optional<SteadyClock::time_point> active_fallback;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (fallback_deadline_.has_value()) {
        active_fallback = fallback_deadline_;
      }
    }
    if (active_fallback.has_value() && tick_time >= *active_fallback) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (fallback_deadline_ == active_fallback) {
          active_.reset();
          fallback_deadline_.reset();
          ++generation_;
          last_control_time_.reset();
        }
      }
      SafetyInhibit();
      PublishZero();
      return;
    }

    WorkItem work;
    work.sequence = next_sequence_++;
    work.deadline = tick_time + kControllerDeadline;
    work.request = snapshot->request;
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
    if (result.has_value() && result->command.solved &&
        result->completed <= work.deadline &&
        SteadyClock::now() <= work.deadline &&
        ApplyCandidate(*snapshot, std::move(result->command), work.deadline,
                       false)) {
      return;
    }

    if (!algorithm_is_mpc_) {
      SafetyInhibit();
      PublishZero();
      PublishDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "LADRC deadline or numerical failure; output inhibited");
      return;
    }
    const auto fallback_deadline =
        active_fallback.value_or(tick_time + kMpcFallbackLimit);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_ || active_->trajectory != snapshot->trajectory ||
          generation_ != snapshot->generation || inhibited_) {
        return;
      }
      if (!fallback_deadline_.has_value()) {
        fallback_deadline_ = fallback_deadline;
      }
    }
    const MpcCommand fallback =
        FeedbackCommand(SnapshotBounds(), snapshot->request.mpc);
    if (!fallback.solved ||
        !ApplyCandidate(*snapshot, fallback, fallback_deadline, true)) {
      SafetyInhibit();
      PublishZero();
    } else {
      PublishDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                        "ALTO deadline miss; bounded feedback active");
    }
  }

  MpcConfiguration SnapshotBounds() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return live_configuration_;
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
      MpcCommand command;
      try {
        std::lock_guard<std::mutex> lock(plugin_mutex_);
        command = plugin_->Compute(work.request);
      } catch (const std::exception& error) {
        command = {false, 0.0, 0.0, 0.0,
                   std::string("tracker exception: ") + error.what()};
      }
      const auto completed = SteadyClock::now();
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_result_ =
            WorkResult{work.sequence, std::move(command), completed};
      }
      result_condition_.notify_all();
    }
  }

  void SafetyInhibit() {
    bool reset = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!inhibited_) {
        inhibited_ = true;
        fallback_deadline_.reset();
        last_control_time_.reset();
        ++generation_;
        reset = true;
      }
    }
    if (reset) {
      // Publish before waiting for a possible 25 ms worker compute.  This is
      // why safety callbacks remain responsive when a plugin is busy.
      PublishZero();
      ResetPlugin();
    }
  }

  void ResetPlugin() {
    // MPC has no observer state.  In particular, never wait behind an overdue
    // ALTO solve just to invoke its no-op Reset during a safety transition.
    if (algorithm_is_mpc_) {
      return;
    }
    std::lock_guard<std::mutex> lock(plugin_mutex_);
    plugin_->Reset();
  }

  void PublishZero() {
    std::lock_guard<std::mutex> lock(publication_mutex_);
    PublishCommandLocked({});
  }

  void PublishCommandLocked(const MpcCommand& command) {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = vehicle_ == VehicleType::kAckermann
                                  ? "rear_axle_footprint"
                                  : "base_footprint";
    message.twist.linear.x = command.linear_x;
    message.twist.linear.y = command.linear_y;
    message.twist.angular.z = command.angular_z;
    command_publisher_->publish(message);
  }

  void PublishDiagnostic(std::uint8_t level, const std::string& text) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    const auto current = SteadyClock::now();
    if (text == last_diagnostic_ &&
        current - last_diagnostic_time_ < std::chrono::seconds(1)) {
      return;
    }
    last_diagnostic_ = text;
    last_diagnostic_time_ = current;
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

  VehicleType vehicle_{};
  bool algorithm_is_mpc_{};
  std::string controller_;
  MpcConfiguration configured_;
  TrackerConfiguration configuration_;
  double wheel_radius_{};
  pluginlib::ClassLoader<TrackerPlugin> loader_;
  std::shared_ptr<TrackerPlugin> plugin_;

  mutable std::mutex state_mutex_;
  std::array<double, 3> state_{};
  MpcConfiguration live_configuration_{};
  SteadyClock::time_point odometry_time_{};
  SteadyClock::time_point motor_time_{};
  bool has_odometry_{};
  bool has_motor_profile_{};
  bool has_raw_yaw_{};
  double last_raw_yaw_{};
  double yaw_unwrapped_{};
  std::uint64_t motion_authorization_{};
  std::uint32_t heartbeat_session_id_{};
  SteadyClock::time_point heartbeat_time_{};
  bool has_heartbeat_{};
  bool heartbeat_ready_{};
  bool inhibited_{true};
  std::optional<ScheduledTrajectory> active_;
  std::optional<ScheduledTrajectory> pending_;
  std::optional<SteadyClock::time_point> fallback_deadline_;
  std::optional<SteadyClock::time_point> last_control_time_;
  std::uint64_t generation_{};
  std::uint64_t next_sequence_{};

  std::mutex plugin_mutex_;
  std::mutex publication_mutex_;
  std::mutex tick_mutex_;
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
  rclcpp::CallbackGroup::SharedPtr safety_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

rclcpp::Node::SharedPtr MakeTrackerNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<TrackerNode>(options);
}

}  // namespace mentor_pi::tracking
