// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mentor_pi_interfaces/motor_profile_contract.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

extern char** environ;

namespace {

using namespace std::chrono_literals;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using Result = mentor_pi_interfaces::msg::Result;
using SetBatteryThreshold = mentor_pi_interfaces::srv::SetBatteryThreshold;
using SetMotorModel = mentor_pi_interfaces::srv::SetMotorModel;
using SetPwmServoOffsets = mentor_pi_interfaces::srv::SetPwmServoOffsets;

enum class Operation {
  kMotorModel,
  kPwmOffsets,
  kBatteryThreshold,
};

constexpr std::array<std::int16_t, 4> kExpectedOffsets{-100, -50, 50, 100};
constexpr std::uint16_t kExpectedBatteryThresholdMv = 9000;
constexpr auto kNativeLaunchDeadline = 10s;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

class ScopedLaunchProcess {
 public:
  ScopedLaunchProcess() = default;
  ScopedLaunchProcess(const ScopedLaunchProcess&) = delete;
  ScopedLaunchProcess& operator=(const ScopedLaunchProcess&) = delete;

  ~ScopedLaunchProcess() { Stop(); }

  bool Start(std::vector<std::string> arguments) {
    if (arguments.empty()) {
      return false;
    }

    std::vector<char*> argument_pointers;
    argument_pointers.reserve(arguments.size() + 1U);
    for (std::string& argument : arguments) {
      argument_pointers.push_back(argument.data());
    }
    argument_pointers.push_back(nullptr);

    posix_spawnattr_t attributes;
    int result = posix_spawnattr_init(&attributes);
    if (result != 0) {
      std::cerr << "posix_spawnattr_init failed: " << std::strerror(result)
                << '\n';
      return false;
    }

    result = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    if (result == 0) {
      result = posix_spawnattr_setpgroup(&attributes, 0);
    }
    if (result == 0) {
      result = posix_spawnp(&process_id_, argument_pointers[0], nullptr,
                            &attributes, argument_pointers.data(), environ);
    }
    const int destroy_result = posix_spawnattr_destroy(&attributes);
    if (destroy_result != 0) {
      std::cerr << "posix_spawnattr_destroy failed: "
                << std::strerror(destroy_result) << '\n';
    }
    if (result != 0) {
      std::cerr << "could not start ros2 launch: " << std::strerror(result)
                << '\n';
      process_id_ = -1;
      return false;
    }
    return true;
  }

  bool IsRunning() {
    if (process_id_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = waitpid(process_id_, &status, WNOHANG);
    if (wait_result == 0) {
      return true;
    }
    if (wait_result == process_id_ || (wait_result < 0 && errno == ECHILD)) {
      process_id_ = -1;
      return false;
    }
    return wait_result < 0 && errno == EINTR;
  }

  void Stop() {
    if (process_id_ <= 0) {
      return;
    }
    static_cast<void>(kill(-process_id_, SIGINT));
    if (WaitForExit(3s)) {
      return;
    }
    static_cast<void>(kill(-process_id_, SIGTERM));
    if (WaitForExit(1s)) {
      return;
    }
    static_cast<void>(kill(-process_id_, SIGKILL));
    static_cast<void>(WaitForExit(1s));
  }

 private:
  bool WaitForExit(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!IsRunning()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return !IsRunning();
  }

  pid_t process_id_ = -1;
};

class ControllerPeer {
 public:
  ControllerPeer()
      : node_(std::make_shared<rclcpp::Node>("launch_controller_peer",
                                             "/mentor_pi")) {
    const auto reliable_depth_one =
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .durability_volatile();
    heartbeat_publisher_ =
        node_->create_publisher<Heartbeat>("heartbeat", reliable_depth_one);
    gate_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        "configuration/motion_enabled",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          gate_events_.push_back(message->data);
        });

    motor_service_ = node_->create_service<SetMotorModel>(
        "motors/set_model",
        [this](const std::shared_ptr<SetMotorModel::Request> request,
               std::shared_ptr<SetMotorModel::Response> response) {
          operations_.push_back(Operation::kMotorModel);
          Expect(request->model == SetMotorModel::Request::MODEL_JGB37,
                 "launched supervisor loads the YAML motor model");
          const auto* profile =
              mentor_pi_interfaces::FindMotorProfileContract(request->model);
          Expect(profile != nullptr,
                 "launch peer receives a supported motor model");
          if (profile == nullptr) {
            response->result.code = Result::INVALID_ARGUMENT;
            return;
          }
          response->result.code = Result::OK;
          response->active_model = request->model;
          response->ticks_per_revolution = profile->ticks_per_revolution;
          response->max_rps = profile->max_rps;
        });
    pwm_service_ = node_->create_service<SetPwmServoOffsets>(
        "pwm_servos/set_offsets",
        [this](const std::shared_ptr<SetPwmServoOffsets::Request> request,
               std::shared_ptr<SetPwmServoOffsets::Response> response) {
          operations_.push_back(Operation::kPwmOffsets);
          Expect(
              request->update_mask == SetPwmServoOffsets::Request::ALL_SERVOS,
              "launched supervisor sets all PWM offsets");
          Expect(request->offset_us == kExpectedOffsets,
                 "launched supervisor loads the YAML PWM offsets");
          response->result.code = Result::OK;
          response->applied_mask = request->update_mask;
        });
    battery_service_ = node_->create_service<SetBatteryThreshold>(
        "battery/set_low_threshold",
        [this](const std::shared_ptr<SetBatteryThreshold::Request> request,
               std::shared_ptr<SetBatteryThreshold::Response> response) {
          operations_.push_back(Operation::kBatteryThreshold);
          Expect(request->threshold_mv == kExpectedBatteryThresholdMv,
                 "launched supervisor loads the YAML battery threshold");
          response->result.code = Result::OK;
          response->active_threshold_mv = request->threshold_mv;
        });
  }

  void PublishHeartbeat() {
    Heartbeat heartbeat;
    heartbeat.sequence = sequence_++;
    heartbeat.uptime_ms = sequence_ * 10U;
    heartbeat.agent_session_id = 42;
    heartbeat.state = Heartbeat::READY;
    heartbeat_publisher_->publish(heartbeat);
  }

  bool ready() const {
    return operations_.size() >= 3U && !gate_events_.empty() &&
           gate_events_.back();
  }

  const std::shared_ptr<rclcpp::Node>& node() const { return node_; }
  const std::vector<Operation>& operations() const { return operations_; }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_subscription_;
  rclcpp::Service<SetMotorModel>::SharedPtr motor_service_;
  rclcpp::Service<SetPwmServoOffsets>::SharedPtr pwm_service_;
  rclcpp::Service<SetBatteryThreshold>::SharedPtr battery_service_;
  std::vector<Operation> operations_;
  std::vector<bool> gate_events_;
  std::uint32_t sequence_ = 0;
};

bool MarkerIsReady(const std::string& marker_path) {
  std::ifstream marker{marker_path};
  std::string content;
  return marker.is_open() && std::getline(marker, content) &&
         content == "READY";
}

void ExpectOperation(const std::vector<Operation>& operations,
                     std::size_t index, Operation expected,
                     const std::string& description) {
  Expect(index < operations.size(), description + ": operation is present");
  if (index < operations.size()) {
    Expect(operations[index] == expected,
           description + ": operation order is correct");
  }
}

bool SetEnvironment(const char* name, const std::string& value) {
  if (setenv(name, value.c_str(), 1) == 0) {
    return true;
  }
  std::cerr << "could not set " << name << ": " << std::strerror(errno) << '\n';
  return false;
}

bool IsQemuEmulatedTest() {
  const char* emulated = std::getenv("RRCLITE_QEMU_EMULATED_TESTS");
  return emulated != nullptr && std::string{emulated} == "1";
}

bool WriteExecutable(const std::string& path, const std::string& content) {
  std::ofstream output{path};
  if (!output.is_open()) {
    std::cerr << "could not create " << path << '\n';
    return false;
  }
  output << content;
  output.close();
  if (!output || chmod(path.c_str(), 0700) != 0) {
    std::cerr << "could not prepare " << path << ": " << std::strerror(errno)
              << '\n';
    return false;
  }
  return true;
}

void RunLaunchTest(const std::string& fake_agent_path,
                   const std::string& configuration_path) {
  const std::string process_suffix = std::to_string(getpid());
  const std::string marker_path =
      "/tmp/mentor_pi_fake_agent_" + process_suffix + ".marker";
  const std::string tool_root =
      "/tmp/mentor_pi_launch_tools_" + process_suffix;
  const std::string fake_udevadm = tool_root + "/udevadm";
  const std::string fake_fuser = tool_root + "/fuser";
  const std::string serial_device = "/dev/null";
  const std::string domain_id =
      std::to_string(100 + static_cast<int>(getpid() % 100));
  static_cast<void>(std::remove(marker_path.c_str()));

  if (mkdir(tool_root.c_str(), 0700) != 0 ||
      !WriteExecutable(fake_udevadm,
                       "#!/bin/sh\n"
                       "printf 'ID_VENDOR_ID=1a86\\nID_MODEL_ID=55d4\\n'\n") ||
      !WriteExecutable(fake_fuser, "#!/bin/sh\nexit 1\n")) {
    ++g_failures;
    return;
  }
  const char* current_path = std::getenv("PATH");
  const std::string test_path =
      tool_root + ":" + (current_path != nullptr ? current_path : "");

  if (!SetEnvironment("ROS_DOMAIN_ID", domain_id) ||
      !SetEnvironment("RRCLITE_RUNTIME_ACK",
                      "PID_FIRMWARE_ACTUATORS_PREPARED") ||
      !SetEnvironment("PATH", test_path) ||
      !SetEnvironment("MENTOR_PI_FAKE_AGENT_MARKER", marker_path) ||
      !SetEnvironment("MENTOR_PI_FAKE_AGENT_EXPECTED_DEVICE", serial_device)) {
    ++g_failures;
    return;
  }

  rclcpp::init(0, nullptr);
  ControllerPeer controller;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.node());

  ScopedLaunchProcess launch;
  const bool started = launch.Start({"ros2", "launch", "mentor_pi_bringup",
                                     "controller.launch.py",
                                     "agent_executable:=" + fake_agent_path,
                                     "serial_device:=" + serial_device,
                                     "config_file:=" + configuration_path});
  Expect(started, "the checked-in Python launch description starts");

  bool configured = false;
  if (started) {
    const bool qemu_emulated = IsQemuEmulatedTest();
    const auto native_deadline =
        std::chrono::steady_clock::now() + kNativeLaunchDeadline;
    while (launch.IsRunning() &&
           (qemu_emulated ||
            std::chrono::steady_clock::now() < native_deadline)) {
      controller.PublishHeartbeat();
      executor.spin_some();
      if (controller.ready() && MarkerIsReady(marker_path)) {
        configured = true;
        break;
      }
      std::this_thread::sleep_for(5ms);
    }
  }

  Expect(configured,
         "external supervisor configures through the C++ controller peer");
  Expect(MarkerIsReady(marker_path),
         "launch passes the exact serial Agent command line");
  Expect(controller.operations().size() == 3U,
         "launch applies exactly three configuration services");
  ExpectOperation(controller.operations(), 0, Operation::kMotorModel,
                  "launch motor model");
  ExpectOperation(controller.operations(), 1, Operation::kPwmOffsets,
                  "launch PWM offsets");
  ExpectOperation(controller.operations(), 2, Operation::kBatteryThreshold,
                  "launch battery threshold");

  launch.Stop();
  executor.remove_node(controller.node());
  rclcpp::shutdown();
  static_cast<void>(std::remove(marker_path.c_str()));
  static_cast<void>(std::remove(fake_udevadm.c_str()));
  static_cast<void>(std::remove(fake_fuser.c_str()));
  static_cast<void>(rmdir(tool_root.c_str()));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: configuration_supervisor_launch_test "
                 "FAKE_AGENT CONFIGURATION\n";
    return 2;
  }
  RunLaunchTest(argv[1], argv[2]);
  if (g_failures == 0) {
    std::cout << "configuration supervisor process launch test passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
