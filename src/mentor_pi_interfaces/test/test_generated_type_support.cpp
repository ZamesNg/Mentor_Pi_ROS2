// Copyright 2026 Mentor Pi maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "mentor_pi_interfaces/msg/battery_state.hpp"
#include "mentor_pi_interfaces/msg/bus_servo_command.hpp"
#include "mentor_pi_interfaces/msg/bus_servo_state.hpp"
#include "mentor_pi_interfaces/msg/button_event.hpp"
#include "mentor_pi_interfaces/msg/buzzer_command.hpp"
#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/imu_state.hpp"
#include "mentor_pi_interfaces/msg/led_command.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_interfaces/msg/oled_command.hpp"
#include "mentor_pi_interfaces/msg/pwm_servo_command.hpp"
#include "mentor_pi_interfaces/msg/pwm_servo_state.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/msg/rgb_command.hpp"
#include "mentor_pi_interfaces/srv/configure_bus_servo.hpp"
#include "mentor_pi_interfaces/srv/get_bus_servo_state.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "mentor_pi_interfaces/srv/stop_bus_servos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace mentor_pi_interfaces::test {
namespace {

constexpr std::size_t kCdrEncapsulationBytes = 4;
constexpr std::size_t kMaximumXrceSampleBytes = 512;
constexpr std::size_t kMaximumOledLineBytes = 23;

enum class Boundary {
  kLower,
  kUpper,
};

template <typename T>
T NumericBoundary(Boundary boundary) {
  static_assert(std::is_arithmetic_v<T>);
  if constexpr (std::is_same_v<T, bool>) {
    return boundary == Boundary::kUpper;
  }
  if (boundary == Boundary::kLower) {
    if constexpr (std::is_floating_point_v<T>) {
      return std::numeric_limits<T>::lowest();
    }
    return std::numeric_limits<T>::min();
  }
  return std::numeric_limits<T>::max();
}

template <typename T, std::size_t kSize>
void SetArrayBoundary(std::array<T, kSize>* values, Boundary boundary) {
  values->fill(NumericBoundary<T>(boundary));
}

void SetBoundaryFields(builtin_interfaces::msg::Time* message,
                       Boundary boundary) {
  message->sec = NumericBoundary<int32_t>(boundary);
  message->nanosec = boundary == Boundary::kLower ? 0U : 999'999'999U;
}

void SetBoundaryFields(msg::BatteryState* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  message->voltage_mv = NumericBoundary<uint16_t>(boundary);
  message->low_threshold_mv = NumericBoundary<uint16_t>(boundary);
  message->valid = NumericBoundary<bool>(boundary);
  message->below_threshold = NumericBoundary<bool>(boundary);
}

void SetBoundaryFields(msg::BusServoCommand* message, Boundary boundary) {
  message->count = boundary == Boundary::kLower ? 1U : 16U;
  SetArrayBoundary(&message->servo_id, boundary);
  SetArrayBoundary(&message->position, boundary);
  message->duration_ms = NumericBoundary<uint16_t>(boundary);
}

void SetBoundaryFields(msg::BusServoState* message, Boundary boundary) {
  message->valid_fields = boundary == Boundary::kLower
                              ? msg::BusServoState::FIELD_ID
                              : msg::BusServoState::ALL_FIELDS;
  message->requested_id = NumericBoundary<uint8_t>(boundary);
  message->reported_id = NumericBoundary<uint8_t>(boundary);
  message->position = NumericBoundary<int16_t>(boundary);
  message->offset = NumericBoundary<int8_t>(boundary);
  message->voltage_mv = NumericBoundary<uint16_t>(boundary);
  message->temperature_c = NumericBoundary<uint8_t>(boundary);
  message->position_min = NumericBoundary<uint16_t>(boundary);
  message->position_max = NumericBoundary<uint16_t>(boundary);
  message->voltage_min_mv = NumericBoundary<uint16_t>(boundary);
  message->voltage_max_mv = NumericBoundary<uint16_t>(boundary);
  message->temperature_limit_c = NumericBoundary<uint8_t>(boundary);
  message->torque_enabled = NumericBoundary<bool>(boundary);
}

void SetBoundaryFields(msg::ButtonEvent* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  message->button_id = boundary == Boundary::kLower ? 1U : 2U;
  message->event = boundary == Boundary::kLower
                       ? msg::ButtonEvent::PRESSED
                       : msg::ButtonEvent::TRIPLE_CLICK;
}

void SetBoundaryFields(msg::BuzzerCommand* message, Boundary boundary) {
  message->frequency_hz = NumericBoundary<uint16_t>(boundary);
  message->on_time_ms = NumericBoundary<uint16_t>(boundary);
  message->off_time_ms = NumericBoundary<uint16_t>(boundary);
  message->repeat = NumericBoundary<uint16_t>(boundary);
}

void SetBoundaryFields(msg::ControllerDiagnostics* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  const uint64_t uint64_value = NumericBoundary<uint64_t>(boundary);
  const uint32_t uint32_value = NumericBoundary<uint32_t>(boundary);
  const uint16_t uint16_value = NumericBoundary<uint16_t>(boundary);

  message->transport_rx_bytes = uint64_value;
  message->transport_tx_bytes = uint64_value;
  message->uptime_ms = uint32_value;
  message->session_generation = uint32_value;
  message->agent_reconnects = uint32_value;
  message->command_messages = uint32_value;
  message->command_rejections = uint32_value;
  SetArrayBoundary(&message->mailbox_overwrites, boundary);
  message->button_event_drops = uint32_value;
  message->publication_errors = uint32_value;
  message->service_requests = uint32_value;
  message->service_completions = uint32_value;
  message->service_busy_rejections = uint32_value;
  message->service_timeouts = uint32_value;
  message->service_partial_results = uint32_value;
  message->late_response_drops = uint32_value;
  SetArrayBoundary(&message->motor_lease_expiries, boundary);
  SetArrayBoundary(&message->motor_command_rejections, boundary);
  message->motor_watchdog_trips = uint32_value;
  message->motor_command_consumptions = uint32_value;
  message->motor_command_age_over_20_ms = uint32_value;
  message->motor_command_max_age_us = uint32_value;
  message->executor_overruns = uint32_value;
  SetArrayBoundary(&message->peripheral_errors, boundary);
  SetArrayBoundary(&message->peripheral_timeouts, boundary);
  SetArrayBoundary(&message->usart1_errors, boundary);
  message->usart1_rx_dma_high_water_bytes = uint32_value;
  message->transport_rx_overruns = uint32_value;
  message->transport_tx_timeouts = uint32_value;
  message->maximum_transport_wait_us = uint32_value;
  SetArrayBoundary(&message->task_missed_releases, boundary);
  SetArrayBoundary(&message->task_max_execution_us, boundary);
  SetArrayBoundary(&message->task_stack_high_water_bytes, boundary);
  SetArrayBoundary(&message->task_heartbeat_age_ms, boundary);
  SetArrayBoundary(&message->free_ram_bytes, boundary);
  SetArrayBoundary(&message->minimum_free_ram_bytes, boundary);
  message->flash_used_bytes = uint32_value;
  message->flash_total_bytes = uint32_value;
  message->post_seal_allocation_attempts = uint32_value;
  message->last_error_uptime_ms = uint32_value;
  message->last_error_detail = uint16_value;
  message->session_state = boundary == Boundary::kLower
                               ? msg::ControllerDiagnostics::SESSION_SAFE_BOOT
                               : msg::ControllerDiagnostics::SESSION_BACKOFF;
  message->last_teardown_reason =
      boundary == Boundary::kLower
          ? msg::ControllerDiagnostics::TEARDOWN_NONE
          : msg::ControllerDiagnostics::TEARDOWN_TASK_STALL;
  message->last_reset_reason = boundary == Boundary::kLower
                                   ? msg::ControllerDiagnostics::RESET_POWER_ON
                                   : msg::ControllerDiagnostics::RESET_UNKNOWN;
  message->last_watchdog_task =
      boundary == Boundary::kLower
          ? msg::ControllerDiagnostics::TASK_SAFETY_SUPERVISOR
          : msg::ControllerDiagnostics::TASK_NONE;
  message->last_error_code =
      boundary == Boundary::kLower ? msg::Result::OK : msg::Result::PARTIAL;
  message->last_error_source = boundary == Boundary::kLower
                                   ? msg::ControllerDiagnostics::SOURCE_NONE
                                   : msg::ControllerDiagnostics::SOURCE_MEMORY;
}

void SetBoundaryFields(msg::Heartbeat* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  message->sequence = NumericBoundary<uint32_t>(boundary);
  message->uptime_ms = NumericBoundary<uint32_t>(boundary);
  message->agent_session_id = NumericBoundary<uint32_t>(boundary);
  message->state = boundary == Boundary::kLower ? msg::Heartbeat::BOOTING
                                                : msg::Heartbeat::FAULT;
  message->flags =
      boundary == Boundary::kLower
          ? 0U
          : static_cast<uint16_t>(msg::Heartbeat::TIME_SYNCHRONIZED |
                                  msg::Heartbeat::MOTOR_WATCHDOG_ACTIVE |
                                  msg::Heartbeat::LOW_BATTERY |
                                  msg::Heartbeat::IMU_HEALTHY |
                                  msg::Heartbeat::BUS_SERVO_BUSY);
}

void SetBoundaryFields(msg::ImuState* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  SetArrayBoundary(&message->angular_velocity_rad_s, boundary);
  SetArrayBoundary(&message->linear_acceleration_m_s2, boundary);
  message->valid = NumericBoundary<bool>(boundary);
}

void SetBoundaryFields(msg::LedCommand* message, Boundary boundary) {
  message->led_id = boundary == Boundary::kLower ? 1U : 3U;
  message->on_time_ms = NumericBoundary<uint16_t>(boundary);
  message->off_time_ms = NumericBoundary<uint16_t>(boundary);
  message->repeat = NumericBoundary<uint16_t>(boundary);
}

void SetBoundaryFields(msg::MotorCommand* message, Boundary boundary) {
  message->update_mask = boundary == Boundary::kLower
                             ? msg::MotorCommand::MOTOR_1
                             : msg::MotorCommand::ALL_MOTORS;
  SetArrayBoundary(&message->target_rps, boundary);
}

void SetBoundaryFields(msg::MotorState* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  SetArrayBoundary(&message->target_rps, boundary);
  SetArrayBoundary(&message->measured_rps, boundary);
  SetArrayBoundary(&message->encoder_count, boundary);
  message->motor_model = boundary == Boundary::kLower
                             ? msg::MotorState::MODEL_JGB520
                             : msg::MotorState::MODEL_JGB528;
  message->watchdog_stop_mask =
      boundary == Boundary::kLower ? 0U : msg::MotorCommand::ALL_MOTORS;
}

void SetBoundaryFields(msg::OledCommand* message, Boundary boundary) {
  message->update_mask = boundary == Boundary::kLower
                             ? msg::OledCommand::LINE_1
                             : msg::OledCommand::ALL_LINES;
  const std::string line = boundary == Boundary::kLower
                               ? std::string()
                               : std::string(kMaximumOledLineBytes, '~');
  message->line_1 = line;
  message->line_2 = line;
}

void SetBoundaryFields(msg::PwmServoCommand* message, Boundary boundary) {
  message->update_mask = boundary == Boundary::kLower
                             ? msg::PwmServoCommand::SERVO_1
                             : msg::PwmServoCommand::ALL_SERVOS;
  message->duration_ms = boundary == Boundary::kLower ? 20U : 30'000U;
  message->pulse_width_us.fill(boundary == Boundary::kLower ? 500U : 2'500U);
}

void SetBoundaryFields(msg::PwmServoState* message, Boundary boundary) {
  SetBoundaryFields(&message->stamp, boundary);
  SetArrayBoundary(&message->target_pulse_width_us, boundary);
  SetArrayBoundary(&message->output_pulse_width_us, boundary);
  SetArrayBoundary(&message->offset_us, boundary);
  message->moving_mask =
      boundary == Boundary::kLower ? 0U : msg::PwmServoCommand::ALL_SERVOS;
}

void SetBoundaryFields(msg::Result* message, Boundary boundary) {
  message->code =
      boundary == Boundary::kLower ? msg::Result::OK : msg::Result::PARTIAL;
  message->detail = NumericBoundary<uint16_t>(boundary);
}

void SetBoundaryFields(msg::RgbCommand* message, Boundary boundary) {
  message->update_mask = boundary == Boundary::kLower
                             ? msg::RgbCommand::PIXEL_1
                             : msg::RgbCommand::ALL_PIXELS;
  SetArrayBoundary(&message->red, boundary);
  SetArrayBoundary(&message->green, boundary);
  SetArrayBoundary(&message->blue, boundary);
}

void SetBoundaryFields(srv::ConfigureBusServo::Request* message,
                       Boundary boundary) {
  message->servo_id = boundary == Boundary::kLower ? 1U : 253U;
  message->update_mask = boundary == Boundary::kLower
                             ? srv::ConfigureBusServo::Request::SET_ID
                             : srv::ConfigureBusServo::Request::ALL_UPDATES;
  message->new_id = boundary == Boundary::kLower ? 1U : 253U;
  message->offset = boundary == Boundary::kLower ? -125 : 125;
  message->position_min = boundary == Boundary::kLower ? 0U : 1'000U;
  message->position_max = boundary == Boundary::kLower ? 0U : 1'000U;
  message->voltage_min_mv = boundary == Boundary::kLower ? 4'500U : 14'000U;
  message->voltage_max_mv = boundary == Boundary::kLower ? 4'500U : 14'000U;
  message->temperature_limit_c = boundary == Boundary::kLower ? 0U : 100U;
  message->torque_enabled = NumericBoundary<bool>(boundary);
}

void SetBoundaryFields(srv::ConfigureBusServo::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  message->applied_mask = boundary == Boundary::kLower
                              ? 0U
                              : srv::ConfigureBusServo::Request::ALL_UPDATES;
  message->effective_id = boundary == Boundary::kLower ? 1U : 253U;
}

void SetBoundaryFields(srv::GetBusServoState::Request* message,
                       Boundary boundary) {
  message->servo_id = boundary == Boundary::kLower ? 1U : 254U;
  message->fields = boundary == Boundary::kLower
                        ? srv::GetBusServoState::Request::FIELD_ID
                        : srv::GetBusServoState::Request::ALL_FIELDS;
}

void SetBoundaryFields(srv::GetBusServoState::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  SetBoundaryFields(&message->state, boundary);
}

void SetBoundaryFields(srv::SetBatteryThreshold::Request* message,
                       Boundary boundary) {
  message->threshold_mv = boundary == Boundary::kLower ? 5'000U : 20'000U;
}

void SetBoundaryFields(srv::SetBatteryThreshold::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  message->active_threshold_mv =
      boundary == Boundary::kLower ? 5'000U : 20'000U;
}

void SetBoundaryFields(srv::SetMotorModel::Request* message,
                       Boundary boundary) {
  message->model = boundary == Boundary::kLower
                       ? srv::SetMotorModel::Request::MODEL_JGB520
                       : srv::SetMotorModel::Request::MODEL_JGB528;
}

void SetBoundaryFields(srv::SetMotorModel::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  message->active_model = boundary == Boundary::kLower
                              ? srv::SetMotorModel::Request::MODEL_JGB520
                              : srv::SetMotorModel::Request::MODEL_JGB528;
  message->ticks_per_revolution = NumericBoundary<uint32_t>(boundary);
  message->max_rps = NumericBoundary<float>(boundary);
}

void SetBoundaryFields(srv::SetPwmServoOffsets::Request* message,
                       Boundary boundary) {
  message->update_mask = boundary == Boundary::kLower
                             ? srv::SetPwmServoOffsets::Request::SERVO_1
                             : srv::SetPwmServoOffsets::Request::ALL_SERVOS;
  message->offset_us.fill(boundary == Boundary::kLower ? -100 : 100);
}

void SetBoundaryFields(srv::SetPwmServoOffsets::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  message->applied_mask = boundary == Boundary::kLower
                              ? 0U
                              : srv::SetPwmServoOffsets::Request::ALL_SERVOS;
}

void SetBoundaryFields(srv::StopBusServos::Request* message,
                       Boundary boundary) {
  message->count = boundary == Boundary::kLower ? 1U : 16U;
  message->servo_id.fill(boundary == Boundary::kLower ? 1U : 253U);
}

void SetBoundaryFields(srv::StopBusServos::Response* message,
                       Boundary boundary) {
  SetBoundaryFields(&message->result, boundary);
  message->commands_transmitted = boundary == Boundary::kLower ? 0U : 16U;
}

template <typename MessageT>
rclcpp::SerializedMessage Serialize(const MessageT& input) {
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&input, &serialized);
  return serialized;
}

template <typename MessageT>
void ExpectRoundTrip(const MessageT& input) {
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  ASSERT_NO_THROW(serialization.serialize_message(&input, &serialized));
  EXPECT_LE(serialized.size(), kMaximumXrceSampleBytes);

  MessageT output;
  ASSERT_NO_THROW(serialization.deserialize_message(&serialized, &output));
  EXPECT_EQ(input, output);
}

template <typename MessageT>
void ExpectDefaultAndBoundaryRoundTrips() {
  static_assert(rosidl_generator_traits::has_bounded_size<MessageT>::value);

  {
    SCOPED_TRACE("default/zero sample");
    ExpectRoundTrip(MessageT{});
  }
  for (const Boundary boundary : {Boundary::kLower, Boundary::kUpper}) {
    SCOPED_TRACE(boundary == Boundary::kLower ? "lower boundary"
                                              : "upper boundary");
    MessageT message;
    SetBoundaryFields(&message, boundary);
    ExpectRoundTrip(message);
  }
}

using GeneratedMessageTypes =
    ::testing::Types<msg::BatteryState, msg::BusServoCommand,
                     msg::BusServoState, msg::ButtonEvent, msg::BuzzerCommand,
                     msg::ControllerDiagnostics, msg::Heartbeat, msg::ImuState,
                     msg::LedCommand, msg::MotorCommand, msg::MotorState,
                     msg::OledCommand, msg::PwmServoCommand, msg::PwmServoState,
                     msg::Result, msg::RgbCommand>;

template <typename MessageT>
class GeneratedMessageCdrRoundTripTest : public ::testing::Test {};

TYPED_TEST_SUITE(GeneratedMessageCdrRoundTripTest, GeneratedMessageTypes);

TYPED_TEST(GeneratedMessageCdrRoundTripTest,
           DefaultAndIdlBoundariesRoundTripThroughGeneratedRmwCdr) {
  ExpectDefaultAndBoundaryRoundTrips<TypeParam>();
}

using GeneratedServicePartTypes = ::testing::Types<
    srv::ConfigureBusServo::Request, srv::ConfigureBusServo::Response,
    srv::GetBusServoState::Request, srv::GetBusServoState::Response,
    srv::SetBatteryThreshold::Request, srv::SetBatteryThreshold::Response,
    srv::SetMotorModel::Request, srv::SetMotorModel::Response,
    srv::SetPwmServoOffsets::Request, srv::SetPwmServoOffsets::Response,
    srv::StopBusServos::Request, srv::StopBusServos::Response>;

template <typename MessageT>
class GeneratedServicePartCdrRoundTripTest : public ::testing::Test {};

TYPED_TEST_SUITE(GeneratedServicePartCdrRoundTripTest,
                 GeneratedServicePartTypes);

TYPED_TEST(GeneratedServicePartCdrRoundTripTest,
           DefaultAndIdlBoundariesRoundTripThroughGeneratedRmwCdr) {
  ExpectDefaultAndBoundaryRoundTrips<TypeParam>();
}

TEST(GeneratedTypeSupportTest,
     ControllerDiagnosticsIsExactly392BytesIncludingCdrEncapsulation) {
  std::array<msg::ControllerDiagnostics, 3> samples;
  SetBoundaryFields(&samples[1], Boundary::kLower);
  SetBoundaryFields(&samples[2], Boundary::kUpper);
  for (const msg::ControllerDiagnostics& input : samples) {
    rclcpp::SerializedMessage serialized;
    ASSERT_NO_THROW(serialized = Serialize(input));

    ASSERT_EQ(392U, serialized.size());
    const auto& rmw_message = serialized.get_rcl_serialized_message();
    ASSERT_NE(nullptr, rmw_message.buffer);
    ASSERT_GE(rmw_message.buffer_length, kCdrEncapsulationBytes);
    EXPECT_EQ(0x00U, rmw_message.buffer[0]);
    EXPECT_EQ(0x01U, rmw_message.buffer[1]);
    EXPECT_EQ(0x00U, rmw_message.buffer[2]);
    EXPECT_EQ(0x00U, rmw_message.buffer[3]);
  }
}

TEST(GeneratedTypeSupportTest,
     OledMaximumBoundedStringsRoundTripAndBoundsAreGenerated) {
  msg::OledCommand maximum;
  SetBoundaryFields(&maximum, Boundary::kUpper);
  ASSERT_EQ(kMaximumOledLineBytes, maximum.line_1.size());
  ASSERT_EQ(kMaximumOledLineBytes, maximum.line_2.size());
  ExpectRoundTrip(maximum);

  const rosidl_message_type_support_t* generic_type_support =
      rosidl_typesupport_cpp::get_message_type_support_handle<
          msg::OledCommand>();
  ASSERT_NE(nullptr, generic_type_support);
  const rosidl_message_type_support_t* introspection_type_support =
      rosidl_typesupport_cpp::get_message_typesupport_handle_function(
          generic_type_support,
          rosidl_typesupport_introspection_cpp::typesupport_identifier);
  ASSERT_NE(nullptr, introspection_type_support);
  const auto* members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
          introspection_type_support->data);
  ASSERT_NE(nullptr, members);
  ASSERT_EQ(3U, members->member_count_);
  ASSERT_STREQ("line_1", members->members_[1].name_);
  ASSERT_STREQ("line_2", members->members_[2].name_);
  EXPECT_EQ(kMaximumOledLineBytes, members->members_[1].string_upper_bound_);
  EXPECT_EQ(kMaximumOledLineBytes, members->members_[2].string_upper_bound_);

  // Humble rosidl_generator_cpp represents a bounded string as a std::string,
  // so assignment is not an enforcement point. Its generated
  // introspection metadata carries the limit for validation. There are no
  // variable-length sequence fields in this package; every array is std::array
  // and its limit is therefore enforced by the generated C++ type itself.
  msg::OledCommand overlong_line_1 = maximum;
  overlong_line_1.line_1.push_back('X');
  EXPECT_GT(overlong_line_1.line_1.size(),
            members->members_[1].string_upper_bound_);

  msg::OledCommand overlong_line_2 = maximum;
  overlong_line_2.line_2.push_back('X');
  EXPECT_GT(overlong_line_2.line_2.size(),
            members->members_[2].string_upper_bound_);
}

}  // namespace
}  // namespace mentor_pi_interfaces::test
