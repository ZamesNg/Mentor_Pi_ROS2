// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_RUNTIME_HOOKS_H_
#define MENTOR_PI_MCU_APP_MICROROS_RUNTIME_HOOKS_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/button_controller.h"
#include "mentor_pi_mcu/domain/command_mailboxes.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi_mcu::app::microros {

inline constexpr std::size_t kSubscriptionCount = 7U;
inline constexpr std::size_t kTaskCount = 6U;
inline constexpr std::size_t kRamClassCount = 2U;
inline constexpr std::size_t kPeripheralCount = 8U;
inline constexpr std::size_t kUsartErrorCount = 4U;

struct MotorTelemetry {
  std::uint32_t timestamp_ms{0U};
  std::array<float, mentor_pi::mcu::kMotorCount> target_rps{};
  std::array<float, mentor_pi::mcu::kMotorCount> measured_rps{};
  std::array<std::int64_t, mentor_pi::mcu::kMotorCount> encoder_count{};
  mentor_pi::mcu::MotorModel motor_model{mentor_pi::mcu::MotorModel::kJga27};
  std::uint8_t watchdog_stop_mask{0U};
};

struct PwmServoTelemetry {
  std::uint32_t timestamp_ms{0U};
  std::array<std::uint16_t, mentor_pi::mcu::kPwmServoCount>
      target_pulse_width_us{1500U, 1500U, 1500U, 1500U};
  std::array<std::uint16_t, mentor_pi::mcu::kPwmServoCount>
      output_pulse_width_us{1500U, 1500U, 1500U, 1500U};
  std::array<std::int16_t, mentor_pi::mcu::kPwmServoCount> offset_us{};
  std::uint8_t moving_mask{0U};
};

struct ImuTelemetry {
  std::uint32_t timestamp_ms{0U};
  std::array<float, 3> angular_velocity_rad_s{};
  std::array<float, 3> linear_acceleration_m_s2{};
  bool valid{false};
};

struct BatteryTelemetry {
  std::uint32_t timestamp_ms{0U};
  std::uint16_t voltage_mv{0U};
  std::uint16_t low_threshold_mv{6300U};
  bool valid{false};
  bool below_threshold{false};
};

struct HealthSnapshot {
  bool motor_watchdog_active{false};
  bool low_battery{false};
  bool imu_healthy{false};
  bool bus_servo_busy{false};
  bool nonfatal_degraded{false};
  bool output_processing_fault{false};
};

// The owning tasks provide hardware and resource values. ROS/session/transport
// counters are overlaid by MicroRosRuntime so no worker needs an rcl type.
struct WorkerDiagnostics {
  std::uint32_t button_event_drops{0U};
  std::array<std::uint32_t, mentor_pi::mcu::kMotorCount> motor_lease_expiries{};
  std::array<std::uint32_t, mentor_pi::mcu::kMotorCount>
      motor_command_rejections{};
  std::uint32_t motor_watchdog_trips{0U};
  std::uint32_t motor_command_consumptions{0U};
  std::uint32_t motor_command_age_over_20_ms{0U};
  std::uint32_t motor_command_max_age_us{0U};
  std::array<std::uint32_t, kPeripheralCount> peripheral_errors{};
  std::array<std::uint32_t, kPeripheralCount> peripheral_timeouts{};
  std::array<std::uint32_t, kTaskCount> task_missed_releases{};
  std::array<std::uint32_t, kTaskCount> task_max_execution_us{};
  std::array<std::uint32_t, kTaskCount> task_stack_high_water_bytes{};
  std::array<std::uint32_t, kTaskCount> task_heartbeat_age_ms{};
  std::array<std::uint32_t, kRamClassCount> free_ram_bytes{};
  std::array<std::uint32_t, kRamClassCount> minimum_free_ram_bytes{};
  std::uint32_t flash_used_bytes{0U};
  std::uint32_t flash_total_bytes{0U};
  std::uint8_t last_reset_reason{255U};
  std::uint8_t last_watchdog_task{255U};
  std::uint32_t last_error_uptime_ms{0U};
  std::uint16_t last_error_detail{0U};
  std::uint8_t last_error_code{0U};
  std::uint8_t last_error_source{0U};
};

struct MotorModelReply {
  mentor_pi::mcu::Result result{};
  mentor_pi::mcu::MotorModel active_model{mentor_pi::mcu::MotorModel::kJga27};
  std::uint32_t ticks_per_revolution{0U};
  float max_rps{0.0F};
};

struct MotorPidReply {
  mentor_pi::mcu::Result result{};
  std::uint8_t applied_mask{0U};
};

struct PwmOffsetsReply {
  mentor_pi::mcu::Result result{};
  std::uint8_t applied_mask{0U};
};

struct BatteryThresholdReply {
  mentor_pi::mcu::Result result{};
  std::uint16_t active_threshold_mv{6300U};
};

struct BusServoState {
  std::uint16_t valid_fields{0U};
  std::uint8_t requested_id{0U};
  std::uint8_t reported_id{0U};
  std::int16_t position{0};
  std::int8_t offset{0};
  std::uint16_t voltage_mv{0U};
  std::uint8_t temperature_c{0U};
  std::uint16_t position_min{0U};
  std::uint16_t position_max{0U};
  std::uint16_t voltage_min_mv{0U};
  std::uint16_t voltage_max_mv{0U};
  std::uint8_t temperature_limit_c{0U};
  bool torque_enabled{false};
};

struct GetBusServoStateReply {
  mentor_pi::mcu::Result result{};
  BusServoState state{};
};

struct ConfigureBusServoReply {
  mentor_pi::mcu::Result result{};
  std::uint16_t applied_mask{0U};
  std::uint8_t effective_id{0U};
};

struct StopBusServosReply {
  mentor_pi::mcu::Result result{};
  std::uint8_t commands_transmitted{0U};
};

enum class BusServiceKind : std::uint8_t {
  kNone = 0,
  kGetState = 1,
  kConfigure = 2,
  kStop = 3,
};

struct ServiceToken {
  std::uint32_t session_generation{0U};
  std::uint32_t request_generation{0U};
};

// Every callback in this table is called only by MicroRosTask. Publish and
// dispatch callbacks must be bounded, nonblocking copies into storage owned by
// another task. Poll callbacks return false until a matching completion exists.
struct RuntimeHooks {
  void* context{nullptr};

  std::uint32_t (*monotonic_milliseconds)(void* context){nullptr};
  std::uint32_t (*monotonic_microseconds)(void* context){nullptr};
  void (*wait_milliseconds)(void* context, std::uint32_t maximum_ms){nullptr};
  void (*advance_task_heartbeat)(void* context){nullptr};
  void (*record_successful_ros_heartbeat)(void* context){nullptr};
  void (*emergency_stop_motors)(void* context){nullptr};
  void (*set_session_active)(void* context, bool active,
                             std::uint32_t generation){nullptr};
  void (*invalidate_session_work)(void* context,
                                  std::uint32_t generation){nullptr};

  float (*motor_max_rps)(void* context){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_motor_command)(
      void* context, const mentor_pi::mcu::MotorCommand& command,
      std::uint32_t accepted_at_us){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_pwm_servo_command)(
      void* context, const mentor_pi::mcu::PwmServoCommand& command){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_bus_servo_command)(
      void* context, const mentor_pi::mcu::BusServoCommand& command){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_led_command)(
      void* context, const mentor_pi::mcu::LedCommand& command){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_buzzer_command)(
      void* context, const mentor_pi::mcu::BuzzerCommand& command){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_rgb_command)(
      void* context, const mentor_pi::mcu::RgbCommand& command){nullptr};
  mentor_pi::mcu::CommandAdmission (*publish_oled_command)(
      void* context, const mentor_pi::mcu::OledCommand& command){nullptr};

  bool (*read_motor_telemetry)(void* context, MotorTelemetry* output){nullptr};
  bool (*read_pwm_servo_telemetry)(void* context,
                                   PwmServoTelemetry* output){nullptr};
  bool (*read_imu_telemetry)(void* context, ImuTelemetry* output){nullptr};
  bool (*read_battery_telemetry)(void* context,
                                 BatteryTelemetry* output){nullptr};
  bool (*pop_button_event)(void* context,
                           mentor_pi::mcu::ButtonEvent* output){nullptr};
  void (*read_health)(void* context, HealthSnapshot* output){nullptr};
  void (*read_worker_diagnostics)(void* context,
                                  WorkerDiagnostics* output){nullptr};

  bool (*dispatch_motor_model)(void* context, ServiceToken token,
                               mentor_pi::mcu::MotorModel model){nullptr};
  bool (*poll_motor_model)(void* context, ServiceToken token,
                           MotorModelReply* output){nullptr};
  bool (*dispatch_motor_pid)(void* context, ServiceToken token,
                             const mentor_pi::mcu::SetMotorPidCommand& command){
      nullptr};
  bool (*poll_motor_pid)(void* context, ServiceToken token,
                         MotorPidReply* output){nullptr};
  bool (*cancel_motor_pid)(void* context, ServiceToken token){nullptr};
  bool (*dispatch_pwm_offsets)(
      void* context, ServiceToken token,
      const mentor_pi::mcu::PwmServoOffsetCommand& command){nullptr};
  bool (*poll_pwm_offsets)(void* context, ServiceToken token,
                           PwmOffsetsReply* output){nullptr};
  bool (*dispatch_battery_threshold)(void* context, ServiceToken token,
                                     std::uint16_t threshold_mv){nullptr};
  bool (*poll_battery_threshold)(void* context, ServiceToken token,
                                 BatteryThresholdReply* output){nullptr};

  bool (*dispatch_bus_get_state)(
      void* context, ServiceToken token,
      const mentor_pi::mcu::GetBusServoStateCommand& command){nullptr};
  bool (*poll_bus_get_state)(void* context, ServiceToken token,
                             GetBusServoStateReply* output){nullptr};
  bool (*dispatch_bus_configure)(
      void* context, ServiceToken token,
      const mentor_pi::mcu::ConfigureBusServoCommand& command){nullptr};
  bool (*poll_bus_configure)(void* context, ServiceToken token,
                             ConfigureBusServoReply* output){nullptr};
  bool (*dispatch_bus_stop)(
      void* context, ServiceToken token,
      const mentor_pi::mcu::StopBusServosCommand& command){nullptr};
  bool (*poll_bus_stop)(void* context, ServiceToken token,
                        StopBusServosReply* output){nullptr};
};

bool RuntimeHooksAreComplete(const RuntimeHooks& hooks);

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_RUNTIME_HOOKS_H_
