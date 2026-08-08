// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_CONTROLLER_CONTROLLER_RUNTIME_H_
#define MENTOR_PI_MCU_APP_CONTROLLER_CONTROLLER_RUNTIME_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/app/controller/platform_hooks.h"
#include "mentor_pi_mcu/app/controller/status_rgb.h"
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/app/microros/runtime_hooks.h"
#include "mentor_pi_mcu/domain/battery_monitor.h"
#include "mentor_pi_mcu/domain/button_controller.h"
#include "mentor_pi_mcu/domain/command_mailboxes.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/motor_controller.h"
#include "mentor_pi_mcu/domain/pattern_controller.h"
#include "mentor_pi_mcu/domain/pwm_servo_controller.h"
#include "mentor_pi_mcu/drivers/bus_servo_uart.h"
#include "mentor_pi_mcu/drivers/gpio_peripherals.h"
#include "mentor_pi_mcu/drivers/qmi8658.h"
#include "mentor_pi_mcu/drivers/rgb_spi.h"
#include "mentor_pi_mcu/drivers/ssd1306.h"

namespace mentor_pi_mcu::app::controller {

using ControllerTaskMain = void (*)(void* context);

struct ControllerTaskEntry {
  ControllerTaskMain main{nullptr};
  void* context{nullptr};
};

using ControllerTaskEntries =
    std::array<ControllerTaskEntry, kControllerTaskCount>;

class ControllerRuntime {
 public:
  explicit ControllerRuntime(
      const mentor_pi::mcu::MotorControlConfiguration& motor_configuration);
  ControllerRuntime(const ControllerRuntime&) = delete;
  ControllerRuntime& operator=(const ControllerRuntime&) = delete;

  // Configure and InitializeSafeBoot are called once, before task creation.
  // No hook may be changed after InitializeSafeBoot succeeds.
  bool Configure(const PlatformHooks& hooks,
                 const mentor_pi::mcu::drivers::AxisTransform& imu_transform);
  bool InitializeSafeBoot();
  bool initialized() const {
    return initialized_.load(std::memory_order_acquire);
  }

  // Bounded task iterations. These expose deterministic host-side testing and
  // are also used by the non-returning task trampolines below.
  void RunSafetySupervisorOnce();
  void RunMotorControlOnce();
  void RunBusServoOnce();
  void RunSensorOnce();
  void RunPeripheralOnce();

  [[noreturn]] void RunSafetySupervisorTask();
  [[noreturn]] void RunMotorControlTask();
  [[noreturn]] void RunBusServoTask();
  [[noreturn]] void RunSensorTask();
  [[noreturn]] void RunPeripheralTask();

  // The target supplies the already-existing micro-ROS entry, while this
  // runtime supplies the other five owner-task entries and contexts.
  ControllerTaskEntries BuildTaskEntries(ControllerTaskEntry micro_ros_entry);

  mentor_pi_mcu::app::microros::RuntimeHooks BuildMicroRosHooks();

  std::uint32_t MonotonicMilliseconds() const;
  std::uint32_t MonotonicMicroseconds() const;
  void WaitForMicroRos(std::uint32_t maximum_ms) const;
  float MotorMaximumRps() const;
  void EmergencyStopMotors() const;
  void SetSessionActive(bool active, std::uint32_t generation);
  void InvalidateSessionWork(std::uint32_t generation);

  mentor_pi::mcu::CommandAdmission PublishMotorCommand(
      const mentor_pi::mcu::MotorCommand& command,
      std::uint32_t accepted_at_us);
  mentor_pi::mcu::CommandAdmission PublishPwmServoCommand(
      const mentor_pi::mcu::PwmServoCommand& command);
  mentor_pi::mcu::CommandAdmission PublishBusServoCommand(
      const mentor_pi::mcu::BusServoCommand& command);
  mentor_pi::mcu::CommandAdmission PublishLedCommand(
      const mentor_pi::mcu::LedCommand& command);
  mentor_pi::mcu::CommandAdmission PublishBuzzerCommand(
      const mentor_pi::mcu::BuzzerCommand& command);
  mentor_pi::mcu::CommandAdmission PublishRgbCommand(
      const mentor_pi::mcu::RgbCommand& command);
  mentor_pi::mcu::CommandAdmission PublishOledCommand(
      const mentor_pi::mcu::OledCommand& command);

  bool ReadMotorTelemetry(mentor_pi_mcu::app::microros::MotorTelemetry* output);
  bool ReadPwmServoTelemetry(
      mentor_pi_mcu::app::microros::PwmServoTelemetry* output);
  bool ReadImuTelemetry(mentor_pi_mcu::app::microros::ImuTelemetry* output);
  bool ReadBatteryTelemetry(
      mentor_pi_mcu::app::microros::BatteryTelemetry* output);
  bool PopButtonEvent(mentor_pi::mcu::ButtonEvent* output);
  void ReadHealth(mentor_pi_mcu::app::microros::HealthSnapshot* output) const;
  void ReadWorkerDiagnostics(
      mentor_pi_mcu::app::microros::WorkerDiagnostics* output) const;

  bool DispatchMotorModel(mentor_pi_mcu::app::microros::ServiceToken token,
                          mentor_pi::mcu::MotorModel model);
  bool PollMotorModel(mentor_pi_mcu::app::microros::ServiceToken token,
                      mentor_pi_mcu::app::microros::MotorModelReply* output);
  bool DispatchMotorPid(mentor_pi_mcu::app::microros::ServiceToken token,
                        const mentor_pi::mcu::SetMotorPidCommand& command);
  bool PollMotorPid(mentor_pi_mcu::app::microros::ServiceToken token,
                    mentor_pi_mcu::app::microros::MotorPidReply* output);
  bool CancelMotorPid(mentor_pi_mcu::app::microros::ServiceToken token);
  bool DispatchPwmOffsets(mentor_pi_mcu::app::microros::ServiceToken token,
                          const mentor_pi::mcu::PwmServoOffsetCommand& command);
  bool PollPwmOffsets(mentor_pi_mcu::app::microros::ServiceToken token,
                      mentor_pi_mcu::app::microros::PwmOffsetsReply* output);
  bool DispatchBatteryThreshold(
      mentor_pi_mcu::app::microros::ServiceToken token,
      std::uint16_t threshold_mv);
  bool PollBatteryThreshold(
      mentor_pi_mcu::app::microros::ServiceToken token,
      mentor_pi_mcu::app::microros::BatteryThresholdReply* output);
  bool DispatchBusGetState(
      mentor_pi_mcu::app::microros::ServiceToken token,
      const mentor_pi::mcu::GetBusServoStateCommand& command);
  bool PollBusGetState(
      mentor_pi_mcu::app::microros::ServiceToken token,
      mentor_pi_mcu::app::microros::GetBusServoStateReply* output);
  bool DispatchBusConfigure(
      mentor_pi_mcu::app::microros::ServiceToken token,
      const mentor_pi::mcu::ConfigureBusServoCommand& command);
  bool PollBusConfigure(
      mentor_pi_mcu::app::microros::ServiceToken token,
      mentor_pi_mcu::app::microros::ConfigureBusServoReply* output);
  bool DispatchBusStop(mentor_pi_mcu::app::microros::ServiceToken token,
                       const mentor_pi::mcu::StopBusServosCommand& command);
  bool PollBusStop(mentor_pi_mcu::app::microros::ServiceToken token,
                   mentor_pi_mcu::app::microros::StopBusServosReply* output);

  void AdvanceMicroRosHeartbeat();
  void RecordSuccessfulRosHeartbeat();

 private:
  enum class SlotState : std::uint8_t {
    kIdle = 0,
    kWriting = 1,
    kReady = 2,
    kProcessing = 3,
    kComplete = 4,
    kCanceled = 5,
  };

  template <typename Request, typename Reply>
  struct ServiceSlot {
    std::atomic<SlotState> state{SlotState::kIdle};
    mentor_pi_mcu::app::microros::ServiceToken token{};
    Request request{};
    Reply reply{};
  };

  struct MotorModelRequest {
    mentor_pi::mcu::MotorModel model{mentor_pi::mcu::MotorModel::kJga27};
  };

  struct BatteryThresholdRequest {
    std::uint16_t threshold_mv{mentor_pi::mcu::kDefaultBatteryThresholdMv};
  };

  struct BusServiceRequest {
    mentor_pi_mcu::app::microros::BusServiceKind kind{
        mentor_pi_mcu::app::microros::BusServiceKind::kNone};
    mentor_pi::mcu::GetBusServoStateCommand get_state{};
    mentor_pi::mcu::ConfigureBusServoCommand configure{};
    mentor_pi::mcu::StopBusServosCommand stop{};
  };

  struct BusServiceReply {
    mentor_pi_mcu::app::microros::BusServiceKind kind{
        mentor_pi_mcu::app::microros::BusServiceKind::kNone};
    mentor_pi_mcu::app::microros::GetBusServoStateReply get_state{};
    mentor_pi_mcu::app::microros::ConfigureBusServoReply configure{};
    mentor_pi_mcu::app::microros::StopBusServosReply stop{};
  };

  struct MailboxTag {
    std::uint32_t session_generation{0U};
    std::uint32_t command_generation{0U};
  };

  struct BatteryAlarmRequest {
    std::uint32_t timestamp_ms{0U};
  };

  struct BatteryDisplayState {
    std::uint16_t voltage_mv{0U};
    bool valid{false};
  };

  class RegisterI2cAdapter final : public mentor_pi::mcu::drivers::RegisterI2c {
   public:
    explicit RegisterI2cAdapter(ControllerRuntime* runtime)
        : runtime_(runtime) {}
    mentor_pi::mcu::drivers::IoStatus Read(std::uint8_t address,
                                           std::uint8_t reg, std::uint8_t* data,
                                           std::size_t size,
                                           std::uint32_t deadline_us) override;
    mentor_pi::mcu::drivers::IoStatus Write(std::uint8_t address,
                                            std::uint8_t reg,
                                            const std::uint8_t* data,
                                            std::size_t size,
                                            std::uint32_t deadline_us) override;

   private:
    ControllerRuntime* runtime_;
  };

  class RawI2cAdapter final : public mentor_pi::mcu::drivers::RawI2c {
   public:
    explicit RawI2cAdapter(ControllerRuntime* runtime) : runtime_(runtime) {}
    mentor_pi::mcu::drivers::IoStatus Write(std::uint8_t address,
                                            const std::uint8_t* data,
                                            std::size_t size,
                                            std::uint32_t deadline_ms) override;

   private:
    ControllerRuntime* runtime_;
  };

  class BusUartAdapter final : public mentor_pi::mcu::drivers::HalfDuplexUart {
   public:
    explicit BusUartAdapter(ControllerRuntime* runtime) : runtime_(runtime) {}
    mentor_pi::mcu::drivers::IoStatus BeginExchange(
        const std::uint8_t* tx, std::size_t tx_size, std::size_t max_reply_size,
        std::uint32_t deadline_ms) override;
    mentor_pi::mcu::drivers::IoStatus PollExchange(
        std::uint32_t now_ms, std::uint8_t* reply, std::size_t capacity,
        std::size_t* reply_size) override;
    void Cancel() override;

   private:
    ControllerRuntime* runtime_;
  };

  class RgbSpiAdapter final : public mentor_pi::mcu::drivers::AsyncSpi {
   public:
    explicit RgbSpiAdapter(ControllerRuntime* runtime) : runtime_(runtime) {}
    mentor_pi::mcu::drivers::IoStatus BeginTransmit(
        const std::uint8_t* data, std::size_t size,
        std::uint32_t deadline_us) override;
    mentor_pi::mcu::drivers::IoStatus PollTransmit(
        std::uint32_t now_us) override;
    void Cancel() override;

   private:
    ControllerRuntime* runtime_;
  };

  class PeripheralAdapter final
      : public mentor_pi::mcu::drivers::PeripheralHardware {
   public:
    explicit PeripheralAdapter(ControllerRuntime* runtime)
        : runtime_(runtime) {}
    bool ReadButtonPin(std::size_t button) const override;
    void WriteLedPin(std::size_t led, bool high) override;
    mentor_pi::mcu::Result SetBuzzerTone(std::uint16_t frequency_hz,
                                         bool enabled) override;

   private:
    ControllerRuntime* runtime_;
  };

  class CriticalGuard {
   public:
    explicit CriticalGuard(ControllerRuntime* runtime) : runtime_(runtime) {
      runtime_->hooks_.enter_critical(runtime_->hooks_.context);
    }
    ~CriticalGuard() {
      runtime_->hooks_.exit_critical(runtime_->hooks_.context);
    }

   private:
    ControllerRuntime* runtime_;
  };

  template <typename Request, typename Reply>
  bool Dispatch(ServiceSlot<Request, Reply>* slot,
                mentor_pi_mcu::app::microros::ServiceToken token,
                const Request& request);
  template <typename Request, typename Reply>
  bool Poll(ServiceSlot<Request, Reply>* slot,
            mentor_pi_mcu::app::microros::ServiceToken token, Reply* output);
  template <typename Request, typename Reply>
  bool Take(ServiceSlot<Request, Reply>* slot,
            mentor_pi_mcu::app::microros::ServiceToken* token,
            Request* request);
  template <typename Request, typename Reply>
  void Complete(ServiceSlot<Request, Reply>* slot,
                mentor_pi_mcu::app::microros::ServiceToken token,
                const Reply& reply);
  template <typename Request, typename Reply>
  void Cancel(ServiceSlot<Request, Reply>* slot,
              std::uint32_t session_generation);

  bool TokenIsCurrent(mentor_pi_mcu::app::microros::ServiceToken token) const;
  void ConsumeMotorCommand(std::uint32_t now_us);
  void ProcessMotorModelService();
  void ProcessMotorPidService();
  void PublishMotorSnapshot(std::uint32_t now_ms);
  void SynchronizePwmSession(std::uint32_t now_ms);
  void ProcessPwmCommands(std::uint32_t now_ms);
  void ProcessPwmOffsetService(std::uint32_t now_ms);
  void ProcessPwmFrame(std::uint32_t now_ms);
  void ProcessPwmFrameLocked(std::uint32_t now_ms);
  void PreparePendingPwmFrame(std::uint32_t now_ms);
  void ProcessDiscreteOutputs(std::uint32_t now_ms);
  void ProcessRgb(std::uint32_t now_ms, std::uint32_t now_us);
  void ProcessOled(std::uint32_t now_ms);
  void ProcessBusDriver(std::uint32_t now_ms);
  void StartNextBusWork(std::uint32_t now_ms);
  void FinishBusOperation(
      const mentor_pi::mcu::drivers::BusServoPollResult& result);
  void SampleImu(std::uint32_t now_ms, std::uint32_t now_us);
  void SampleButtons(std::uint32_t now_ms);
  void SampleBattery(std::uint32_t now_ms);
  std::uint32_t NextSensorWaitMilliseconds() const;
  void ProcessBatteryThresholdService();

  void RecordTaskProgress(ControllerTask task, std::uint32_t started_us,
                          std::uint32_t expected_period_us);
  void RecordPeripheralResult(std::size_t peripheral_index,
                              mentor_pi::mcu::Result result,
                              mentor_pi_mcu::app::microros::ErrorSource source);
  void RecordLastError(mentor_pi::mcu::Result result,
                       mentor_pi_mcu::app::microros::ErrorSource source);
  // Caller holds CriticalGuard. This is the common fail-closed transition for
  // target output faults and safety-supervisor actions.
  void RevokeMotorAuthorityLocked(bool fatal_output_fault);
  // Caller is MotorControlTask and holds CriticalGuard. Synchronizes the
  // portable owner state with an already-revoked hardware authority.
  void DeactivateMotorOwnerLocked();
  void IncrementMotorWatchdogTrips(std::uint8_t previous_mask,
                                   std::uint8_t current_mask);
  static bool GenerationIsAfter(std::uint32_t candidate,
                                std::uint32_t watermark);
  static std::uint32_t FloatBits(float value);
  static float BitsFloat(std::uint32_t value);

  PlatformHooks hooks_{};
  mentor_pi::mcu::drivers::AxisTransform imu_transform_{};
  std::atomic<bool> configured_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> desired_session_active_{false};
  std::atomic<std::uint32_t> session_generation_{0U};
  std::atomic<std::uint32_t> motor_max_rps_bits_{0U};
  std::atomic<bool> fatal_output_fault_{false};
  std::atomic<bool> startup_motor_inhibited_{true};

  RegisterI2cAdapter register_i2c_adapter_;
  RawI2cAdapter raw_i2c_adapter_;
  BusUartAdapter bus_uart_adapter_;
  RgbSpiAdapter rgb_spi_adapter_;
  PeripheralAdapter peripheral_adapter_;
  mentor_pi::mcu::drivers::Qmi8658Driver imu_driver_;
  mentor_pi::mcu::drivers::BusServoUartDriver bus_driver_;
  mentor_pi::mcu::drivers::RgbSpiDriver rgb_driver_;
  mentor_pi::mcu::drivers::Ssd1306Driver oled_driver_;
  mentor_pi::mcu::drivers::GpioPeripheralDriver gpio_driver_;

  mentor_pi::mcu::MotorController motor_controller_{};
  mentor_pi::mcu::PwmServoController pwm_controller_{};
  mentor_pi::mcu::BatteryMonitor battery_monitor_{};
  mentor_pi::mcu::ButtonController button_controller_{};
  mentor_pi::mcu::LedController led_controller_{};
  mentor_pi::mcu::BuzzerController buzzer_controller_{};
  StatusRgbController status_rgb_controller_{};
  HeartbeatLedController heartbeat_led_controller_{};

  mentor_pi::mcu::MotorCommandMailbox motor_mailbox_{};
  mentor_pi::mcu::PwmCommandMailbox pwm_mailbox_{};
  mentor_pi::mcu::BusMotionMailbox bus_mailbox_{};
  mentor_pi::mcu::LedCommandMailbox led_mailbox_{};
  mentor_pi::mcu::BuzzerCommandMailbox buzzer_mailbox_{};
  mentor_pi::mcu::RgbCommandMailbox rgb_mailbox_{};
  mentor_pi::mcu::OledCommandMailbox oled_mailbox_{};
  MailboxTag motor_mailbox_tag_{};
  MailboxTag pwm_mailbox_tag_{};
  MailboxTag bus_mailbox_tag_{};
  MailboxTag led_mailbox_tag_{};
  MailboxTag buzzer_mailbox_tag_{};
  MailboxTag rgb_mailbox_tag_{};
  MailboxTag oled_mailbox_tag_{};
  std::uint32_t motor_mailbox_session_generation_{0U};
  std::uint32_t pwm_mailbox_session_generation_{0U};
  std::uint32_t led_mailbox_session_generation_{0U};
  std::uint32_t rgb_mailbox_session_generation_{0U};
  std::uint32_t oled_mailbox_session_generation_{0U};

  ServiceSlot<MotorModelRequest, mentor_pi_mcu::app::microros::MotorModelReply>
      motor_model_slot_{};
  ServiceSlot<mentor_pi::mcu::SetMotorPidCommand,
              mentor_pi_mcu::app::microros::MotorPidReply>
      motor_pid_slot_{};
  ServiceSlot<mentor_pi::mcu::PwmServoOffsetCommand,
              mentor_pi_mcu::app::microros::PwmOffsetsReply>
      pwm_offsets_slot_{};
  ServiceSlot<BatteryThresholdRequest,
              mentor_pi_mcu::app::microros::BatteryThresholdReply>
      battery_threshold_slot_{};
  ServiceSlot<BusServiceRequest, BusServiceReply> bus_service_slot_{};

  mentor_pi::mcu::LatestMailbox<mentor_pi_mcu::app::microros::MotorTelemetry>
      motor_telemetry_{};
  mentor_pi::mcu::LatestMailbox<mentor_pi_mcu::app::microros::PwmServoTelemetry>
      pwm_telemetry_{};
  mentor_pi::mcu::LatestMailbox<mentor_pi_mcu::app::microros::ImuTelemetry>
      imu_telemetry_{};
  mentor_pi::mcu::LatestMailbox<mentor_pi_mcu::app::microros::BatteryTelemetry>
      battery_telemetry_{};
  mentor_pi::mcu::LatestMailbox<BatteryAlarmRequest> battery_alarm_mailbox_{};
  mentor_pi::mcu::LatestMailbox<BatteryDisplayState> battery_display_mailbox_{};

  std::array<std::uint32_t, mentor_pi::mcu::kMotorCount>
      consumed_motor_field_generation_{};
  std::array<std::uint32_t, mentor_pi::mcu::kPwmServoCount>
      consumed_pwm_field_generation_{};
  std::uint32_t motor_consumed_session_generation_{0U};
  std::uint32_t pwm_consumed_session_generation_{0U};
  std::array<std::uint32_t, mentor_pi::mcu::kLedCount>
      consumed_led_field_generation_{};
  std::array<std::uint32_t, mentor_pi::mcu::kRgbPixelCount>
      consumed_rgb_field_generation_{};
  std::array<std::uint32_t, mentor_pi::mcu::kOledHostLineCount>
      consumed_oled_field_generation_{};
  std::uint32_t led_consumed_session_generation_{0U};
  std::uint32_t rgb_consumed_session_generation_{0U};
  std::uint32_t oled_consumed_session_generation_{0U};
  std::array<std::uint32_t, mentor_pi::mcu::kMotorCount>
      last_lease_expiry_count_{};

  mentor_pi_mcu::app::microros::ServiceToken pending_pwm_offset_token_{};
  mentor_pi_mcu::app::microros::PwmOffsetsReply pending_pwm_offset_reply_{};
  mentor_pi::mcu::PwmFrameUpdate pending_pwm_frame_{};
  std::array<std::uint16_t, mentor_pi::mcu::kPwmServoCount>
      active_pwm_output_us_{
          mentor_pi::mcu::kPwmResetPulseUs, mentor_pi::mcu::kPwmResetPulseUs,
          mentor_pi::mcu::kPwmResetPulseUs, mentor_pi::mcu::kPwmResetPulseUs};
  std::array<std::int16_t, mentor_pi::mcu::kPwmServoCount>
      active_pwm_offset_us_{};
  std::uint32_t last_pwm_frame_sequence_{0U};
  std::uint32_t pwm_owner_session_generation_{0U};
  bool pwm_offset_waiting_for_commit_{false};
  bool pwm_shadow_waiting_for_commit_{false};

  mentor_pi::mcu::RgbState active_rgb_state_{};
  mentor_pi::mcu::RgbState inflight_rgb_state_{};
  mentor_pi::mcu::RgbState pending_rgb_state_{};
  mentor_pi::mcu::OledState active_oled_state_{};
  mentor_pi::mcu::OledState pending_oled_state_{};
  std::uint16_t oled_battery_mv_{0U};
  std::uint16_t rendered_oled_battery_mv_{0U};
  std::uint32_t last_oled_attempt_ms_{0U};
  std::uint32_t pending_rgb_session_generation_{0U};
  std::uint32_t active_rgb_session_generation_{0U};
  std::uint32_t pending_oled_session_generation_{0U};
  bool rgb_pending_{false};
  bool oled_pending_{false};
  bool oled_initialized_{false};

  mentor_pi_mcu::app::microros::ServiceToken active_bus_service_token_{};
  mentor_pi_mcu::app::microros::BusServiceKind active_bus_operation_{
      mentor_pi_mcu::app::microros::BusServiceKind::kNone};
  BusServiceRequest pending_bus_request_{};
  mentor_pi_mcu::app::microros::ServiceToken pending_bus_token_{};
  std::uint16_t previous_bus_completed_mask_{0U};
  std::uint32_t bus_stop_watermark_{0U};
  std::uint32_t last_bus_command_generation_{0U};
  std::uint32_t bus_move_session_generation_{0U};
  bool bus_service_pending_{false};
  bool bus_move_active_{false};

  mentor_pi_mcu::app::microros::ImuTelemetry imu_state_{};
  mentor_pi_mcu::app::microros::BatteryTelemetry battery_state_{};
  std::uint32_t next_imu_initialize_ms_{0U};
  std::uint32_t next_button_sample_ms_{0U};
  std::uint32_t next_battery_sample_ms_{0U};
  bool imu_initialized_{false};
  bool imu_transform_error_recorded_{false};
  bool button_sampling_started_{false};
  bool battery_sampling_started_{false};

  static constexpr std::uint32_t kSensorMaximumWaitMs = 4U;
  static constexpr std::uint32_t kBatterySamplePeriodMs = 50U;

  std::array<std::atomic<std::uint32_t>, kControllerTaskCount>
      task_heartbeat_ms_{};
  std::array<std::atomic<std::uint32_t>, kControllerTaskCount>
      task_missed_releases_{};
  std::array<std::atomic<std::uint32_t>, kControllerTaskCount>
      task_max_execution_us_{};
  std::array<std::atomic<bool>, kControllerTaskCount> task_seen_{};
  std::array<mentor_pi::mcu::SaturatingCounter<std::uint32_t>, 8>
      peripheral_errors_{};
  std::array<mentor_pi::mcu::SaturatingCounter<std::uint32_t>, 8>
      peripheral_timeouts_{};
  mentor_pi::mcu::SaturatingCounter<std::uint32_t> motor_watchdog_trips_{};
  mentor_pi::mcu::SaturatingCounter<std::uint32_t>
      motor_command_consumptions_{};
  mentor_pi::mcu::SaturatingCounter<std::uint32_t>
      motor_command_age_over_20_ms_{};
  std::atomic<std::uint32_t> motor_command_max_age_us_{0U};
  std::atomic<std::uint8_t> last_watchdog_task_{255U};
  std::atomic<std::uint32_t> last_error_uptime_ms_{0U};
  std::atomic<std::uint16_t> last_error_detail_{0U};
  std::atomic<std::uint8_t> last_error_code_{0U};
  std::atomic<std::uint8_t> last_error_source_{0U};
  std::atomic<bool> watchdog_withheld_{false};
  std::atomic<bool> imu_healthy_{false};
  std::atomic<bool> low_battery_{false};
  std::atomic<bool> bus_busy_{false};
  std::atomic<std::uint32_t> successful_ros_heartbeats_{0U};
  std::atomic<std::uint8_t> motor_watchdog_mask_{0U};
  std::uint32_t motor_release_count_{0U};
  std::uint32_t last_motor_control_sample_us_{0U};
  std::uint32_t motor_owner_session_generation_{0U};
  bool motor_control_sample_initialized_{false};
  std::uint32_t safety_startup_deadline_ms_{0U};
  bool safety_startup_grace_started_{false};
  bool watchdog_task_persist_requested_{false};
};

template <typename Request, typename Reply>
bool ControllerRuntime::Dispatch(
    ServiceSlot<Request, Reply>* slot,
    mentor_pi_mcu::app::microros::ServiceToken token, const Request& request) {
  if (slot == nullptr || !TokenIsCurrent(token)) {
    return false;
  }
  SlotState expected = SlotState::kIdle;
  if (!slot->state.compare_exchange_strong(expected, SlotState::kWriting,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire)) {
    return false;
  }
  slot->token = token;
  slot->request = request;
  slot->reply = {};
  slot->state.store(SlotState::kReady, std::memory_order_release);
  return true;
}

template <typename Request, typename Reply>
bool ControllerRuntime::Poll(ServiceSlot<Request, Reply>* slot,
                             mentor_pi_mcu::app::microros::ServiceToken token,
                             Reply* output) {
  if (slot == nullptr || output == nullptr ||
      slot->state.load(std::memory_order_acquire) != SlotState::kComplete ||
      slot->token.session_generation != token.session_generation ||
      slot->token.request_generation != token.request_generation) {
    return false;
  }
  *output = slot->reply;
  slot->state.store(SlotState::kIdle, std::memory_order_release);
  return true;
}

template <typename Request, typename Reply>
bool ControllerRuntime::Take(ServiceSlot<Request, Reply>* slot,
                             mentor_pi_mcu::app::microros::ServiceToken* token,
                             Request* request) {
  if (slot == nullptr || token == nullptr || request == nullptr) {
    return false;
  }
  SlotState expected = SlotState::kReady;
  if (!slot->state.compare_exchange_strong(expected, SlotState::kProcessing,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire)) {
    return false;
  }
  *token = slot->token;
  *request = slot->request;
  return true;
}

template <typename Request, typename Reply>
void ControllerRuntime::Complete(
    ServiceSlot<Request, Reply>* slot,
    mentor_pi_mcu::app::microros::ServiceToken token, const Reply& reply) {
  if (slot == nullptr ||
      slot->token.session_generation != token.session_generation ||
      slot->token.request_generation != token.request_generation) {
    return;
  }
  if (slot->state.load(std::memory_order_acquire) == SlotState::kCanceled ||
      !TokenIsCurrent(token)) {
    slot->state.store(SlotState::kIdle, std::memory_order_release);
    return;
  }
  slot->reply = reply;
  SlotState expected = SlotState::kProcessing;
  if (!slot->state.compare_exchange_strong(expected, SlotState::kComplete,
                                           std::memory_order_release,
                                           std::memory_order_acquire) &&
      expected == SlotState::kCanceled) {
    slot->state.store(SlotState::kIdle, std::memory_order_release);
  }
}

template <typename Request, typename Reply>
void ControllerRuntime::Cancel(ServiceSlot<Request, Reply>* slot,
                               std::uint32_t session_generation) {
  if (slot == nullptr) {
    return;
  }
  SlotState current = slot->state.load(std::memory_order_acquire);
  while (current != SlotState::kIdle) {
    if (current == SlotState::kWriting ||
        slot->token.session_generation != session_generation) {
      return;
    }
    const SlotState replacement = current == SlotState::kProcessing
                                      ? SlotState::kCanceled
                                      : SlotState::kIdle;
    if (slot->state.compare_exchange_weak(current, replacement,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire)) {
      return;
    }
  }
}

// Process-wide static instance used by target main. Tests may construct a
// separate ControllerRuntime when isolation is useful.
ControllerRuntime& ControllerInstance(
    const mentor_pi::mcu::MotorControlConfiguration& motor_configuration);

}  // namespace mentor_pi_mcu::app::controller

#endif  // MENTOR_PI_MCU_APP_CONTROLLER_CONTROLLER_RUNTIME_H_
