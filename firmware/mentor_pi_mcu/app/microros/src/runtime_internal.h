// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_SRC_RUNTIME_INTERNAL_H_
#define MENTOR_PI_MCU_APP_MICROROS_SRC_RUNTIME_INTERNAL_H_

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "mentor_pi_interfaces/msg/battery_state.h"
#include "mentor_pi_interfaces/msg/bus_servo_command.h"
#include "mentor_pi_interfaces/msg/button_event.h"
#include "mentor_pi_interfaces/msg/buzzer_command.h"
#include "mentor_pi_interfaces/msg/controller_diagnostics.h"
#include "mentor_pi_interfaces/msg/heartbeat.h"
#include "mentor_pi_interfaces/msg/imu_state.h"
#include "mentor_pi_interfaces/msg/led_command.h"
#include "mentor_pi_interfaces/msg/motor_command.h"
#include "mentor_pi_interfaces/msg/motor_state.h"
#include "mentor_pi_interfaces/msg/oled_command.h"
#include "mentor_pi_interfaces/msg/pwm_servo_command.h"
#include "mentor_pi_interfaces/msg/pwm_servo_state.h"
#include "mentor_pi_interfaces/msg/rgb_command.h"
#include "mentor_pi_interfaces/srv/configure_bus_servo.h"
#include "mentor_pi_interfaces/srv/get_bus_servo_state.h"
#include "mentor_pi_interfaces/srv/set_battery_threshold.h"
#include "mentor_pi_interfaces/srv/set_motor_adrc.h"
#include "mentor_pi_interfaces/srv/set_motor_model.h"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.h"
#include "mentor_pi_interfaces/srv/stop_bus_servos.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw/types.h"
}

#include "mentor_pi_mcu/app/microros/arena_allocator.h"
#include "mentor_pi_mcu/app/microros/endpoint_contract.h"
#include "mentor_pi_mcu/app/microros/middleware_fault_proxy.h"
#include "mentor_pi_mcu/app/microros/runtime.h"
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/app/microros/runtime_hooks.h"

namespace mentor_pi_mcu::app::microros {

enum class PublisherIndex : std::uint8_t {
  kMotors = 0,
  kPwmServos,
  kImu,
  kButtons,
  kBattery,
  kHeartbeat,
  kDiagnostics,
  kCount,
};

enum class SubscriptionIndex : std::uint8_t {
  kMotors = 0,
  kPwmServos,
  kBusServos,
  kLeds,
  kBuzzer,
  kRgb,
  kOled,
  kCount,
};

enum class ServiceIndex : std::uint8_t {
  kMotorModel = 0,
  kMotorAdrc,
  kPwmOffsets,
  kBusGetState,
  kBusConfigure,
  kBusStop,
  kBatteryThreshold,
  kCount,
};

inline constexpr std::size_t kPublisherCount =
    static_cast<std::size_t>(PublisherIndex::kCount);
inline constexpr std::size_t kServiceCount =
    static_cast<std::size_t>(ServiceIndex::kCount);
inline constexpr std::size_t kServiceSlotCount = 5U;
inline constexpr std::size_t kServiceRequestGroupCount = 5U;
inline constexpr std::size_t kBusServiceEndpointCount = 3U;
inline constexpr std::size_t kBestEffortPublisherCount = 3U;
inline constexpr std::size_t kReliablePublisherCount = 4U;
static_assert(kPublisherCount == kPublisherEndpoints.size());
static_assert(kSubscriptionCount == kSubscriptionEndpoints.size());
static_assert(kServiceCount == kServiceEndpoints.size());
static_assert(kPublisherCount == kLifecyclePublisherCount);
static_assert(kSubscriptionCount == kLifecycleSubscriptionCount);
static_assert(kServiceCount == kLifecycleServiceCount);

struct SubscriptionMessages {
  mentor_pi_interfaces__msg__MotorCommand motors{};
  mentor_pi_interfaces__msg__PwmServoCommand pwm_servos{};
  mentor_pi_interfaces__msg__BusServoCommand bus_servos{};
  mentor_pi_interfaces__msg__LedCommand leds{};
  mentor_pi_interfaces__msg__BuzzerCommand buzzer{};
  mentor_pi_interfaces__msg__RgbCommand rgb{};
  mentor_pi_interfaces__msg__OledCommand oled{};
  std::array<char, mentor_pi::mcu::kOledLineCapacity + 1U> oled_line_1{};
  std::array<char, mentor_pi::mcu::kOledLineCapacity + 1U> oled_line_2{};
};

struct PublicationMessages {
  mentor_pi_interfaces__msg__MotorState motors{};
  mentor_pi_interfaces__msg__PwmServoState pwm_servos{};
  mentor_pi_interfaces__msg__ImuState imu{};
  mentor_pi_interfaces__msg__ButtonEvent button{};
  mentor_pi_interfaces__msg__BatteryState battery{};
  mentor_pi_interfaces__msg__Heartbeat heartbeat{};
  mentor_pi_interfaces__msg__ControllerDiagnostics diagnostics{};
};

struct ServiceMessages {
  mentor_pi_interfaces__srv__SetMotorModel_Request motor_model_request{};
  mentor_pi_interfaces__srv__SetMotorModel_Response motor_model_response{};
  mentor_pi_interfaces__srv__SetMotorAdrc_Request motor_adrc_request{};
  mentor_pi_interfaces__srv__SetMotorAdrc_Response motor_adrc_response{};
  mentor_pi_interfaces__srv__SetPwmServoOffsets_Request pwm_offsets_request{};
  mentor_pi_interfaces__srv__SetPwmServoOffsets_Response pwm_offsets_response{};
  mentor_pi_interfaces__srv__GetBusServoState_Request bus_get_request{};
  mentor_pi_interfaces__srv__GetBusServoState_Response bus_get_response{};
  mentor_pi_interfaces__srv__ConfigureBusServo_Request bus_configure_request{};
  mentor_pi_interfaces__srv__ConfigureBusServo_Response
      bus_configure_response{};
  mentor_pi_interfaces__srv__StopBusServos_Request bus_stop_request{};
  mentor_pi_interfaces__srv__StopBusServos_Response bus_stop_response{};
  mentor_pi_interfaces__srv__SetBatteryThreshold_Request
      battery_threshold_request{};
  mentor_pi_interfaces__srv__SetBatteryThreshold_Response
      battery_threshold_response{};
};

struct PendingServiceSlot {
  rmw_request_id_t request_id{};
  ServiceToken token{};
  std::uint32_t deadline_ms{0U};
  bool occupied{false};
  bool response_sent{false};
};

struct PendingBusServiceSlot : PendingServiceSlot {
  BusServiceKind kind{BusServiceKind::kNone};
};

struct RuntimeCounters {
  std::uint32_t command_messages{0U};
  std::uint32_t command_rejections{0U};
  std::array<std::uint32_t, kSubscriptionCount> mailbox_overwrites{};
  std::uint32_t publication_errors{0U};
  std::uint32_t service_requests{0U};
  std::uint32_t service_completions{0U};
  std::uint32_t service_busy_rejections{0U};
  std::uint32_t service_timeouts{0U};
  std::uint32_t service_partial_results{0U};
  std::uint32_t late_response_drops{0U};
  std::uint32_t executor_overruns{0U};
  std::array<std::uint32_t, kUsartErrorCount> usart1_errors{};
  std::uint32_t transport_rx_overruns{0U};
  std::uint32_t transport_tx_timeouts{0U};
  std::uint32_t last_error_uptime_ms{0U};
  std::uint16_t last_error_detail{0U};
  std::uint8_t last_error_code{0U};
  ErrorSource last_error_source{ErrorSource::kNone};
};

class MicroRosRuntime {
 public:
  bool Configure(const RuntimeHooks& hooks);
  void RunOnce();
  [[noreturn]] void RunForever();
  SessionState state() const { return lifecycle_.state(); }

#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
  bool ConfigureMiddlewareFaultForTesting(const MiddlewareFaultPlan& plan) {
    return middleware_fault_proxy_.Configure(plan);
  }
  void ClearMiddlewareFaultForTesting() {
    middleware_fault_proxy_.ClearFault();
  }
  MiddlewareBoundaryStats MiddlewareStatsForTesting(
      MiddlewareBoundary boundary) const {
    return middleware_fault_proxy_.stats(boundary);
  }
#endif

  void OnMotorCommand(const mentor_pi_interfaces__msg__MotorCommand& message);
  void OnPwmServoCommand(
      const mentor_pi_interfaces__msg__PwmServoCommand& message);
  void OnBusServoCommand(
      const mentor_pi_interfaces__msg__BusServoCommand& message);
  void OnLedCommand(const mentor_pi_interfaces__msg__LedCommand& message);
  void OnBuzzerCommand(const mentor_pi_interfaces__msg__BuzzerCommand& message);
  void OnRgbCommand(const mentor_pi_interfaces__msg__RgbCommand& message);
  void OnOledCommand(const mentor_pi_interfaces__msg__OledCommand& message);

 private:
  void SafeBootStep(std::uint32_t now_ms);
  void WaitAgentStep(std::uint32_t now_ms);
  void CreateEntitiesStep(std::uint32_t now_ms);
  void ActiveStep(std::uint32_t now_ms);
  void TeardownStep(std::uint32_t now_ms);
  void BackoffStep(std::uint32_t now_ms);
  void RunOneMaintenanceOperation(std::uint32_t now_ms);

  void PrepareCreateEntities(std::uint32_t now_ms);
  std::int32_t ExecuteCreateStep(const EntityCreateStep& step);
  void FailCreateEntities(mentor_pi::mcu::Result result);
  std::int32_t CreatePublisher(
      std::size_t index, const rosidl_message_type_support_t* type_support,
      const TopicEndpoint& endpoint);
  std::int32_t SetPublisherTimeout(std::size_t index);
  std::int32_t CreateSubscription(
      std::size_t index, const rosidl_message_type_support_t* type_support,
      const TopicEndpoint& endpoint);
  std::int32_t CreateService(std::size_t index,
                             const rosidl_service_type_support_t* type_support,
                             const ServiceEndpoint& endpoint);
  std::int32_t SetServiceTimeout(std::size_t index);
  std::int32_t AddExecutorSubscription(std::size_t index);
  void PrepareDestroyEntities(std::uint32_t now_ms);
  bool DestroyStepIsApplicable(const EntityDestroyStep& step) const;
  rcl_ret_t ExecuteDestroyStep(const EntityDestroyStep& step);
  void RecordDestroyResult(rcl_ret_t result);
  void CompleteDestroyEntities();
  void InitializeRosStorage();
  void InitializeSchedules(std::uint32_t now_ms);
  bool CanStartBoundedBlockingOperation(std::uint32_t now_ms) const;

  void PumpServices(std::uint32_t now_ms);
  bool TakeOneServiceRequest(std::uint32_t now_ms);
  bool TakeOneBusRequest(std::uint32_t now_ms);
  rcl_ret_t TakeServiceRequest(std::size_t service_index,
                               rmw_request_id_t* request_id, void* request);
  bool TakeMotorModelRequest(std::uint32_t now_ms);
  bool TakePwmOffsetsRequest(std::uint32_t now_ms);
  bool TakeBatteryThresholdRequest(std::uint32_t now_ms);
  bool TakeBusStopRequest(std::uint32_t now_ms);
  bool TakeBusGetRequest(std::uint32_t now_ms);
  bool TakeBusConfigureRequest(std::uint32_t now_ms);
  bool TakeMotorAdrcRequest(std::uint32_t now_ms);
  bool PollOneServiceCompletion(std::uint32_t now_ms);
  bool PollMotorModelCompletion(std::uint32_t now_ms);
  bool PollMotorAdrcCompletion(std::uint32_t now_ms);
  bool PollPwmOffsetsCompletion(std::uint32_t now_ms);
  bool PollBatteryThresholdCompletion(std::uint32_t now_ms);
  bool PollBusCompletion(std::uint32_t now_ms);
  bool SendServiceResponse(std::size_t service_index,
                           rmw_request_id_t* request_id, void* response,
                           mentor_pi::mcu::Result result);

  void PublishDueTelemetry(std::uint32_t now_ms, bool allow_reliable);
  void PublishOneDueBestEffortTelemetry(std::uint32_t now_ms);
  void PublishMotorState(std::uint32_t now_ms);
  void PublishPwmServoState(std::uint32_t now_ms);
  void PublishImuState(std::uint32_t now_ms);
  void PublishOneDueReliableTelemetry(std::uint32_t now_ms);
  bool PublishButtonEvent(std::uint32_t now_ms);
  bool PublishBatteryState(std::uint32_t now_ms);
  bool PublishHeartbeat(std::uint32_t now_ms);
  bool PublishDiagnostics(std::uint32_t now_ms);
  bool Publish(std::size_t publisher_index, const void* message, bool reliable,
               bool proves_agent_liveness = false);

  bool SynchronizeTime(std::uint32_t now_ms, std::uint32_t timeout_ms,
                       bool initial_attempt);
  void SetStamp(std::uint32_t source_ms, std::size_t publisher_index,
                builtin_interfaces__msg__Time* stamp);
  std::uint64_t ExtendTimestamp(std::uint32_t timestamp_ms) const;
  void UpdateExtendedMonotonic(std::uint32_t now_ms);

  void RequestTeardown(TeardownReason reason, ErrorSource source,
                       mentor_pi::mcu::Result result);
  void RecordCommandResult(std::size_t subscription_index,
                           mentor_pi::mcu::Result result, bool overwrote_unread,
                           ErrorSource source);
  void RecordTransportFault(std::uint8_t flags);
  void ClearServiceSlots();
  ServiceToken NewServiceToken();
  bool IsCurrentToken(ServiceToken token) const;
  void AdvanceHeartbeat() const;
  std::uint32_t NowMs() const;
  std::uint32_t NowUs() const;

  template <typename Operation>
  std::int32_t InvokeMiddleware(MiddlewareBoundary boundary,
                                std::uint32_t deadline_ms,
                                Operation operation) {
#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
    return middleware_fault_proxy_.Invoke(
        boundary, deadline_ms, operation, [this]() { return NowMs(); },
        [this](std::uint32_t wait_ms) {
          hooks_.wait_milliseconds(hooks_.context, wait_ms);
          AdvanceHeartbeat();
        });
#else
    static_cast<void>(boundary);
    static_cast<void>(deadline_ms);
    return operation();
#endif
  }

  RuntimeHooks hooks_{};
  SessionLifecycle lifecycle_{};
  EntityCreateCursor create_cursor_{};
  EntityDestroyCursor destroy_cursor_{};
#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
  MiddlewareFaultProxy middleware_fault_proxy_{};
#endif
  ArenaAllocator arena_{};
  rcl_allocator_t allocator_{};
  rclc_support_t support_{};
  rcl_node_t node_{};
  std::array<rcl_publisher_t, kPublisherCount> publishers_{};
  std::array<rcl_subscription_t, kSubscriptionCount> subscriptions_{};
  std::array<rcl_service_t, kServiceCount> services_{};
  rclc_executor_t executor_{};
  SubscriptionMessages subscription_messages_{};
  PublicationMessages publication_messages_{};
  ServiceMessages service_messages_{};
  PendingServiceSlot motor_model_slot_{};
  PendingServiceSlot motor_adrc_slot_{};
  PendingServiceSlot pwm_offsets_slot_{};
  PendingServiceSlot battery_threshold_slot_{};
  PendingBusServiceSlot bus_service_slot_{};
  RuntimeCounters counters_{};
  MotorTelemetry motor_telemetry_cache_{};
  PwmServoTelemetry pwm_telemetry_cache_{};
  ImuTelemetry imu_telemetry_cache_{};
  BatteryTelemetry battery_telemetry_cache_{};

  std::array<bool, kPublisherCount> publisher_constructed_{};
  std::array<bool, kSubscriptionCount> subscription_constructed_{};
  std::array<bool, kServiceCount> service_constructed_{};
  bool support_constructed_{false};
  bool node_constructed_{false};
  bool executor_constructed_{false};
  bool configured_{false};
  bool fatal_memory_violation_{false};
  bool has_motor_telemetry_{false};
  bool has_pwm_telemetry_{false};
  bool has_imu_sample_{false};
  bool initial_invalid_imu_published_{false};
  bool has_battery_telemetry_{false};
  bool time_synchronized_{false};
  bool time_offset_initialized_{false};
  bool time_sync_retry_pending_{false};
  bool alternate_bus_get_first_{true};
  std::uint8_t accounted_transport_flags_{0U};
  std::uint32_t request_generation_{0U};
  std::uint32_t heartbeat_sequence_{0U};
  std::uint32_t last_monotonic_ms_{0U};
  std::uint64_t extended_monotonic_ms_{0U};
  std::int64_t epoch_offset_ns_{0};
  std::array<std::int64_t, kPublisherCount> last_stamp_ns_{};
  std::uint32_t last_time_sync_attempt_ms_{0U};
  std::uint32_t last_time_sync_success_ms_{0U};
  std::uint32_t last_motor_publish_ms_{0U};
  std::uint32_t last_pwm_publish_ms_{0U};
  std::uint32_t last_imu_publish_ms_{0U};
  std::uint32_t last_button_publish_ms_{0U};
  std::uint32_t last_battery_publish_ms_{0U};
  std::uint32_t last_heartbeat_publish_ms_{0U};
  std::uint32_t last_diagnostics_publish_ms_{0U};
  ActiveSliceBudget active_slice_budget_{};
  ActiveWorkScheduler active_work_scheduler_{};
  RoundRobinCursor<kServiceSlotCount> service_completion_cursor_{};
  RoundRobinCursor<kServiceRequestGroupCount> service_request_cursor_{};
  RoundRobinCursor<kBusServiceEndpointCount> busy_bus_request_cursor_{};
  RoundRobinCursor<kBestEffortPublisherCount> best_effort_publisher_cursor_{};
  RoundRobinCursor<kReliablePublisherCount> reliable_publisher_cursor_{};
};

MicroRosRuntime& RuntimeInstance();

void MotorSubscriptionCallback(const void* message);
void PwmServoSubscriptionCallback(const void* message);
void BusServoSubscriptionCallback(const void* message);
void LedSubscriptionCallback(const void* message);
void BuzzerSubscriptionCallback(const void* message);
void RgbSubscriptionCallback(const void* message);
void OledSubscriptionCallback(const void* message);

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_SRC_RUNTIME_INTERNAL_H_
