#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <limits>

#include "mentor_pi_interfaces/srv/set_motor_adrc.h"
#include "mentor_pi_mcu/app/controller/controller_runtime.h"
#include "mentor_pi_mcu/app/controller/imu_characterization.h"
#include "mentor_pi_mcu/app/microros/runtime_hooks.h"
#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/validation.h"
#include "motor_adrc_service_adapter.h"

namespace {

using mentor_pi::mcu::BusServoCommand;
using mentor_pi::mcu::BusServoFrame;
using mentor_pi::mcu::BusServoOpcode;
using mentor_pi::mcu::BuzzerCommand;
using mentor_pi::mcu::ConfigureBusServoCommand;
using mentor_pi::mcu::DefaultAdrcMotorControlConfiguration;
using mentor_pi::mcu::GetBusServoStateCommand;
using mentor_pi::mcu::LedCommand;
using mentor_pi::mcu::MotorCommand;
using mentor_pi::mcu::MotorControlConfiguration;
using mentor_pi::mcu::MotorModel;
using mentor_pi::mcu::OkResult;
using mentor_pi::mcu::OledCommand;
using mentor_pi::mcu::PwmServoCommand;
using mentor_pi::mcu::PwmServoOffsetCommand;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi::mcu::RgbCommand;
using mentor_pi::mcu::SetMotorAdrcCommand;
using mentor_pi::mcu::StopBusServosCommand;
using mentor_pi::mcu::drivers::AxisTransform;
using mentor_pi::mcu::drivers::IoStatus;
using mentor_pi_mcu::app::controller::BatterySample;
using mentor_pi_mcu::app::controller::ControllerRuntime;
using mentor_pi_mcu::app::controller::ControllerTask;
using mentor_pi_mcu::app::controller::MicroRosHeartbeatController;
using mentor_pi_mcu::app::controller::PlatformHooks;
using mentor_pi_mcu::app::controller::PlatformHooksAreComplete;
using mentor_pi_mcu::app::controller::StatusRgbColor;
using mentor_pi_mcu::app::controller::StatusRgbController;
using mentor_pi_mcu::app::controller::TransportActivity;
using mentor_pi_mcu::app::controller::UpdateImuCharacterizationSnapshot;
using mentor_pi_mcu::app::microros::BatteryTelemetry;
using mentor_pi_mcu::app::microros::BatteryThresholdReply;
using mentor_pi_mcu::app::microros::ConfigureBusServoReply;
using mentor_pi_mcu::app::microros::GetBusServoStateReply;
using mentor_pi_mcu::app::microros::HealthSnapshot;
using mentor_pi_mcu::app::microros::ImuTelemetry;
using mentor_pi_mcu::app::microros::MotorAdrcReply;
using mentor_pi_mcu::app::microros::MotorModelReply;
using mentor_pi_mcu::app::microros::MotorTelemetry;
using mentor_pi_mcu::app::microros::PwmOffsetsReply;
using mentor_pi_mcu::app::microros::PwmServoTelemetry;
using mentor_pi_mcu::app::microros::ServiceToken;
using mentor_pi_mcu::app::microros::StopBusServosReply;
using mentor_pi_mcu::app::microros::WorkerDiagnostics;

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                               \
      return false;                                                           \
    }                                                                         \
  } while (false)

MotorControlConfiguration FullRangeTestMotorConfiguration() {
  return DefaultAdrcMotorControlConfiguration();
}

SetMotorAdrcCommand DefaultMotorAdrcCommand(std::uint8_t update_mask) {
  const mentor_pi::mcu::AdrcCalibration defaults{};
  SetMotorAdrcCommand command{};
  command.update_mask = update_mask;
  command.known_velocity_decay_rate_s_inverse.fill(
      defaults.known_velocity_decay_rate_s_inverse);
  command.input_gain_rps_per_second_per_permille.fill(
      defaults.input_gain_rps_per_second_per_permille);
  command.controller_bandwidth_rad_s.fill(
      defaults.controller_bandwidth_rad_s);
  command.controller_fal_exponent.fill(defaults.controller_fal_exponent);
  command.controller_fal_threshold_rps.fill(
      defaults.controller_fal_threshold_rps);
  command.observer_bandwidth_rad_s.fill(defaults.observer_bandwidth_rad_s);
  command.observer_velocity_fal_exponent.fill(
      defaults.observer_velocity_fal_exponent);
  command.observer_disturbance_fal_exponent.fill(
      defaults.observer_disturbance_fal_exponent);
  command.observer_fal_threshold_rps.fill(
      defaults.observer_fal_threshold_rps);
  command.disturbance_leakage_s_inverse.fill(
      defaults.disturbance_leakage_s_inverse);
  command.disturbance_estimate_limit_rps_per_second.fill(
      defaults.disturbance_estimate_limit_rps_per_second);
  command.velocity_filter_new_weight.fill(
      defaults.velocity_filter_new_weight);
  command.positive_minimum_drive_permille.fill(
      defaults.positive_minimum_drive_permille);
  command.negative_minimum_drive_permille.fill(
      defaults.negative_minimum_drive_permille);
  return command;
}

bool TestMotorAdrcWireAdapter() {
  mentor_pi_interfaces__srv__SetMotorAdrc_Request request{};
  request.update_mask = 0x0fU;
  for (std::size_t index = 0U; index < 4U; ++index) {
    const float offset = static_cast<float>(index);
    request.known_velocity_decay_rate_s_inverse[index] = 1.0F + offset;
    request.input_gain_rps_per_second_per_permille[index] =
        0.05F + 0.01F * offset;
    request.controller_bandwidth_rad_s[index] = 4.0F + offset;
    request.controller_fal_exponent[index] = 0.55F + 0.05F * offset;
    request.controller_fal_threshold_rps[index] = 0.05F + 0.01F * offset;
    request.observer_bandwidth_rad_s[index] = 12.0F + offset;
    request.observer_velocity_fal_exponent[index] = 0.70F + 0.05F * offset;
    request.observer_disturbance_fal_exponent[index] =
        0.45F + 0.05F * offset;
    request.observer_fal_threshold_rps[index] = 0.09F + 0.01F * offset;
    request.disturbance_leakage_s_inverse[index] = 0.1F + 0.1F * offset;
    request.disturbance_estimate_limit_rps_per_second[index] =
        10.0F + 10.0F * offset;
    request.velocity_filter_new_weight[index] = 0.2F + 0.2F * offset;
    request.positive_minimum_drive_permille[index] =
        static_cast<std::uint16_t>(81U + index);
    request.negative_minimum_drive_permille[index] =
        static_cast<std::uint16_t>(91U + index);
  }

  SetMotorAdrcCommand decoded{};
  const Result decoded_result =
      mentor_pi_mcu::app::microros::DecodeMotorAdrcRequest(request, &decoded);
  CHECK(decoded_result.ok());
  CHECK(decoded.update_mask == request.update_mask);
  for (std::size_t index = 0U; index < 4U; ++index) {
    CHECK(decoded.known_velocity_decay_rate_s_inverse[index] ==
          request.known_velocity_decay_rate_s_inverse[index]);
    CHECK(decoded.input_gain_rps_per_second_per_permille[index] ==
          request.input_gain_rps_per_second_per_permille[index]);
    CHECK(decoded.controller_bandwidth_rad_s[index] ==
          request.controller_bandwidth_rad_s[index]);
    CHECK(decoded.controller_fal_exponent[index] ==
          request.controller_fal_exponent[index]);
    CHECK(decoded.controller_fal_threshold_rps[index] ==
          request.controller_fal_threshold_rps[index]);
    CHECK(decoded.observer_bandwidth_rad_s[index] ==
          request.observer_bandwidth_rad_s[index]);
    CHECK(decoded.observer_velocity_fal_exponent[index] ==
          request.observer_velocity_fal_exponent[index]);
    CHECK(decoded.observer_disturbance_fal_exponent[index] ==
          request.observer_disturbance_fal_exponent[index]);
    CHECK(decoded.observer_fal_threshold_rps[index] ==
          request.observer_fal_threshold_rps[index]);
    CHECK(decoded.disturbance_leakage_s_inverse[index] ==
          request.disturbance_leakage_s_inverse[index]);
    CHECK(decoded.disturbance_estimate_limit_rps_per_second[index] ==
          request.disturbance_estimate_limit_rps_per_second[index]);
    CHECK(decoded.velocity_filter_new_weight[index] ==
          request.velocity_filter_new_weight[index]);
    CHECK(decoded.positive_minimum_drive_permille[index] ==
          request.positive_minimum_drive_permille[index]);
    CHECK(decoded.negative_minimum_drive_permille[index] ==
          request.negative_minimum_drive_permille[index]);
  }

  mentor_pi_interfaces__srv__SetMotorAdrc_Response response{};
  response.result.code = 255U;
  response.result.detail = 65535U;
  response.applied_mask = 0xffU;
  mentor_pi_mcu::app::microros::EncodeMotorAdrcResponse(
      OkResult(), 0x05U, &response);
  CHECK(response.result.code ==
        static_cast<std::uint8_t>(ResultCode::kOk));
  CHECK(response.result.detail == 0U);
  CHECK(response.applied_mask == 0x05U);

  request.controller_fal_threshold_rps[2] = 0.0F;
  const Result invalid_result =
      mentor_pi_mcu::app::microros::DecodeMotorAdrcRequest(request, &decoded);
  CHECK(invalid_result.code == ResultCode::kOutOfRange);
  CHECK(invalid_result.detail == 3U);
  mentor_pi_mcu::app::microros::EncodeMotorAdrcResponse(
      invalid_result, 0x05U, &response);
  CHECK(response.result.code ==
        static_cast<std::uint8_t>(ResultCode::kOutOfRange));
  CHECK(response.result.detail == 3U);
  CHECK(response.applied_mask == 0U);
  return true;
}

struct FakePlatform {
  std::uint32_t now_ms{0U};
  std::uint32_t now_us{0U};
  std::uint32_t pwm_sequence{0U};
  std::uint32_t watchdog_refreshes{0U};
  std::uint32_t persist_watchdog_task_calls{0U};
  std::uint32_t emergency_stops{0U};
  std::uint32_t critical_depth{0U};
  std::uint32_t motor_arm_calls{0U};
  std::uint32_t motor_apply_calls{0U};
  std::uint32_t bus_exchanges{0U};
  std::uint32_t rgb_transfers{0U};
  std::uint64_t transport_rx_wire_bytes{0U};
  std::uint64_t transport_tx_wire_bytes{0U};
  std::uint32_t buzzer_calls{0U};
  std::uint32_t battery_samples{0U};
  std::uint32_t button_reads{0U};
  std::array<std::uint32_t, 4> encoders{};
  std::array<std::uint32_t, 4> motor_arm_calls_by_channel{};
  std::array<std::int16_t, 4> motor_duty{};
  std::array<bool, 4> motor_armed{};
  std::array<std::uint16_t, 4> pwm_shadow{1500U, 1500U, 1500U, 1500U};
  std::array<std::uint16_t, 4> pwm_active{1500U, 1500U, 1500U, 1500U};
  mentor_pi::mcu::drivers::RgbEncodedFrame rgb_frame{};
  std::array<std::uint8_t, mentor_pi::mcu::drivers::kSsd1306FramebufferSize>
      oled_framebuffer{};
  std::array<bool, 3> leds{};
  std::array<bool, 2> buttons{};
  std::uint16_t buzzer_hz{0U};
  std::uint16_t battery_mv{6200U};
  std::uint8_t reset_reason{0U};
  std::uint8_t captured_watchdog_task{255U};
  std::uint8_t persisted_watchdog_task{255U};
  ControllerRuntime* stop_probe_runtime{nullptr};
  ControllerRuntime* critical_transition_runtime{nullptr};
  Result arm_result{};
  Result apply_result{};
  Result buzzer_result{};
  IoStatus imu_identity_read_status{IoStatus::kOk};
  IoStatus imu_status_read_status{IoStatus::kOk};
  bool imu_data_ready{true};
  std::uint32_t imu_reset_writes{0U};
  IoStatus bus_begin_status{IoStatus::kOk};
  IoStatus bus_poll_status{IoStatus::kOk};
  bool pwm_started{false};
  bool read_encoders_ok{true};
  bool bus_active{false};
  bool bus_hold_active{false};
  bool bus_expects_reply{false};
  bool rgb_active{false};
  bool rgb_hold_active{false};
  bool transport_open{false};
  bool stop_probe_enabled{false};
  bool stop_observed_inactive{false};
  bool stop_observed_inside_critical{false};
  bool arm_observed_inside_critical{false};
  bool apply_observed_inside_critical{false};
  bool transition_session_on_critical_enter{false};
  std::uint32_t transition_from_generation{0U};
  std::uint32_t transition_to_generation{0U};
  std::array<std::uint8_t, 14> bus_request{};
  std::size_t bus_request_size{0U};
  std::size_t oled_framebuffer_offset{0U};

  void SetTimeMs(std::uint32_t value) {
    now_ms = value;
    now_us = value * 1000U;
  }

  // Models BeginServoFrameFromIsr(): the complete task-prepared shadow is
  // copied to the physical active frame before the sequence becomes visible.
  void CommitPwmFrame(std::uint32_t value_ms) {
    SetTimeMs(value_ms);
    pwm_active = pwm_shadow;
    ++pwm_sequence;
  }

  static FakePlatform* Self(void* context) {
    return static_cast<FakePlatform*>(context);
  }

  static std::uint32_t Milliseconds(void* context) {
    return Self(context)->now_ms;
  }
  static std::uint32_t Microseconds(void* context) {
    return Self(context)->now_us;
  }
  static void Wait(void*, ControllerTask, std::uint32_t) {}
  static void EnterCritical(void* context) {
    FakePlatform* self = Self(context);
    ++self->critical_depth;
    if (self->transition_session_on_critical_enter &&
        self->critical_transition_runtime != nullptr) {
      self->transition_session_on_critical_enter = false;
      self->critical_transition_runtime->SetSessionActive(
          false, self->transition_from_generation);
      if (self->transition_to_generation != 0U) {
        self->critical_transition_runtime->SetSessionActive(
            true, self->transition_to_generation);
      }
    }
  }
  static void ExitCritical(void* context) { --Self(context)->critical_depth; }
  static void EmergencyStop(void* context) {
    FakePlatform* self = Self(context);
    ++self->emergency_stops;
    if (self->stop_probe_enabled && self->stop_probe_runtime != nullptr) {
      MotorCommand stop{};
      stop.update_mask = 1U;
      self->stop_observed_inactive =
          self->stop_probe_runtime->PublishMotorCommand(stop, self->now_us)
              .result.code == ResultCode::kBusy;
      self->stop_observed_inside_critical = self->critical_depth != 0U;
    }
    self->motor_duty.fill(0);
    self->motor_armed.fill(false);
  }
  static Result RefreshWatchdog(void* context) {
    ++Self(context)->watchdog_refreshes;
    return OkResult();
  }
  static void PersistWatchdogTask(void* context, ControllerTask task) {
    FakePlatform* self = Self(context);
    ++self->persist_watchdog_task_calls;
    if (self->persisted_watchdog_task == 255U) {
      self->persisted_watchdog_task = static_cast<std::uint8_t>(task);
    }
  }
  static Result InitializeMotors(void* context) {
    EmergencyStop(context);
    return OkResult();
  }
  static Result ArmMotor(void* context, std::size_t motor) {
    FakePlatform* self = Self(context);
    ++self->motor_arm_calls;
    ++self->motor_arm_calls_by_channel[motor];
    self->arm_observed_inside_critical = self->critical_depth != 0U;
    if (!self->arm_result.ok()) {
      return self->arm_result;
    }
    self->motor_armed[motor] = true;
    return OkResult();
  }
  static void DisarmMotor(void* context, std::size_t motor) {
    FakePlatform* self = Self(context);
    self->motor_armed[motor] = false;
    self->motor_duty[motor] = 0;
  }
  static bool ReadEncoders(void* context,
                           std::array<std::uint32_t, 4>* counters) {
    FakePlatform* self = Self(context);
    *counters = self->encoders;
    return self->read_encoders_ok;
  }
  static Result ApplyDuty(void* context,
                          const std::array<std::int16_t, 4>& duty) {
    FakePlatform* self = Self(context);
    ++self->motor_apply_calls;
    self->apply_observed_inside_critical = self->critical_depth != 0U;
    if (!self->apply_result.ok()) {
      return self->apply_result;
    }
    self->motor_duty = duty;
    return OkResult();
  }
  static Result InitializePwm(void* context) {
    Self(context)->pwm_started = true;
    return OkResult();
  }
  static Result SetPwmShadow(void* context,
                             const std::array<std::uint16_t, 4>& pulses) {
    Self(context)->pwm_shadow = pulses;
    return OkResult();
  }
  static std::uint32_t PwmSequence(void* context) {
    return Self(context)->pwm_sequence;
  }
  static bool ReadButton(void* context, std::size_t button) {
    FakePlatform* self = Self(context);
    ++self->button_reads;
    return self->buttons[button];
  }
  static void SetLed(void* context, std::size_t led, bool on) {
    Self(context)->leds[led] = on;
  }
  static Result SetBuzzer(void* context, std::uint16_t frequency_hz, bool on) {
    FakePlatform* self = Self(context);
    ++self->buzzer_calls;
    if (!self->buzzer_result.ok()) {
      return self->buzzer_result;
    }
    self->buzzer_hz = on ? frequency_hz : 0U;
    return OkResult();
  }
  static BatterySample TakeBattery(void* context, std::uint32_t) {
    ++Self(context)->battery_samples;
    BatterySample sample{};
    sample.result = OkResult();
    sample.reading = {Self(context)->battery_mv, true};
    sample.available = true;
    return sample;
  }
  static IoStatus RegisterRead(void* context, std::uint8_t address,
                               std::uint8_t reg, std::uint8_t* data,
                               std::size_t size, std::uint32_t) {
    if (address != 0x6aU) {
      return IoStatus::kIoError;
    }
    if (reg == 0U && Self(context)->imu_identity_read_status != IoStatus::kOk) {
      return Self(context)->imu_identity_read_status;
    }
    if (reg == 46U && Self(context)->imu_status_read_status != IoStatus::kOk) {
      return Self(context)->imu_status_read_status;
    }
    for (std::size_t index = 0; index < size; ++index) {
      data[index] = 0U;
    }
    if (reg == 0U) {
      data[0] = 0x05U;
    } else if (reg == 1U) {
      data[0] = 1U;
    } else if (reg == 46U) {
      data[0] = Self(context)->imu_data_ready ? 3U : 0U;
    }
    return IoStatus::kOk;
  }
  static IoStatus RegisterWrite(void* context, std::uint8_t, std::uint8_t reg,
                                const std::uint8_t* data, std::size_t size,
                                std::uint32_t) {
    if (reg == 96U && data != nullptr && size == 1U && data[0] == 0xb0U) {
      ++Self(context)->imu_reset_writes;
    }
    return IoStatus::kOk;
  }
  static IoStatus RawWrite(void* context, std::uint8_t,
                           const std::uint8_t* data, std::size_t size,
                           std::uint32_t) {
    FakePlatform* self = Self(context);
    if (data == nullptr) {
      return IoStatus::kIoError;
    }
    if (size == 7U && data[0] == 0x00U && data[1] == 0x21U) {
      self->oled_framebuffer_offset = 0U;
    } else if (size == 33U && data[0] == 0x40U &&
               self->oled_framebuffer_offset + 32U <=
                   self->oled_framebuffer.size()) {
      for (std::size_t index = 0U; index < 32U; ++index) {
        self->oled_framebuffer[self->oled_framebuffer_offset + index] =
            data[index + 1U];
      }
      self->oled_framebuffer_offset += 32U;
    }
    return IoStatus::kOk;
  }
  static IoStatus BusBegin(void* context, const std::uint8_t* tx,
                           std::size_t tx_size, std::size_t max_reply_size,
                           std::uint32_t) {
    FakePlatform* self = Self(context);
    if (self->bus_begin_status != IoStatus::kOk) {
      return self->bus_begin_status;
    }
    if (self->bus_active) {
      return IoStatus::kBusy;
    }
    for (std::size_t index = 0; index < tx_size; ++index) {
      self->bus_request[index] = tx[index];
    }
    self->bus_request_size = tx_size;
    self->bus_expects_reply = max_reply_size != 0U;
    self->bus_active = true;
    ++self->bus_exchanges;
    return IoStatus::kOk;
  }
  static IoStatus BusPoll(void* context, std::uint32_t, std::uint8_t* reply,
                          std::size_t capacity, std::size_t* reply_size) {
    FakePlatform* self = Self(context);
    if (!self->bus_active) {
      return IoStatus::kIoError;
    }
    if (self->bus_hold_active) {
      return IoStatus::kBusy;
    }
    if (self->bus_poll_status != IoStatus::kOk) {
      self->bus_active = false;
      return self->bus_poll_status;
    }
    self->bus_active = false;
    if (!self->bus_expects_reply) {
      *reply_size = 0U;
      return IoStatus::kOk;
    }
    const auto opcode = static_cast<BusServoOpcode>(self->bus_request[4]);
    std::array<std::uint8_t, 4> arguments{};
    std::size_t argument_count = 1U;
    switch (opcode) {
      case BusServoOpcode::kIdRead:
        arguments[0] = self->bus_request[2];
        break;
      case BusServoOpcode::kPositionRead:
        arguments[0] = 0xccU;
        arguments[1] = 0xffU;
        argument_count = 2U;
        break;
      case BusServoOpcode::kOffsetRead:
        arguments[0] = 0xf6U;
        break;
      case BusServoOpcode::kVoltageRead:
        arguments[0] = 0x20U;
        arguments[1] = 0x1cU;
        argument_count = 2U;
        break;
      case BusServoOpcode::kTemperatureRead:
        arguments[0] = 42U;
        break;
      case BusServoOpcode::kPositionLimitsRead:
        arguments = {100U, 0U, 0x84U, 0x03U};
        argument_count = 4U;
        break;
      case BusServoOpcode::kVoltageLimitsRead:
        arguments = {0x70U, 0x17U, 0x28U, 0x23U};
        argument_count = 4U;
        break;
      case BusServoOpcode::kTemperatureLimitRead:
        arguments[0] = 75U;
        break;
      case BusServoOpcode::kTorqueRead:
        arguments[0] = 1U;
        break;
      default:
        return IoStatus::kIoError;
    }
    BusServoFrame frame{};
    const Result built = mentor_pi::mcu::BusServoCodec::BuildFrame(
        self->bus_request[2], opcode, arguments.data(), argument_count, &frame);
    if (!built.ok() || frame.size > capacity) {
      return IoStatus::kIoError;
    }
    for (std::size_t index = 0; index < frame.size; ++index) {
      reply[index] = frame.bytes[index];
    }
    *reply_size = frame.size;
    return IoStatus::kOk;
  }
  static void BusCancel(void* context) { Self(context)->bus_active = false; }
  static IoStatus RgbBegin(void* context, const std::uint8_t* data,
                           std::size_t size, std::uint32_t) {
    FakePlatform* self = Self(context);
    if (self->rgb_active || data == nullptr || size != self->rgb_frame.size()) {
      return IoStatus::kBusy;
    }
    for (std::size_t index = 0U; index < size; ++index) {
      self->rgb_frame[index] = data[index];
    }
    self->rgb_active = true;
    ++self->rgb_transfers;
    return IoStatus::kOk;
  }
  static IoStatus RgbPoll(void* context, std::uint32_t) {
    FakePlatform* self = Self(context);
    if (!self->rgb_active) {
      return IoStatus::kOk;
    }
    if (self->rgb_hold_active) {
      return IoStatus::kBusy;
    }
    self->rgb_active = false;
    return IoStatus::kOk;
  }
  static void RgbCancel(void* context) { Self(context)->rgb_active = false; }
  static TransportActivity ReadTransportActivity(void* context) {
    FakePlatform* self = Self(context);
    return {self->transport_rx_wire_bytes, self->transport_tx_wire_bytes,
            self->transport_open};
  }
  static std::uint32_t StackHighWater(void*, ControllerTask) { return 512U; }
  static void Memory(void*, std::array<std::uint32_t, 2>* free_bytes,
                     std::array<std::uint32_t, 2>* minimum_bytes) {
    *free_bytes = {64000U, 32000U};
    *minimum_bytes = {60000U, 30000U};
  }
  static std::uint32_t FlashUsed(void*) { return 100000U; }
  static std::uint32_t FlashTotal(void*) { return 512U * 1024U; }
  static std::uint8_t ResetReason(void* context) {
    return Self(context)->reset_reason;
  }
  static std::uint8_t CapturedWatchdogTask(void* context) {
    return Self(context)->captured_watchdog_task;
  }

  PlatformHooks Hooks() {
    PlatformHooks hooks{};
    hooks.context = this;
    hooks.monotonic_milliseconds = &Milliseconds;
    hooks.monotonic_microseconds = &Microseconds;
    hooks.wait_for_task = &Wait;
    hooks.enter_critical = &EnterCritical;
    hooks.exit_critical = &ExitCritical;
    hooks.emergency_stop_motors = &EmergencyStop;
    hooks.refresh_watchdog = &RefreshWatchdog;
    hooks.persist_watchdog_task = &PersistWatchdogTask;
    hooks.initialize_motor_outputs = &InitializeMotors;
    hooks.arm_motor = &ArmMotor;
    hooks.disarm_motor = &DisarmMotor;
    hooks.read_encoder_counters = &ReadEncoders;
    hooks.apply_motor_duty = &ApplyDuty;
    hooks.initialize_pwm_servos = &InitializePwm;
    hooks.set_pwm_servo_shadow = &SetPwmShadow;
    hooks.pwm_servo_frame_sequence = &PwmSequence;
    hooks.read_button_pressed = &ReadButton;
    hooks.set_led = &SetLed;
    hooks.set_buzzer = &SetBuzzer;
    hooks.take_battery_sample = &TakeBattery;
    hooks.register_i2c_read = &RegisterRead;
    hooks.register_i2c_write = &RegisterWrite;
    hooks.raw_i2c_write = &RawWrite;
    hooks.bus_uart_begin_exchange = &BusBegin;
    hooks.bus_uart_poll_exchange = &BusPoll;
    hooks.bus_uart_cancel = &BusCancel;
    hooks.rgb_spi_begin_transmit = &RgbBegin;
    hooks.rgb_spi_poll_transmit = &RgbPoll;
    hooks.rgb_spi_cancel = &RgbCancel;
    hooks.read_transport_activity = &ReadTransportActivity;
    hooks.task_stack_high_water_bytes = &StackHighWater;
    hooks.read_memory_metrics = &Memory;
    hooks.flash_used_bytes = &FlashUsed;
    hooks.flash_total_bytes = &FlashTotal;
    hooks.last_reset_reason = &ResetReason;
    hooks.captured_watchdog_task = &CapturedWatchdogTask;
    return hooks;
  }
};

bool EstablishStartupReadiness(ControllerRuntime* runtime,
                               FakePlatform* platform) {
  runtime->RunMotorControlOnce();
  runtime->AdvanceMicroRosHeartbeat();
  runtime->RunBusServoOnce();
  runtime->RunSensorOnce();
  runtime->RunPeripheralOnce();
  runtime->RunSafetySupervisorOnce();
  return platform->watchdog_refreshes != 0U && platform->critical_depth == 0U;
}

bool TestStatusRgbSemantics() {
  MicroRosHeartbeatController heartbeat;
  CHECK(!heartbeat.Update(0U));
  CHECK(heartbeat.Update(1U));
  CHECK(!heartbeat.Update(2U));
  CHECK(heartbeat.Update(3U));
  CHECK(mentor_pi::mcu::ValidateLedCommand({1U, 1U, 1U, 1U}).code ==
        ResultCode::kOutOfRange);
  CHECK(mentor_pi::mcu::ValidateLedCommand({2U, 1U, 1U, 1U}).ok());
  CHECK(mentor_pi::mcu::ValidateLedCommand({3U, 1U, 1U, 1U}).ok());
  CHECK(heartbeat.Update(3U));

  StatusRgbController status;
  CHECK(status.TransportSampleDue(0U));
  status.ObserveTransport(0U, {10U, 20U, true});
  StatusRgbColor color = status.Update(0U);
  CHECK(color.red == 0U && color.green == 0U && color.blue == 0U);
  CHECK(!status.TransportSampleDue(99U));
  CHECK(status.TransportSampleDue(100U));

  status.ObserveTransport(100U, {11U, 20U, true});
  color = status.Update(100U);
  CHECK(color.red == 0U);
  CHECK(color.green == 0U);
  CHECK(color.blue == StatusRgbController::kBrightness);
  color = status.Update(149U);
  CHECK(color.red == 0U && color.green == 0U && color.blue != 0U);
  color = status.Update(150U);
  CHECK(color.red == 0U && color.green == 0U && color.blue == 0U);

  status.ObserveTransport(200U, {11U, 21U, true});
  color = status.Update(200U);
  CHECK(color.red == 0U);
  CHECK(color.green == StatusRgbController::kBrightness);
  CHECK(color.blue == 0U);
  color = status.Update(250U);
  CHECK(color.red == 0U && color.green == 0U && color.blue == 0U);

  status.ObserveTransport(300U, {12U, 22U, true});
  color = status.Update(300U);
  CHECK(color.red == 0U);
  CHECK(color.green == StatusRgbController::kBrightness);
  CHECK(color.blue == StatusRgbController::kBrightness);

  status.ObserveTransport(400U, {13U, 23U, false});
  color = status.Update(400U);
  CHECK(color.red == 0U && color.green == 0U && color.blue == 0U);

  StatusRgbController wrapping;
  wrapping.ObserveTransport(std::numeric_limits<std::uint32_t>::max() - 50U,
                            {0U, 0U, true});
  CHECK(wrapping.TransportSampleDue(49U));
  wrapping.ObserveTransport(std::numeric_limits<std::uint32_t>::max() - 40U,
                            {1U, 0U, true});
  color = wrapping.Update(8U);
  CHECK(color.blue == StatusRgbController::kBrightness);
  color = wrapping.Update(9U);
  CHECK(color.red == 0U && color.green == 0U && color.blue == 0U);
  return true;
}

bool TestStatusRgbControllerIntegration() {
  FakePlatform platform;
  platform.transport_open = true;
  platform.transport_rx_wire_bytes = 10U;
  platform.transport_tx_wire_bytes = 20U;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  platform.SetTimeMs(0U);
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == 1U);
  CHECK(!platform.leds[mentor_pi::mcu::kSystemHeartbeatLedIndex]);
  platform.SetTimeMs(1U);
  runtime.RunPeripheralOnce();

  platform.transport_rx_wire_bytes = 11U;
  platform.transport_tx_wire_bytes = 21U;
  platform.SetTimeMs(99U);
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == 1U);

  platform.SetTimeMs(100U);
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == 2U);
  mentor_pi::mcu::RgbState expected{};
  expected.green[mentor_pi::mcu::kStatusRgbPixelIndex] =
      StatusRgbController::kBrightness;
  expected.blue[mentor_pi::mcu::kStatusRgbPixelIndex] =
      StatusRgbController::kBrightness;
  CHECK(platform.rgb_frame ==
        mentor_pi::mcu::drivers::EncodeRgbFrame(expected));

  platform.SetTimeMs(101U);
  runtime.RunPeripheralOnce();
  runtime.RecordSuccessfulRosHeartbeat();
  platform.SetTimeMs(102U);
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == 3U);
  expected.red[mentor_pi::mcu::kStatusRgbPixelIndex] =
      StatusRgbController::kBrightness;
  CHECK(platform.rgb_frame ==
        mentor_pi::mcu::drivers::EncodeRgbFrame(expected));
  runtime.RecordSuccessfulRosHeartbeat();
  platform.SetTimeMs(103U);
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == 4U);
  expected.red[mentor_pi::mcu::kStatusRgbPixelIndex] = 0U;
  CHECK(platform.rgb_frame ==
        mentor_pi::mcu::drivers::EncodeRgbFrame(expected));
  CHECK(!platform.leds[mentor_pi::mcu::kSystemHeartbeatLedIndex]);
  platform.SetTimeMs(500U);
  runtime.RunPeripheralOnce();
  CHECK(platform.leds[mentor_pi::mcu::kSystemHeartbeatLedIndex]);
  platform.SetTimeMs(1000U);
  runtime.RunPeripheralOnce();
  CHECK(!platform.leds[mentor_pi::mcu::kSystemHeartbeatLedIndex]);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestPlatformHookCompletenessContract() {
  FakePlatform platform;
  const PlatformHooks complete = platform.Hooks();
  CHECK(PlatformHooksAreComplete(complete));

  const auto rejects_missing = [&complete](auto member) {
    PlatformHooks incomplete = complete;
    incomplete.*member = nullptr;
    return !PlatformHooksAreComplete(incomplete);
  };
  CHECK(rejects_missing(&PlatformHooks::monotonic_milliseconds));
  CHECK(rejects_missing(&PlatformHooks::monotonic_microseconds));
  CHECK(rejects_missing(&PlatformHooks::wait_for_task));
  CHECK(rejects_missing(&PlatformHooks::enter_critical));
  CHECK(rejects_missing(&PlatformHooks::exit_critical));
  CHECK(rejects_missing(&PlatformHooks::emergency_stop_motors));
  CHECK(rejects_missing(&PlatformHooks::refresh_watchdog));
  CHECK(rejects_missing(&PlatformHooks::persist_watchdog_task));
  CHECK(rejects_missing(&PlatformHooks::initialize_motor_outputs));
  CHECK(rejects_missing(&PlatformHooks::arm_motor));
  CHECK(rejects_missing(&PlatformHooks::disarm_motor));
  CHECK(rejects_missing(&PlatformHooks::read_encoder_counters));
  CHECK(rejects_missing(&PlatformHooks::apply_motor_duty));
  CHECK(rejects_missing(&PlatformHooks::initialize_pwm_servos));
  CHECK(rejects_missing(&PlatformHooks::set_pwm_servo_shadow));
  CHECK(rejects_missing(&PlatformHooks::pwm_servo_frame_sequence));
  CHECK(rejects_missing(&PlatformHooks::read_button_pressed));
  CHECK(rejects_missing(&PlatformHooks::set_led));
  CHECK(rejects_missing(&PlatformHooks::set_buzzer));
  CHECK(rejects_missing(&PlatformHooks::take_battery_sample));
  CHECK(rejects_missing(&PlatformHooks::register_i2c_read));
  CHECK(rejects_missing(&PlatformHooks::register_i2c_write));
  CHECK(rejects_missing(&PlatformHooks::raw_i2c_write));
  CHECK(rejects_missing(&PlatformHooks::bus_uart_begin_exchange));
  CHECK(rejects_missing(&PlatformHooks::bus_uart_poll_exchange));
  CHECK(rejects_missing(&PlatformHooks::bus_uart_cancel));
  CHECK(rejects_missing(&PlatformHooks::rgb_spi_begin_transmit));
  CHECK(rejects_missing(&PlatformHooks::rgb_spi_poll_transmit));
  CHECK(rejects_missing(&PlatformHooks::rgb_spi_cancel));
  CHECK(rejects_missing(&PlatformHooks::read_transport_activity));
  CHECK(rejects_missing(&PlatformHooks::task_stack_high_water_bytes));
  CHECK(rejects_missing(&PlatformHooks::read_memory_metrics));
  CHECK(rejects_missing(&PlatformHooks::flash_used_bytes));
  CHECK(rejects_missing(&PlatformHooks::flash_total_bytes));
  CHECK(rejects_missing(&PlatformHooks::last_reset_reason));
  CHECK(rejects_missing(&PlatformHooks::captured_watchdog_task));
  return true;
}

bool TestBuzzerFailuresAreObservable() {
  {
    FakePlatform platform;
    platform.SetTimeMs(17U);
    platform.buzzer_result = {ResultCode::kTimeout, 77U};
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(!runtime.InitializeSafeBoot());
    CHECK(!runtime.initialized());
    CHECK(platform.buzzer_calls == 1U);
    CHECK(platform.buzzer_hz == 0U);
    CHECK(!platform.pwm_started);

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.peripheral_errors[6] == 1U);
    CHECK(diagnostics.peripheral_timeouts[6] == 1U);
    CHECK(diagnostics.last_error_code ==
          static_cast<std::uint8_t>(ResultCode::kTimeout));
    CHECK(diagnostics.last_error_source ==
          static_cast<std::uint8_t>(
              mentor_pi_mcu::app::microros::ErrorSource::kBuzzer));
    CHECK(diagnostics.last_error_detail == 77U);
    CHECK(diagnostics.last_error_uptime_ms == 17U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(platform.buzzer_calls == 1U);
    runtime.SetSessionActive(true, 1U);

    BuzzerCommand command{};
    command.frequency_hz = 1000U;
    command.on_time_ms = 100U;
    command.repeat = 1U;
    CHECK(runtime.PublishBuzzerCommand(command).result.ok());
    platform.buzzer_result = {ResultCode::kIoError, 88U};
    platform.SetTimeMs(23U);
    runtime.RunPeripheralOnce();
    CHECK(platform.buzzer_calls == 2U);
    CHECK(platform.buzzer_hz == 0U);

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.peripheral_errors[6] == 1U);
    CHECK(diagnostics.peripheral_timeouts[6] == 0U);
    CHECK(diagnostics.last_error_code ==
          static_cast<std::uint8_t>(ResultCode::kIoError));
    CHECK(diagnostics.last_error_source ==
          static_cast<std::uint8_t>(
              mentor_pi_mcu::app::microros::ErrorSource::kBuzzer));
    CHECK(diagnostics.last_error_detail == 88U);
    CHECK(diagnostics.last_error_uptime_ms == 23U);
    HealthSnapshot health{};
    runtime.ReadHealth(&health);
    CHECK(health.nonfatal_degraded);
    CHECK(!health.output_processing_fault);
  }
  return true;
}

bool TestMicroRosAdapterDelegates() {
  FakePlatform platform;
  platform.SetTimeMs(123U);
  platform.now_us = 123456U;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  const auto task_entries = runtime.BuildTaskEntries({nullptr, nullptr});
  CHECK(
      task_entries[static_cast<std::size_t>(ControllerTask::kSafetySupervisor)]
          .main != nullptr);
  CHECK(task_entries[static_cast<std::size_t>(ControllerTask::kMotorControl)]
            .main != nullptr);
  CHECK(
      task_entries[static_cast<std::size_t>(ControllerTask::kMicroRos)].main ==
      nullptr);
  CHECK(
      task_entries[static_cast<std::size_t>(ControllerTask::kBusServo)].main !=
      nullptr);
  CHECK(task_entries[static_cast<std::size_t>(ControllerTask::kSensor)].main !=
        nullptr);
  CHECK(task_entries[static_cast<std::size_t>(ControllerTask::kPeripheral)]
            .main != nullptr);

  const auto hooks = runtime.BuildMicroRosHooks();
  CHECK(mentor_pi_mcu::app::microros::RuntimeHooksAreComplete(hooks));
  CHECK(hooks.monotonic_milliseconds(hooks.context) == 123U);
  CHECK(hooks.monotonic_microseconds(hooks.context) == 123456U);
  auto missing_microsecond_clock = hooks;
  missing_microsecond_clock.monotonic_microseconds = nullptr;
  CHECK(!mentor_pi_mcu::app::microros::RuntimeHooksAreComplete(
      missing_microsecond_clock));
  hooks.wait_milliseconds(hooks.context, 7U);
  hooks.advance_task_heartbeat(hooks.context);
  hooks.record_successful_ros_heartbeat(hooks.context);
  hooks.emergency_stop_motors(hooks.context);
  CHECK(platform.emergency_stops >= 3U);

  hooks.set_session_active(hooks.context, true, 4U);
  CHECK(hooks.motor_max_rps(hooks.context) == 6.0F);

  MotorCommand motor{};
  motor.update_mask = 1U;
  CHECK(hooks
            .publish_motor_command(hooks.context, motor,
                                   hooks.monotonic_microseconds(hooks.context))
            .result.ok());
  PwmServoCommand pwm{};
  pwm.update_mask = 1U;
  pwm.duration_ms = 20U;
  pwm.pulse_width_us[0] = 1500U;
  CHECK(hooks.publish_pwm_servo_command(hooks.context, pwm).result.ok());
  BusServoCommand bus{};
  bus.count = 1U;
  bus.servo_id[0] = 1U;
  bus.position[0] = 500U;
  bus.duration_ms = 20U;
  CHECK(hooks.publish_bus_servo_command(hooks.context, bus).result.ok());
  LedCommand led{};
  led.led_id = mentor_pi::mcu::kFirstHostLedId;
  led.on_time_ms = 1U;
  led.repeat = 1U;
  CHECK(hooks.publish_led_command(hooks.context, led).result.ok());
  led.led_id = 1U;
  CHECK(hooks.publish_led_command(hooks.context, led).result.code ==
        ResultCode::kOutOfRange);
  BuzzerCommand buzzer{};
  buzzer.frequency_hz = 1000U;
  buzzer.on_time_ms = 1U;
  buzzer.repeat = 1U;
  CHECK(hooks.publish_buzzer_command(hooks.context, buzzer).result.ok());
  RgbCommand rgb{};
  rgb.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  CHECK(hooks.publish_rgb_command(hooks.context, rgb).result.ok());
  rgb.update_mask = 1U;
  CHECK(hooks.publish_rgb_command(hooks.context, rgb).result.code ==
        ResultCode::kInvalidArgument);
  OledCommand oled{};
  oled.update_mask = 1U;
  CHECK(hooks.publish_oled_command(hooks.context, oled).result.ok());

  MotorTelemetry motor_telemetry{};
  PwmServoTelemetry pwm_telemetry{};
  ImuTelemetry imu_telemetry{};
  BatteryTelemetry battery_telemetry{};
  mentor_pi::mcu::ButtonEvent button_event{};
  CHECK(!hooks.read_motor_telemetry(hooks.context, &motor_telemetry));
  CHECK(!hooks.read_pwm_servo_telemetry(hooks.context, &pwm_telemetry));
  CHECK(!hooks.read_imu_telemetry(hooks.context, &imu_telemetry));
  CHECK(!hooks.read_battery_telemetry(hooks.context, &battery_telemetry));
  CHECK(!hooks.pop_button_event(hooks.context, &button_event));
  HealthSnapshot health{};
  WorkerDiagnostics diagnostics{};
  hooks.read_health(hooks.context, &health);
  hooks.read_worker_diagnostics(hooks.context, &diagnostics);
  CHECK(diagnostics.flash_total_bytes == 512U * 1024U);

  const ServiceToken token{4U, 1U};
  CHECK(hooks.dispatch_motor_model(hooks.context, token, MotorModel::kJga27));
  MotorModelReply motor_reply{};
  CHECK(!hooks.poll_motor_model(hooks.context, token, &motor_reply));
  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 1U;
  CHECK(hooks.dispatch_pwm_offsets(hooks.context, token, offsets));
  PwmOffsetsReply offsets_reply{};
  CHECK(!hooks.poll_pwm_offsets(hooks.context, token, &offsets_reply));
  CHECK(hooks.dispatch_battery_threshold(hooks.context, token, 6300U));
  BatteryThresholdReply battery_reply{};
  CHECK(!hooks.poll_battery_threshold(hooks.context, token, &battery_reply));

  GetBusServoStateCommand get{};
  get.servo_id = 1U;
  get.fields = GetBusServoStateCommand::kFieldId;
  CHECK(hooks.dispatch_bus_get_state(hooks.context, token, get));
  GetBusServoStateReply get_reply{};
  CHECK(!hooks.poll_bus_get_state(hooks.context, token, &get_reply));
  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.update_mask = ConfigureBusServoCommand::kSetTorque;
  ConfigureBusServoReply configure_reply{};
  CHECK(!hooks.dispatch_bus_configure(hooks.context, token, configure));
  CHECK(!hooks.poll_bus_configure(hooks.context, token, &configure_reply));
  StopBusServosCommand stop{};
  stop.count = 1U;
  stop.servo_id[0] = 1U;
  StopBusServosReply stop_reply{};
  CHECK(!hooks.dispatch_bus_stop(hooks.context, token, stop));
  CHECK(!hooks.poll_bus_stop(hooks.context, token, &stop_reply));

  hooks.invalidate_session_work(hooks.context, 4U);
  hooks.set_session_active(hooks.context, false, 4U);
  return true;
}

bool TestMicroRosMicrosecondAcceptanceBoundary() {
  FakePlatform platform;
  platform.SetTimeMs(123U);
  platform.now_us = 123456U;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));

  const auto hooks = runtime.BuildMicroRosHooks();
  CHECK(mentor_pi_mcu::app::microros::RuntimeHooksAreComplete(hooks));
  hooks.set_session_active(hooks.context, true, 1U);

  MotorCommand zero{};
  zero.update_mask = 1U;
  const std::uint32_t accepted_at_us =
      hooks.monotonic_microseconds(hooks.context);
  CHECK(accepted_at_us == 123456U);
  CHECK(hooks.publish_motor_command(hooks.context, zero, accepted_at_us)
            .result.ok());

  platform.now_us = 123457U;
  runtime.RunMotorControlOnce();
  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.motor_command_consumptions == 1U);
  CHECK(diagnostics.motor_command_age_over_20_ms == 0U);
  CHECK(diagnostics.motor_command_max_age_us == 1U);
  return true;
}

bool CheckPwmPhysicalDuration(std::uint16_t duration_ms) {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  runtime.SetSessionActive(true, 1U);

  PwmServoCommand command{};
  command.update_mask = 1U;
  command.duration_ms = duration_ms;
  command.pulse_width_us[0] = 1601U;
  platform.SetTimeMs(1U);
  CHECK(runtime.PublishPwmServoCommand(command).result.ok());
  runtime.RunPeripheralOnce();

  // The first boundary after acceptance is B0. It installs the current pulse
  // and starts the trajectory; it must not install interpolation step one.
  CHECK(platform.pwm_active[0] == 1500U);
  CHECK(platform.pwm_shadow[0] == 1500U);
  platform.CommitPwmFrame(20U);
  CHECK(platform.pwm_active[0] == 1500U);
  runtime.RunPeripheralOnce();
  mentor_pi_mcu::app::microros::PwmServoTelemetry telemetry{};
  CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
  CHECK(telemetry.target_pulse_width_us[0] == 1601U);
  CHECK(telemetry.output_pulse_width_us[0] == 1500U);
  CHECK((telemetry.moving_mask & 1U) != 0U);

  const std::uint16_t steps = static_cast<std::uint16_t>(
      (duration_ms + mentor_pi::mcu::kPwmFramePeriodMs - 1U) /
      mentor_pi::mcu::kPwmFramePeriodMs);
  for (std::uint16_t step = 1U; step <= steps; ++step) {
    const std::uint16_t expected = static_cast<std::uint16_t>(
        1500U + (((101U * step) + (steps / 2U)) / steps));
    CHECK(platform.pwm_shadow[0] == expected);
    platform.CommitPwmFrame(20U + (static_cast<std::uint32_t>(step) * 20U));
    CHECK(platform.pwm_active[0] == expected);
    runtime.RunPeripheralOnce();
    CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
    CHECK(telemetry.output_pulse_width_us[0] == expected);
    CHECK(((telemetry.moving_mask & 1U) != 0U) == (step < steps));
  }
  CHECK(platform.pwm_active[0] == 1601U);
  return true;
}

bool TestPwmPhysicalFrameTiming() {
  return CheckPwmPhysicalDuration(20U) && CheckPwmPhysicalDuration(21U) &&
         CheckPwmPhysicalDuration(39U) && CheckPwmPhysicalDuration(41U);
}

bool TestPwmChannelThreeRepeatedCommandsAndSessionTransitions() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  PwmServoCommand command{};
  command.update_mask = 0x04U;
  command.duration_ms = 500U;
  command.pulse_width_us[2] = 1600U;
  CHECK(runtime.PublishPwmServoCommand(command).result.ok());
  const auto repeated = runtime.PublishPwmServoCommand(command);
  CHECK(repeated.result.ok());
  CHECK(repeated.overwrote_unread);
  platform.SetTimeMs(1U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_shadow ==
        (std::array<std::uint16_t, 4>{1500U, 1500U, 1500U, 1500U}));

  // The first boundary is B0. The following 25 frames cover exactly 500 ms.
  platform.CommitPwmFrame(20U);
  runtime.RunPeripheralOnce();
  PwmServoTelemetry telemetry{};
  CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
  CHECK(telemetry.target_pulse_width_us[2] == 1600U);
  CHECK(telemetry.output_pulse_width_us[2] == 1500U);
  CHECK(telemetry.moving_mask == 0x04U);
  for (std::uint16_t step = 1U; step <= 25U; ++step) {
    const auto expected = static_cast<std::uint16_t>(1500U + (4U * step));
    CHECK(platform.pwm_shadow[0] == 1500U);
    CHECK(platform.pwm_shadow[1] == 1500U);
    CHECK(platform.pwm_shadow[2] == expected);
    CHECK(platform.pwm_shadow[3] == 1500U);
    platform.CommitPwmFrame(20U + (static_cast<std::uint32_t>(step) * 20U));
    runtime.RunPeripheralOnce();
    CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
    CHECK(telemetry.target_pulse_width_us[2] == 1600U);
    CHECK(telemetry.output_pulse_width_us[2] == expected);
    CHECK(telemetry.moving_mask == (step < 25U ? 0x04U : 0U));
  }

  // Leave unread generation-1 work, then replace the owner. The stale command
  // must be discarded without changing the physically committed PWM3 output.
  command.pulse_width_us[2] = 1400U;
  CHECK(runtime.PublishPwmServoCommand(command).result.ok());
  runtime.SetSessionActive(false, 1U);
  runtime.InvalidateSessionWork(1U);
  runtime.SetSessionActive(true, 2U);
  platform.SetTimeMs(521U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_shadow[2] == 1600U);
  platform.CommitPwmFrame(540U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_active[2] == 1600U);

  // Repeated generation-2 commands must merge to PWM channel index 2 and
  // replan from the committed 1600 us pulse, again over exactly 500 ms.
  const auto current = runtime.PublishPwmServoCommand(command);
  CHECK(current.result.ok());
  const auto current_repeated = runtime.PublishPwmServoCommand(command);
  CHECK(current_repeated.result.ok());
  CHECK(current_repeated.overwrote_unread);
  platform.SetTimeMs(541U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_shadow[2] == 1600U);
  platform.CommitPwmFrame(560U);
  runtime.RunPeripheralOnce();
  CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
  CHECK(telemetry.target_pulse_width_us[2] == 1400U);
  CHECK(telemetry.output_pulse_width_us[2] == 1600U);
  CHECK(telemetry.moving_mask == 0x04U);
  for (std::uint16_t step = 1U; step <= 25U; ++step) {
    const auto expected = static_cast<std::uint16_t>(1600U - (8U * step));
    CHECK(platform.pwm_shadow[0] == 1500U);
    CHECK(platform.pwm_shadow[1] == 1500U);
    CHECK(platform.pwm_shadow[2] == expected);
    CHECK(platform.pwm_shadow[3] == 1500U);
    platform.CommitPwmFrame(560U + (static_cast<std::uint32_t>(step) * 20U));
    runtime.RunPeripheralOnce();
    CHECK(runtime.ReadPwmServoTelemetry(&telemetry));
    CHECK(telemetry.target_pulse_width_us[2] == 1400U);
    CHECK(telemetry.output_pulse_width_us[2] == expected);
    CHECK(telemetry.moving_mask == (step < 25U ? 0x04U : 0U));
  }

  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[4] == 0U);
  return true;
}

bool TestImuCharacterizationSnapshot() {
  mentor_pi::mcu::drivers::ImuSample sample{};
  sample.acceleration_mps2 = {1.0F, -2.0F, 3.0F};
  sample.angular_velocity_rps = {-4.0F, 5.0F, -6.0F};
  const std::uint32_t previous_sequence =
      rrclite_imu_characterization_snapshot.sequence;
  UpdateImuCharacterizationSnapshot(1234U, 0x6aU, 0x42U, OkResult(), &sample);
  CHECK((rrclite_imu_characterization_snapshot.sequence & 1U) == 0U);
  CHECK(rrclite_imu_characterization_snapshot.sequence != previous_sequence);
  CHECK(rrclite_imu_characterization_snapshot.timestamp_ms == 1234U);
  CHECK(rrclite_imu_characterization_snapshot.address == 0x6aU);
  CHECK(rrclite_imu_characterization_snapshot.revision == 0x42U);
  CHECK(rrclite_imu_characterization_snapshot.result_code == 0U);
  CHECK(rrclite_imu_characterization_snapshot.detail == 0U);
  CHECK(rrclite_imu_characterization_snapshot.valid == 1U);
  CHECK(rrclite_imu_characterization_snapshot.acceleration_mps2[1] == -2.0F);
  CHECK(rrclite_imu_characterization_snapshot.angular_velocity_rps[2] == -6.0F);

  UpdateImuCharacterizationSnapshot(1235U, 0x6aU, 0x42U,
                                    {ResultCode::kTimeout, 46U}, nullptr);
  CHECK((rrclite_imu_characterization_snapshot.sequence & 1U) == 0U);
  CHECK(rrclite_imu_characterization_snapshot.timestamp_ms == 1235U);
  CHECK(rrclite_imu_characterization_snapshot.result_code ==
        static_cast<std::uint8_t>(ResultCode::kTimeout));
  CHECK(rrclite_imu_characterization_snapshot.detail == 46U);
  CHECK(rrclite_imu_characterization_snapshot.valid == 0U);
  CHECK(rrclite_imu_characterization_snapshot.acceleration_mps2[1] == 0.0F);
  CHECK(rrclite_imu_characterization_snapshot.angular_velocity_rps[2] == 0.0F);
  return true;
}

bool TestImuCharacterizationRequiresSuccessfulRawRead() {
  FakePlatform platform;
  platform.imu_status_read_status = IoStatus::kTimeout;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  runtime.RunSensorOnce();
  ImuTelemetry telemetry{};
  CHECK(!runtime.ReadImuTelemetry(&telemetry));
  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 1U);
  CHECK(diagnostics.peripheral_timeouts[1] == 1U);
  CHECK(diagnostics.last_error_code ==
        static_cast<std::uint8_t>(ResultCode::kTimeout));
  CHECK(diagnostics.last_error_source ==
        static_cast<std::uint8_t>(
            mentor_pi_mcu::app::microros::ErrorSource::kImu));

  platform.imu_status_read_status = IoStatus::kOk;
  platform.SetTimeMs(20U);
  runtime.RunSensorOnce();
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(!telemetry.valid);
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 2U);
  CHECK(diagnostics.peripheral_timeouts[1] == 1U);
  CHECK(diagnostics.last_error_code ==
        static_cast<std::uint8_t>(ResultCode::kUnsupported));
  CHECK(diagnostics.last_error_source ==
        static_cast<std::uint8_t>(
            mentor_pi_mcu::app::microros::ErrorSource::kImu));
  CHECK(diagnostics.last_error_detail == 1U);
  return true;
}

bool TestImuPersistentBusyIsDiagnosedAndReset() {
  FakePlatform platform;
  platform.imu_data_ready = false;
  AxisTransform transform{};
  transform.output = {{{0U, 1}, {1U, 1}, {2U, 1}}};
  transform.verified = true;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), transform));
  CHECK(runtime.InitializeSafeBoot());

  runtime.RunSensorOnce();
  ImuTelemetry telemetry{};
  CHECK(!runtime.ReadImuTelemetry(&telemetry));

  platform.SetTimeMs(499U);
  runtime.RunSensorOnce();
  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 0U);
  CHECK(platform.imu_reset_writes == 0U);

  platform.SetTimeMs(500U);
  runtime.RunSensorOnce();
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(!telemetry.valid);
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 1U);
  CHECK(diagnostics.peripheral_timeouts[1] == 1U);
  CHECK(diagnostics.last_error_code ==
        static_cast<std::uint8_t>(ResultCode::kTimeout));
  CHECK(diagnostics.last_error_source ==
        static_cast<std::uint8_t>(
            mentor_pi_mcu::app::microros::ErrorSource::kImu));
  CHECK(diagnostics.last_error_detail == 46U);
  CHECK(platform.imu_reset_writes == 1U);

  platform.imu_data_ready = true;
  platform.SetTimeMs(514U);
  runtime.RunSensorOnce();
  CHECK(!runtime.ReadImuTelemetry(&telemetry));
  platform.SetTimeMs(515U);
  runtime.RunSensorOnce();
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(telemetry.valid);
  CHECK(telemetry.timestamp_ms == 515U);
  HealthSnapshot health{};
  runtime.ReadHealth(&health);
  CHECK(health.imu_healthy);
  CHECK(platform.imu_reset_writes == 1U);
  return true;
}

bool TestImuPersistentInitializeBusyIsDiagnosedAndRecovers() {
  FakePlatform platform;
  platform.imu_identity_read_status = IoStatus::kBusy;
  AxisTransform transform{};
  transform.output = {{{0U, 1}, {1U, 1}, {2U, 1}}};
  transform.verified = true;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), transform));
  CHECK(runtime.InitializeSafeBoot());

  runtime.RunSensorOnce();
  ImuTelemetry telemetry{};
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(!telemetry.valid);
  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 0U);
  CHECK(diagnostics.peripheral_timeouts[1] == 0U);

  platform.SetTimeMs(999U);
  runtime.RunSensorOnce();
  CHECK(!runtime.ReadImuTelemetry(&telemetry));
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 0U);

  platform.SetTimeMs(1000U);
  runtime.RunSensorOnce();
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(!telemetry.valid);
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.peripheral_errors[1] == 1U);
  CHECK(diagnostics.peripheral_timeouts[1] == 1U);
  CHECK(diagnostics.last_error_code ==
        static_cast<std::uint8_t>(ResultCode::kTimeout));
  CHECK(diagnostics.last_error_source ==
        static_cast<std::uint8_t>(
            mentor_pi_mcu::app::microros::ErrorSource::kImu));
  CHECK(diagnostics.last_error_detail == 0x6aU);

  platform.imu_identity_read_status = IoStatus::kOk;
  platform.SetTimeMs(2000U);
  runtime.RunSensorOnce();
  CHECK(runtime.ReadImuTelemetry(&telemetry));
  CHECK(telemetry.valid);
  CHECK(telemetry.timestamp_ms == 2000U);
  HealthSnapshot health{};
  runtime.ReadHealth(&health);
  CHECK(health.imu_healthy);
  return true;
}

bool TestSensorSchedulesRemainPhaseStable() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 2U);
  CHECK(platform.battery_samples == 1U);

  platform.SetTimeMs(29U);
  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 2U);
  CHECK(platform.battery_samples == 1U);

  // A late dispatch samples once but must retain the original absolute phase.
  platform.SetTimeMs(31U);
  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 4U);
  CHECK(platform.battery_samples == 1U);

  platform.SetTimeMs(59U);
  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 4U);
  CHECK(platform.battery_samples == 2U);

  // The button deadline remains 60 ms, not 31 + 30 ms.
  platform.SetTimeMs(60U);
  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 6U);
  CHECK(platform.battery_samples == 2U);

  // The late battery sample retained the 50 ms phase and is due at 100 ms.
  platform.SetTimeMs(100U);
  runtime.RunSensorOnce();
  CHECK(platform.button_reads == 8U);
  CHECK(platform.battery_samples == 3U);

  FakePlatform wrapping_platform;
  wrapping_platform.SetTimeMs(0xfffffff0U);
  ControllerRuntime wrapping_runtime(FullRangeTestMotorConfiguration());
  CHECK(wrapping_runtime.Configure(wrapping_platform.Hooks(), AxisTransform{}));
  CHECK(wrapping_runtime.InitializeSafeBoot());
  wrapping_runtime.RunSensorOnce();
  CHECK(wrapping_platform.button_reads == 2U);
  wrapping_platform.SetTimeMs(13U);
  wrapping_runtime.RunSensorOnce();
  CHECK(wrapping_platform.button_reads == 2U);
  wrapping_platform.SetTimeMs(14U);
  wrapping_runtime.RunSensorOnce();
  CHECK(wrapping_platform.button_reads == 4U);
  return true;
}

bool TestControllerIntegration() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  AxisTransform unverified_transform{};
  CHECK(runtime.Configure(platform.Hooks(), unverified_transform));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(platform.pwm_started);
  CHECK(platform.pwm_shadow ==
        (std::array<std::uint16_t, 4>{1500U, 1500U, 1500U, 1500U}));
  CHECK(platform.leds == (std::array<bool, 3>{false, false, false}));
  CHECK(platform.buzzer_hz == 0U);
  CHECK(EstablishStartupReadiness(&runtime, &platform));

  auto ros_hooks = runtime.BuildMicroRosHooks();
  CHECK(mentor_pi_mcu::app::microros::RuntimeHooksAreComplete(ros_hooks));
  runtime.SetSessionActive(true, 1U);

  MotorCommand command{};
  command.update_mask = 1U;
  command.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(command, 0U).result.ok());
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_armed[0]);
  for (std::uint32_t millisecond = 1U; millisecond <= 10U; ++millisecond) {
    platform.SetTimeMs(millisecond);
    runtime.RunMotorControlOnce();
  }
  CHECK(platform.motor_duty[0] < 0);
  CHECK(platform.arm_observed_inside_critical);
  CHECK(platform.apply_observed_inside_critical);
  for (std::uint32_t millisecond = 11U; millisecond <= 198U; ++millisecond) {
    platform.SetTimeMs(millisecond);
    runtime.RunMotorControlOnce();
  }
  CHECK(!platform.motor_armed[0]);
  CHECK(platform.motor_duty[0] == 0);

  CHECK(runtime.PublishMotorCommand(command, 198000U).result.ok());
  runtime.InvalidateSessionWork(1U);
  runtime.SetSessionActive(true, 2U);
  platform.SetTimeMs(199U);
  runtime.RunMotorControlOnce();
  CHECK(!platform.motor_armed[0]);

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 1U;
  offsets.offset_us[0] = 25;
  const ServiceToken offset_token{2U, 1U};
  CHECK(runtime.DispatchPwmOffsets(offset_token, offsets));
  runtime.RunPeripheralOnce();
  PwmOffsetsReply offset_reply{};
  CHECK(!runtime.PollPwmOffsets(offset_token, &offset_reply));
  CHECK(platform.pwm_shadow[0] == 1525U);
  CHECK(platform.pwm_active[0] == 1500U);
  platform.CommitPwmFrame(220U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_active[0] == 1525U);
  CHECK(platform.pwm_shadow[0] == 1525U);
  CHECK(runtime.PollPwmOffsets(offset_token, &offset_reply));
  CHECK(offset_reply.result.ok());
  CHECK(offset_reply.applied_mask == 1U);

  BusServoCommand bus{};
  bus.count = 1U;
  bus.servo_id[0] = 1U;
  bus.position[0] = 500U;
  bus.duration_ms = 20U;
  CHECK(runtime.PublishBusServoCommand(bus).result.ok());
  runtime.RunBusServoOnce();
  CHECK(platform.bus_exchanges == 1U);
  platform.SetTimeMs(250U);
  runtime.RunBusServoOnce();

  platform.buttons[0] = true;
  platform.SetTimeMs(500U);
  runtime.RunSensorOnce();
  CHECK((rrclite_imu_characterization_snapshot.sequence & 1U) == 0U);
  CHECK(rrclite_imu_characterization_snapshot.timestamp_ms == 500U);
  CHECK(rrclite_imu_characterization_snapshot.address == 0x6aU);
  CHECK(rrclite_imu_characterization_snapshot.revision == 1U);
  CHECK(rrclite_imu_characterization_snapshot.result_code == 0U);
  CHECK(rrclite_imu_characterization_snapshot.valid == 1U);
  mentor_pi_mcu::app::microros::ImuTelemetry imu_telemetry{};
  CHECK(runtime.ReadImuTelemetry(&imu_telemetry));
  CHECK(!imu_telemetry.valid);
  platform.SetTimeMs(530U);
  runtime.RunSensorOnce();
  mentor_pi::mcu::ButtonEvent event{};
  CHECK(runtime.PopButtonEvent(&event));
  CHECK(event.button_id == 1U);
  CHECK(event.event == mentor_pi::mcu::ButtonEventType::kPressed);

  platform.SetTimeMs(540U);
  runtime.AdvanceMicroRosHeartbeat();
  runtime.RunMotorControlOnce();
  runtime.RunBusServoOnce();
  runtime.RunSensorOnce();
  runtime.RunPeripheralOnce();
  runtime.RunSafetySupervisorOnce();
  const std::uint32_t refreshes = platform.watchdog_refreshes;
  CHECK(refreshes > 0U);

  platform.SetTimeMs(661U);
  runtime.RunMotorControlOnce();
  runtime.RunBusServoOnce();
  runtime.RunSensorOnce();
  runtime.RunPeripheralOnce();
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.watchdog_refreshes == refreshes);

  mentor_pi_mcu::app::microros::HealthSnapshot health{};
  runtime.ReadHealth(&health);
  CHECK(health.output_processing_fault);
  CHECK(!health.imu_healthy);
  mentor_pi_mcu::app::microros::WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.motor_lease_expiries[0] == 1U);
  CHECK(diagnostics.last_watchdog_task == 255U);
  CHECK(platform.persist_watchdog_task_calls == 1U);
  CHECK(platform.persisted_watchdog_task ==
        static_cast<std::uint8_t>(ControllerTask::kMicroRos));
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.persist_watchdog_task_calls == 1U);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestBusServoServicesAndStopPreemption() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  runtime.RunBusServoOnce();
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  GetBusServoStateCommand get{};
  get.servo_id = 7U;
  get.fields = GetBusServoStateCommand::kAllFields;
  const ServiceToken get_token{1U, 1U};
  CHECK(runtime.DispatchBusGetState(get_token, get));
  CHECK(!runtime.DispatchBusGetState({1U, 2U}, get));
  GetBusServoStateReply get_reply{};
  bool get_complete = false;
  for (std::uint32_t iteration = 0U; iteration < 24U; ++iteration) {
    platform.SetTimeMs(iteration + 1U);
    runtime.RunBusServoOnce();
    if (runtime.PollBusGetState(get_token, &get_reply)) {
      get_complete = true;
      break;
    }
  }
  CHECK(get_complete && get_reply.result.ok());
  CHECK(get_reply.state.valid_fields == GetBusServoStateCommand::kAllFields);
  CHECK(get_reply.state.requested_id == 7U);
  CHECK(get_reply.state.reported_id == 7U);
  CHECK(get_reply.state.position == -52);
  CHECK(get_reply.state.offset == -10);
  CHECK(get_reply.state.voltage_mv == 7200U);
  CHECK(get_reply.state.temperature_c == 42U);
  CHECK(get_reply.state.position_min == 100U);
  CHECK(get_reply.state.position_max == 900U);
  CHECK(get_reply.state.voltage_min_mv == 6000U);
  CHECK(get_reply.state.voltage_max_mv == 9000U);
  CHECK(get_reply.state.temperature_limit_c == 75U);
  CHECK(get_reply.state.torque_enabled);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 7U;
  configure.update_mask = ConfigureBusServoCommand::kAllUpdates;
  configure.new_id = 8U;
  configure.offset = -10;
  configure.position_min = 100U;
  configure.position_max = 900U;
  configure.voltage_min_mv = 6000U;
  configure.voltage_max_mv = 9000U;
  configure.temperature_limit_c = 75U;
  configure.torque_enabled = true;
  const ServiceToken configure_token{1U, 2U};
  CHECK(runtime.DispatchBusConfigure(configure_token, configure));
  ConfigureBusServoReply configure_reply{};
  bool configure_complete = false;
  for (std::uint32_t iteration = 0U; iteration < 20U; ++iteration) {
    platform.SetTimeMs(30U + iteration);
    runtime.RunBusServoOnce();
    if (runtime.PollBusConfigure(configure_token, &configure_reply)) {
      configure_complete = true;
      break;
    }
  }
  CHECK(configure_complete && configure_reply.result.ok());
  CHECK(configure_reply.applied_mask == ConfigureBusServoCommand::kAllUpdates);
  CHECK(configure_reply.effective_id == 8U);

  StopBusServosCommand stop{};
  stop.count = 2U;
  stop.servo_id[0] = 1U;
  stop.servo_id[1] = 2U;
  const ServiceToken stop_token{1U, 3U};
  CHECK(runtime.DispatchBusStop(stop_token, stop));
  StopBusServosReply stop_reply{};
  bool stop_complete = false;
  for (std::uint32_t iteration = 0U; iteration < 8U; ++iteration) {
    platform.SetTimeMs(60U + iteration);
    runtime.RunBusServoOnce();
    if (runtime.PollBusStop(stop_token, &stop_reply)) {
      stop_complete = true;
      break;
    }
  }
  CHECK(stop_complete && stop_reply.result.ok());
  CHECK(stop_reply.commands_transmitted == 2U);

  platform.bus_begin_status = IoStatus::kIoError;
  const ServiceToken failed_token{1U, 4U};
  CHECK(runtime.DispatchBusStop(failed_token, stop));
  runtime.RunBusServoOnce();
  StopBusServosReply failed_reply{};
  CHECK(runtime.PollBusStop(failed_token, &failed_reply));
  CHECK(failed_reply.result.code == ResultCode::kIoError);
  platform.bus_begin_status = IoStatus::kOk;

  // An in-flight service is canceled when its token loses session ownership.
  platform.bus_hold_active = true;
  const ServiceToken stale_token{1U, 5U};
  CHECK(runtime.DispatchBusGetState(stale_token, get));
  runtime.RunBusServoOnce();
  CHECK(platform.bus_active);
  runtime.SetSessionActive(false, 1U);
  runtime.RunBusServoOnce();
  CHECK(!platform.bus_active);
  CHECK(!runtime.PollBusGetState(stale_token, &get_reply));
  platform.bus_hold_active = false;
  runtime.SetSessionActive(true, 2U);

  // A stop waits for the current UART frame boundary, then cancels the unsent
  // part of a move batch and starts its own bounded batch.
  BusServoCommand move{};
  move.count = 2U;
  move.servo_id[0] = 1U;
  move.servo_id[1] = 2U;
  move.position[0] = 400U;
  move.position[1] = 600U;
  move.duration_ms = 100U;
  platform.bus_hold_active = true;
  CHECK(runtime.PublishBusServoCommand(move).result.ok());
  runtime.RunBusServoOnce();
  CHECK(platform.bus_active);
  const ServiceToken preempt_token{2U, 1U};
  CHECK(runtime.DispatchBusStop(preempt_token, stop));
  runtime.RunBusServoOnce();
  platform.bus_hold_active = false;
  runtime.RunBusServoOnce();
  bool preempt_complete = false;
  for (std::uint32_t iteration = 0U; iteration < 8U; ++iteration) {
    runtime.RunBusServoOnce();
    if (runtime.PollBusStop(preempt_token, &stop_reply)) {
      preempt_complete = true;
      break;
    }
  }
  CHECK(preempt_complete && stop_reply.result.ok());
  CHECK(stop_reply.commands_transmitted == 2U);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestSafetyStartupGrace() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  runtime.RunSafetySupervisorOnce();
  CHECK(platform.watchdog_refreshes == 1U);
  mentor_pi_mcu::app::microros::HealthSnapshot health{};
  runtime.ReadHealth(&health);
  CHECK(!health.output_processing_fault);

  platform.SetTimeMs(249U);
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.watchdog_refreshes == 2U);
  platform.SetTimeMs(250U);
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.watchdog_refreshes == 2U);
  runtime.ReadHealth(&health);
  CHECK(health.output_processing_fault);
  return true;
}

bool TestRetainedWatchdogTaskSemantics() {
  FakePlatform platform;
  platform.reset_reason = 3U;
  platform.captured_watchdog_task =
      static_cast<std::uint8_t>(ControllerTask::kMicroRos);
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  WorkerDiagnostics diagnostics{};
  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.last_reset_reason == 3U);
  CHECK(diagnostics.last_watchdog_task ==
        static_cast<std::uint8_t>(ControllerTask::kMicroRos));

  runtime.RunSafetySupervisorOnce();
  platform.SetTimeMs(250U);
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.persist_watchdog_task_calls == 1U);
  CHECK(platform.persisted_watchdog_task ==
        static_cast<std::uint8_t>(ControllerTask::kMotorControl));
  runtime.RunSafetySupervisorOnce();
  CHECK(platform.persist_watchdog_task_calls == 1U);

  runtime.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.last_watchdog_task ==
        static_cast<std::uint8_t>(ControllerTask::kMicroRos));

  FakePlatform malformed_platform;
  malformed_platform.captured_watchdog_task = 6U;
  ControllerRuntime malformed(FullRangeTestMotorConfiguration());
  CHECK(malformed.Configure(malformed_platform.Hooks(), AxisTransform{}));
  malformed.ReadWorkerDiagnostics(&diagnostics);
  CHECK(diagnostics.last_watchdog_task == 255U);
  return true;
}

bool TestSessionTeardownInvalidatesOwnedWork() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  runtime.SetSessionActive(true, 1U);

  PwmServoCommand pwm{};
  pwm.update_mask = 1U;
  pwm.duration_ms = 100U;
  pwm.pulse_width_us[0] = 2000U;
  CHECK(runtime.PublishPwmServoCommand(pwm).result.ok());
  runtime.RunPeripheralOnce();
  platform.CommitPwmFrame(20U);
  runtime.RunPeripheralOnce();
  platform.CommitPwmFrame(40U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_shadow[0] > 1500U);
  CHECK(platform.pwm_active[0] == 1600U);

  // The calculated next interpolation step is still only a shadow. Session
  // teardown replaces it with the physically committed pulse before the next
  // common boundary and cancels the remaining trajectory.
  runtime.InvalidateSessionWork(1U);
  platform.SetTimeMs(41U);
  runtime.RunPeripheralOnce();
  CHECK(platform.pwm_shadow[0] == 1600U);
  runtime.SetSessionActive(true, 2U);
  for (std::uint32_t frame = 0U; frame < 8U; ++frame) {
    platform.CommitPwmFrame(60U + (frame * 20U));
    runtime.RunPeripheralOnce();
    CHECK(platform.pwm_active[0] == 1600U);
    CHECK(platform.pwm_shadow[0] == 1600U);
  }

  platform.bus_hold_active = true;
  BusServoCommand bus{};
  bus.count = 2U;
  bus.servo_id[0] = 1U;
  bus.servo_id[1] = 2U;
  bus.position[0] = 400U;
  bus.position[1] = 600U;
  bus.duration_ms = 100U;
  CHECK(runtime.PublishBusServoCommand(bus).result.ok());
  runtime.RunBusServoOnce();
  CHECK(platform.bus_active);
  CHECK(platform.bus_exchanges == 1U);
  runtime.InvalidateSessionWork(2U);
  runtime.RunBusServoOnce();
  CHECK(!platform.bus_active);
  platform.bus_hold_active = false;
  runtime.SetSessionActive(true, 3U);
  runtime.RunBusServoOnce();
  CHECK(platform.bus_exchanges == 1U);

  // A command waiting behind another in-flight RGB frame is tagged to its ROS
  // session and cannot start after that session ends.
  platform.rgb_hold_active = true;
  RgbCommand rgb{};
  rgb.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  rgb.red[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(runtime.PublishRgbCommand(rgb).result.ok());
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_active);
  const std::uint32_t initial_rgb_transfers = platform.rgb_transfers;
  rgb.red[mentor_pi::mcu::kHostRgbPixelIndex] = 0U;
  rgb.green[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(runtime.PublishRgbCommand(rgb).result.ok());
  runtime.RunPeripheralOnce();
  runtime.InvalidateSessionWork(3U);
  platform.rgb_hold_active = false;
  runtime.RunPeripheralOnce();
  CHECK(platform.rgb_transfers == initial_rgb_transfers);

  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestMotorSessionRevocationOrdering() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  MotorCommand motion{};
  motion.update_mask = 1U;
  motion.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_armed[0]);

  // The emergency-stop hook probes command admission at the exact stop call.
  // It must already observe INACTIVE, and the stop must be inside the same
  // controller critical section as the authority transition.
  platform.stop_probe_runtime = &runtime;
  platform.stop_probe_enabled = true;
  runtime.SetSessionActive(false, 1U);
  CHECK(platform.stop_observed_inactive);
  CHECK(platform.stop_observed_inside_critical);
  CHECK(!platform.motor_armed[0]);
  platform.stop_probe_enabled = false;
  runtime.RunMotorControlOnce();
  CHECK(!platform.motor_armed[0]);
  runtime.SetSessionActive(true, 1U);
  CHECK(runtime.PublishMotorCommand(motion, 1000U).result.code ==
        ResultCode::kBusy);

  // A command accepted in generation 2 cannot be consumed after generation 3
  // activates, even if MotorControlTask never observed the intermediate
  // inactive state before the new session was published.
  runtime.SetSessionActive(true, 2U);
  CHECK(runtime.PublishMotorCommand(motion, 1000U).result.ok());
  runtime.SetSessionActive(false, 2U);
  runtime.SetSessionActive(true, 3U);
  platform.SetTimeMs(2U);
  runtime.RunMotorControlOnce();
  CHECK(!platform.motor_armed[0]);

  // The standalone invalidation path provides the same revoke-before-stop
  // ordering; it is safe even when invoked without a preceding set(false).
  CHECK(runtime.PublishMotorCommand(motion, 2000U).result.ok());
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_armed[0]);
  platform.stop_observed_inactive = false;
  platform.stop_observed_inside_critical = false;
  platform.stop_probe_enabled = true;
  runtime.InvalidateSessionWork(3U);
  CHECK(platform.stop_observed_inactive);
  CHECK(platform.stop_observed_inside_critical);
  CHECK(!platform.motor_armed[0]);
  platform.stop_probe_enabled = false;
  CHECK(runtime.PublishMotorCommand(motion, 2000U).result.code ==
        ResultCode::kBusy);
  CHECK(platform.critical_depth == 0U);

  // BUSY is the target transport-inhibit result. It disarms the channel but
  // remains a reconnectable session fault rather than poisoning the watchdog.
  FakePlatform inhibited_platform;
  inhibited_platform.arm_result = {ResultCode::kBusy, 1U};
  ControllerRuntime inhibited(FullRangeTestMotorConfiguration());
  CHECK(inhibited.Configure(inhibited_platform.Hooks(), AxisTransform{}));
  CHECK(inhibited.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&inhibited, &inhibited_platform));
  inhibited.SetSessionActive(true, 1U);
  CHECK(inhibited.PublishMotorCommand(motion, 0U).result.ok());
  inhibited.RunMotorControlOnce();
  CHECK(inhibited_platform.arm_observed_inside_critical);
  CHECK(!inhibited_platform.motor_armed[0]);
  mentor_pi_mcu::app::microros::HealthSnapshot health{};
  inhibited.ReadHealth(&health);
  CHECK(!health.output_processing_fault);
  inhibited.SetSessionActive(true, 1U);
  CHECK(inhibited.PublishMotorCommand(motion, 1000U).result.code ==
        ResultCode::kBusy);
  inhibited_platform.arm_result = OkResult();
  inhibited.SetSessionActive(true, 2U);
  CHECK(inhibited.PublishMotorCommand(motion, 2000U).result.ok());
  inhibited_platform.SetTimeMs(2U);
  inhibited.RunMotorControlOnce();
  CHECK(inhibited_platform.motor_armed[0]);
  CHECK(inhibited_platform.critical_depth == 0U);
  return true;
}

bool TestSessionGenerationWatermark() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 2U);

  // A different ACTIVE generation cannot replace a live owner. The current
  // token remains generation 2 after the attempted downgrade.
  runtime.SetSessionActive(true, 1U);
  CHECK(!runtime.DispatchBatteryThreshold({1U, 1U}, 6300U));
  CHECK(runtime.DispatchBatteryThreshold({2U, 1U}, 6300U));
  runtime.InvalidateSessionWork(2U);

  MotorCommand stop{};
  stop.update_mask = 1U;
  runtime.SetSessionActive(true, 2U);
  CHECK(runtime.PublishMotorCommand(stop, 0U).result.code == ResultCode::kBusy);
  runtime.SetSessionActive(true, 3U);
  CHECK(runtime.PublishMotorCommand(stop, 0U).result.ok());

  // A stale false transition always stops, but cannot lower the watermark and
  // make replay of the just-revoked generation look fresh.
  runtime.SetSessionActive(false, 2U);
  CHECK(!platform.motor_armed[0]);
  runtime.SetSessionActive(true, 3U);
  CHECK(runtime.PublishMotorCommand(stop, 0U).result.code == ResultCode::kBusy);
  runtime.SetSessionActive(true, 4U);
  CHECK(runtime.PublishMotorCommand(stop, 0U).result.ok());
  runtime.SetSessionActive(true, 3U);
  CHECK(!runtime.DispatchBatteryThreshold({3U, 2U}, 6300U));
  CHECK(runtime.DispatchBatteryThreshold({4U, 2U}, 6300U));
  runtime.InvalidateSessionWork(4U);

  // The signed-delta ordering accepts the documented nonzero wrap and rejects
  // the pre-wrap value after generation 1 becomes the watermark.
  FakePlatform wrapped_platform;
  ControllerRuntime wrapped(FullRangeTestMotorConfiguration());
  CHECK(wrapped.Configure(wrapped_platform.Hooks(), AxisTransform{}));
  CHECK(wrapped.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&wrapped, &wrapped_platform));
  constexpr std::uint32_t kMaximumGeneration =
      std::numeric_limits<std::uint32_t>::max();
  wrapped.SetSessionActive(true, kMaximumGeneration);
  CHECK(wrapped.PublishMotorCommand(stop, 0U).result.ok());
  wrapped.SetSessionActive(false, kMaximumGeneration);
  wrapped.SetSessionActive(true, 1U);
  CHECK(wrapped.PublishMotorCommand(stop, 0U).result.ok());
  wrapped.SetSessionActive(false, kMaximumGeneration);
  wrapped.SetSessionActive(true, 1U);
  CHECK(wrapped.PublishMotorCommand(stop, 0U).result.code == ResultCode::kBusy);
  wrapped.SetSessionActive(true, 2U);
  CHECK(wrapped.PublishMotorCommand(stop, 0U).result.ok());
  CHECK(platform.critical_depth == 0U);
  CHECK(wrapped_platform.critical_depth == 0U);
  return true;
}

bool TestStartupMotorInhibit() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());

  // Start the grace interval, then deliberately activate ROS and queue motion
  // before PeripheralTask has ever reported progress.
  runtime.RunSafetySupervisorOnce();
  runtime.SetSessionActive(true, 1U);
  MotorCommand motion{};
  motion.update_mask = 1U;
  motion.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
  for (std::uint32_t release = 0U; release <= 10U; ++release) {
    platform.SetTimeMs(release);
    runtime.RunMotorControlOnce();
  }
  CHECK(platform.motor_arm_calls == 0U);
  CHECK(platform.motor_apply_calls == 0U);
  CHECK(!platform.motor_armed[0]);
  CHECK(platform.motor_duty[0] == 0);

  runtime.AdvanceMicroRosHeartbeat();
  runtime.RunBusServoOnce();
  runtime.RunSensorOnce();
  runtime.RunSafetySupervisorOnce();
  platform.SetTimeMs(11U);
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_arm_calls == 0U);
  CHECK(platform.motor_apply_calls == 0U);

  // The last peer heartbeat clears the inhibit at the next safety iteration.
  // The original first-generation latest-value command is then consumed.
  runtime.RunPeripheralOnce();
  runtime.RunSafetySupervisorOnce();
  platform.SetTimeMs(12U);
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_arm_calls == 1U);
  CHECK(platform.motor_armed[0]);
  for (std::uint32_t release = 13U; release <= 22U; ++release) {
    platform.SetTimeMs(release);
    runtime.RunMotorControlOnce();
  }
  CHECK(platform.motor_apply_calls > 0U);
  CHECK(platform.motor_duty[0] < 0);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestCrossSessionMergedFieldOwnership() {
  // Gen-1 motor 1 remains retained behind the startup inhibit. A gen-2 update
  // selecting only motor 2 must reset the merged shadow rather than retagging
  // motor 1 as gen-2 work.
  FakePlatform motor_platform;
  ControllerRuntime motor_runtime(FullRangeTestMotorConfiguration());
  CHECK(motor_runtime.Configure(motor_platform.Hooks(), AxisTransform{}));
  CHECK(motor_runtime.InitializeSafeBoot());
  motor_runtime.RunSafetySupervisorOnce();
  motor_runtime.SetSessionActive(true, 1U);
  MotorCommand old_motor{};
  old_motor.update_mask = 0x01U;
  old_motor.target_rps[0] = 1.0F;
  CHECK(motor_runtime.PublishMotorCommand(old_motor, 0U).result.ok());
  motor_runtime.RunMotorControlOnce();
  CHECK(motor_platform.motor_arm_calls_by_channel[0] == 0U);

  motor_runtime.SetSessionActive(false, 1U);
  motor_runtime.SetSessionActive(true, 2U);
  MotorCommand new_motor{};
  new_motor.update_mask = 0x02U;
  new_motor.target_rps[1] = 1.0F;
  CHECK(motor_runtime.PublishMotorCommand(new_motor, 0U).result.ok());
  CHECK(EstablishStartupReadiness(&motor_runtime, &motor_platform));
  motor_runtime.RunMotorControlOnce();
  CHECK(motor_platform.motor_arm_calls_by_channel[0] == 0U);
  CHECK(motor_platform.motor_arm_calls_by_channel[1] == 1U);
  for (std::uint32_t release = 1U; release <= 10U; ++release) {
    motor_platform.SetTimeMs(release);
    motor_runtime.RunMotorControlOnce();
  }
  CHECK(motor_platform.motor_duty[0] == 0);
  CHECK(motor_platform.motor_duty[1] < 0);

  // The analogous unconsumed gen-1 PWM field must not be applied when gen 2
  // first selects another channel. Channel 1 remains at its held reset target.
  FakePlatform pwm_platform;
  ControllerRuntime pwm_runtime(FullRangeTestMotorConfiguration());
  CHECK(pwm_runtime.Configure(pwm_platform.Hooks(), AxisTransform{}));
  CHECK(pwm_runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&pwm_runtime, &pwm_platform));
  pwm_runtime.SetSessionActive(true, 1U);
  PwmServoCommand old_pwm{};
  old_pwm.update_mask = 0x01U;
  old_pwm.duration_ms = 100U;
  old_pwm.pulse_width_us[0] = 1800U;
  CHECK(pwm_runtime.PublishPwmServoCommand(old_pwm).result.ok());
  pwm_runtime.SetSessionActive(false, 1U);
  pwm_runtime.SetSessionActive(true, 2U);
  PwmServoCommand new_pwm{};
  new_pwm.update_mask = 0x02U;
  new_pwm.duration_ms = 100U;
  new_pwm.pulse_width_us[1] = 1700U;
  CHECK(pwm_runtime.PublishPwmServoCommand(new_pwm).result.ok());
  pwm_runtime.RunPeripheralOnce();
  pwm_platform.CommitPwmFrame(20U);
  pwm_runtime.RunPeripheralOnce();
  mentor_pi_mcu::app::microros::PwmServoTelemetry telemetry{};
  CHECK(pwm_runtime.ReadPwmServoTelemetry(&telemetry));
  CHECK(telemetry.target_pulse_width_us[0] == 1500U);
  CHECK(telemetry.target_pulse_width_us[1] == 1700U);

  // The same unread-shadow rule covers every other merged command type.
  // Whole-command bus and buzzer work remains generation-tagged and is simply
  // rejected if no replacement arrives in the new session.
  FakePlatform merged_platform;
  ControllerRuntime merged_runtime(FullRangeTestMotorConfiguration());
  CHECK(merged_runtime.Configure(merged_platform.Hooks(), AxisTransform{}));
  CHECK(merged_runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&merged_runtime, &merged_platform));
  merged_runtime.SetSessionActive(true, 1U);
  LedCommand old_led{mentor_pi::mcu::kFirstHostLedId, 100U, 0U, 0U};
  CHECK(merged_runtime.PublishLedCommand(old_led).result.ok());
  RgbCommand old_rgb{};
  old_rgb.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  old_rgb.red[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(merged_runtime.PublishRgbCommand(old_rgb).result.ok());
  OledCommand old_oled{};
  old_oled.update_mask = 0x01U;
  old_oled.lines[0].size = 1U;
  old_oled.lines[0].bytes[0] = 'A';
  CHECK(merged_runtime.PublishOledCommand(old_oled).result.ok());
  BusServoCommand old_bus{};
  old_bus.count = 1U;
  old_bus.servo_id[0] = 1U;
  old_bus.position[0] = 500U;
  old_bus.duration_ms = 20U;
  CHECK(merged_runtime.PublishBusServoCommand(old_bus).result.ok());
  BuzzerCommand old_buzzer{1000U, 100U, 0U, 0U};
  CHECK(merged_runtime.PublishBuzzerCommand(old_buzzer).result.ok());

  merged_runtime.SetSessionActive(false, 1U);
  merged_runtime.SetSessionActive(true, 2U);
  LedCommand new_led{mentor_pi::mcu::kLastHostLedId, 100U, 0U, 0U};
  CHECK(merged_runtime.PublishLedCommand(new_led).result.ok());
  RgbCommand new_rgb{};
  new_rgb.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  new_rgb.blue[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(merged_runtime.PublishRgbCommand(new_rgb).result.ok());
  OledCommand new_oled{};
  new_oled.update_mask = 0x02U;
  new_oled.lines[1].size = 1U;
  new_oled.lines[1].bytes[0] = 'B';
  CHECK(merged_runtime.PublishOledCommand(new_oled).result.ok());
  merged_platform.SetTimeMs(250U);
  merged_runtime.RunBusServoOnce();
  merged_runtime.RunPeripheralOnce();

  CHECK(merged_platform.bus_exchanges == 0U);
  CHECK(merged_platform.buzzer_hz == 0U);
  CHECK(!merged_platform.leds[0]);
  CHECK(!merged_platform.leds[1]);
  CHECK(merged_platform.leds[2]);
  mentor_pi::mcu::RgbState expected_rgb{};
  expected_rgb.blue[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(merged_platform.rgb_frame ==
        mentor_pi::mcu::drivers::EncodeRgbFrame(expected_rgb));
  bool old_oled_line_visible = false;
  bool new_oled_line_visible = false;
  for (std::size_t column = 0U; column < mentor_pi::mcu::drivers::kSsd1306Width;
       ++column) {
    old_oled_line_visible =
        old_oled_line_visible || merged_platform.oled_framebuffer[column] != 0U;
    new_oled_line_visible =
        new_oled_line_visible ||
        merged_platform
                .oled_framebuffer[mentor_pi::mcu::drivers::kSsd1306Width +
                                  column] != 0U;
  }
  CHECK(!old_oled_line_visible);
  CHECK(new_oled_line_visible);
  CHECK(motor_platform.critical_depth == 0U);
  CHECK(pwm_platform.critical_depth == 0U);
  CHECK(merged_platform.critical_depth == 0U);
  return true;
}

bool TestCrossSessionAppliedFieldHold() {
  // A field which reached its owner is held across reconnect. The first
  // disjoint update in the new session changes only its selected field.
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  LedCommand led_one{mentor_pi::mcu::kFirstHostLedId, 100U, 0U, 0U};
  CHECK(runtime.PublishLedCommand(led_one).result.ok());
  RgbCommand rgb_one{};
  rgb_one.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  rgb_one.red[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(runtime.PublishRgbCommand(rgb_one).result.ok());
  OledCommand oled_one{};
  oled_one.update_mask = 0x01U;
  oled_one.lines[0].size = 1U;
  oled_one.lines[0].bytes[0] = 'A';
  CHECK(runtime.PublishOledCommand(oled_one).result.ok());
  platform.SetTimeMs(250U);
  runtime.RunPeripheralOnce();
  platform.SetTimeMs(251U);
  runtime.RunPeripheralOnce();
  CHECK(platform.leds[mentor_pi::mcu::kFirstHostLedId - 1U]);

  std::array<std::uint8_t, mentor_pi::mcu::drivers::kSsd1306Width>
      first_oled_page{};
  for (std::size_t column = 0U; column < first_oled_page.size(); ++column) {
    first_oled_page[column] = platform.oled_framebuffer[column];
  }

  runtime.SetSessionActive(false, 1U);
  runtime.SetSessionActive(true, 2U);
  LedCommand led_two{mentor_pi::mcu::kLastHostLedId, 100U, 0U, 0U};
  CHECK(runtime.PublishLedCommand(led_two).result.ok());
  OledCommand oled_two{};
  oled_two.update_mask = 0x02U;
  oled_two.lines[1].size = 1U;
  oled_two.lines[1].bytes[0] = 'B';
  CHECK(runtime.PublishOledCommand(oled_two).result.ok());
  platform.SetTimeMs(500U);
  runtime.RunPeripheralOnce();

  CHECK(platform.leds[mentor_pi::mcu::kSystemHeartbeatLedIndex]);
  CHECK(platform.leds[mentor_pi::mcu::kFirstHostLedId - 1U]);
  CHECK(platform.leds[mentor_pi::mcu::kLastHostLedId - 1U]);
  mentor_pi::mcu::RgbState expected_rgb{};
  expected_rgb.red[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  CHECK(platform.rgb_frame ==
        mentor_pi::mcu::drivers::EncodeRgbFrame(expected_rgb));
  bool second_oled_line_visible = false;
  for (std::size_t column = 0U; column < first_oled_page.size(); ++column) {
    CHECK(platform.oled_framebuffer[column] == first_oled_page[column]);
    second_oled_line_visible =
        second_oled_line_visible ||
        platform.oled_framebuffer[mentor_pi::mcu::drivers::kSsd1306Width +
                                  column] != 0U;
  }
  CHECK(second_oled_line_visible);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestGatewayRevocationRace() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  // EnterCritical models preemption after the gateway's fast check but before
  // it owns the lock. Teardown and gen-2 activation complete first; the old
  // callback must fail its captured-generation recheck and publish nothing.
  platform.critical_transition_runtime = &runtime;
  const auto transition_on_lock = [&platform](std::uint32_t from,
                                              std::uint32_t to) {
    platform.transition_from_generation = from;
    platform.transition_to_generation = to;
    platform.transition_session_on_critical_enter = true;
  };
  transition_on_lock(1U, 2U);
  MotorCommand raced{};
  raced.update_mask = 0x01U;
  raced.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(raced, 0U).result.code ==
        ResultCode::kBusy);

  MotorCommand current{};
  current.update_mask = 0x02U;
  current.target_rps[1] = 1.0F;
  CHECK(runtime.PublishMotorCommand(current, 0U).result.ok());
  runtime.RunMotorControlOnce();
  CHECK(platform.motor_arm_calls_by_channel[0] == 0U);
  CHECK(platform.motor_arm_calls_by_channel[1] == 1U);

  PwmServoCommand pwm{};
  pwm.update_mask = 0x01U;
  pwm.duration_ms = 20U;
  pwm.pulse_width_us[0] = 1600U;
  transition_on_lock(2U, 3U);
  CHECK(runtime.PublishPwmServoCommand(pwm).result.code == ResultCode::kBusy);

  BusServoCommand bus{};
  bus.count = 1U;
  bus.servo_id[0] = 1U;
  bus.position[0] = 500U;
  bus.duration_ms = 20U;
  transition_on_lock(3U, 4U);
  CHECK(runtime.PublishBusServoCommand(bus).result.code == ResultCode::kBusy);

  LedCommand led{mentor_pi::mcu::kFirstHostLedId, 100U, 0U, 0U};
  transition_on_lock(4U, 5U);
  CHECK(runtime.PublishLedCommand(led).result.code == ResultCode::kBusy);

  BuzzerCommand buzzer{1000U, 100U, 0U, 0U};
  transition_on_lock(5U, 6U);
  CHECK(runtime.PublishBuzzerCommand(buzzer).result.code == ResultCode::kBusy);

  RgbCommand rgb{};
  rgb.update_mask = mentor_pi::mcu::kHostRgbPixelMask;
  rgb.red[mentor_pi::mcu::kHostRgbPixelIndex] = 255U;
  transition_on_lock(6U, 7U);
  CHECK(runtime.PublishRgbCommand(rgb).result.code == ResultCode::kBusy);

  OledCommand oled{};
  oled.update_mask = 0x01U;
  oled.lines[0].size = 1U;
  oled.lines[0].bytes[0] = 'A';
  transition_on_lock(7U, 8U);
  CHECK(runtime.PublishOledCommand(oled).result.code == ResultCode::kBusy);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestSafetySupervisorRevokesMotorAuthority() {
  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);
  MotorCommand motion{};
  motion.update_mask = 1U;
  motion.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
  runtime.RunMotorControlOnce();
  for (std::uint32_t millisecond = 1U; millisecond <= 10U; ++millisecond) {
    platform.SetTimeMs(millisecond);
    runtime.RunMotorControlOnce();
  }
  CHECK(platform.motor_armed[0]);
  CHECK(platform.motor_duty[0] < 0);

  // Keep a fresh command pending so the next MotorTask iteration is an
  // adversarial re-arm attempt after the supervisor stop.
  CHECK(runtime.PublishMotorCommand(motion, 10000U).result.ok());
  platform.SetTimeMs(250U);
  runtime.RunSafetySupervisorOnce();
  CHECK(!platform.motor_armed[0]);
  CHECK(platform.motor_duty[0] == 0);
  runtime.SetSessionActive(true, 2U);
  CHECK(runtime.PublishMotorCommand(motion, 250000U).result.code ==
        ResultCode::kBusy);
  runtime.RunMotorControlOnce();
  CHECK(!platform.motor_armed[0]);
  CHECK(platform.motor_duty[0] == 0);
  mentor_pi_mcu::app::microros::HealthSnapshot health{};
  runtime.ReadHealth(&health);
  CHECK(health.output_processing_fault);
  CHECK(platform.critical_depth == 0U);
  return true;
}

bool TestFatalMotorOutputRevocation() {
  MotorCommand motion{};
  motion.update_mask = 1U;
  motion.target_rps[0] = 1.0F;

  {
    FakePlatform platform;
    platform.arm_result = {ResultCode::kIoError, 1U};
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);
    CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
    runtime.RunMotorControlOnce();
    CHECK(!platform.motor_armed[0]);
    runtime.SetSessionActive(true, 2U);
    CHECK(runtime.PublishMotorCommand(motion, 1000U).result.code ==
          ResultCode::kBusy);
    mentor_pi_mcu::app::microros::HealthSnapshot health{};
    runtime.ReadHealth(&health);
    CHECK(health.output_processing_fault);
    CHECK(platform.critical_depth == 0U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);
    CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
    runtime.RunMotorControlOnce();
    platform.read_encoders_ok = false;
    for (std::uint32_t release = 1U; release <= 9U; ++release) {
      platform.SetTimeMs(release);
      runtime.RunMotorControlOnce();
    }
    CHECK(!platform.motor_armed[0]);
    runtime.SetSessionActive(true, 2U);
    CHECK(runtime.PublishMotorCommand(motion, 10000U).result.code ==
          ResultCode::kBusy);
    mentor_pi_mcu::app::microros::HealthSnapshot health{};
    runtime.ReadHealth(&health);
    CHECK(health.output_processing_fault);
    CHECK(platform.critical_depth == 0U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);
    CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());
    runtime.RunMotorControlOnce();
    platform.apply_result = {ResultCode::kIoError, 1U};
    for (std::uint32_t release = 1U; release <= 9U; ++release) {
      platform.SetTimeMs(release);
      runtime.RunMotorControlOnce();
    }
    CHECK(platform.apply_observed_inside_critical);
    CHECK(!platform.motor_armed[0]);
    CHECK(platform.motor_duty[0] == 0);
    runtime.SetSessionActive(true, 2U);
    CHECK(runtime.PublishMotorCommand(motion, 10000U).result.code ==
          ResultCode::kBusy);
    mentor_pi_mcu::app::microros::HealthSnapshot health{};
    runtime.ReadHealth(&health);
    CHECK(health.output_processing_fault);
    CHECK(platform.critical_depth == 0U);
  }
  return true;
}

bool TestMotorCalibrationGate() {
  FakePlatform platform;
  MotorControlConfiguration invalid_configuration{};
  invalid_configuration.maximum_accepted_rps = 0.0F;
  ControllerRuntime locked(invalid_configuration);
  CHECK(locked.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(locked.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&locked, &platform));
  locked.SetSessionActive(true, 1U);

  MotorCommand motion{};
  motion.update_mask = 0x0fU;
  motion.target_rps.fill(0.1F);
  CHECK(locked.PublishMotorCommand(motion, 0U).result.code ==
        ResultCode::kUnsupported);
  MotorCommand mixed_motion{};
  mixed_motion.update_mask = 3U;
  mixed_motion.target_rps[1] = 0.1F;
  CHECK(locked.PublishMotorCommand(mixed_motion, 0U).result.code ==
        ResultCode::kUnsupported);
  MotorCommand malformed{};
  malformed.update_mask = 1U;
  malformed.target_rps[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(locked.PublishMotorCommand(malformed, 0U).result.code ==
        ResultCode::kOutOfRange);
  WorkerDiagnostics locked_diagnostics{};
  locked.ReadWorkerDiagnostics(&locked_diagnostics);
  CHECK(locked_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{3U, 2U, 1U, 1U}));

  MotorCommand zero_mask{};
  CHECK(locked.PublishMotorCommand(zero_mask, 0U).result.code ==
        ResultCode::kInvalidArgument);
  MotorCommand unknown_only_mask{};
  unknown_only_mask.update_mask = 0x10U;
  CHECK(locked.PublishMotorCommand(unknown_only_mask, 0U).result.code ==
        ResultCode::kInvalidArgument);
  MotorCommand mixed_invalid_mask{};
  mixed_invalid_mask.update_mask = 0x11U;
  CHECK(locked.PublishMotorCommand(mixed_invalid_mask, 0U).result.code ==
        ResultCode::kInvalidArgument);
  locked.ReadWorkerDiagnostics(&locked_diagnostics);
  CHECK(locked_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{3U, 2U, 1U, 1U}));
  locked.RunMotorControlOnce();
  CHECK(platform.motor_armed ==
        (std::array<bool, 4>{false, false, false, false}));
  locked.ReadWorkerDiagnostics(&locked_diagnostics);
  CHECK(locked_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{3U, 2U, 1U, 1U}));

  const ServiceToken model_token{1U, 1U};
  CHECK(locked.DispatchMotorModel(model_token, MotorModel::kJgb37));
  locked.RunMotorControlOnce();
  mentor_pi_mcu::app::microros::MotorModelReply model_reply{};
  CHECK(locked.PollMotorModel(model_token, &model_reply));
  CHECK(model_reply.result.ok());
  CHECK(locked.PublishMotorCommand(motion, 1000U).result.code ==
        ResultCode::kUnsupported);

  SetMotorAdrcCommand locked_adrc = DefaultMotorAdrcCommand(1U);
  locked_adrc.velocity_filter_new_weight[0] = 1.0F;
  const ServiceToken locked_adrc_token{1U, 2U};
  CHECK(locked.DispatchMotorAdrc(locked_adrc_token, locked_adrc));
  locked.RunMotorControlOnce();
  mentor_pi_mcu::app::microros::MotorAdrcReply locked_adrc_reply{};
  CHECK(locked.PollMotorAdrc(locked_adrc_token, &locked_adrc_reply));
  CHECK(locked_adrc_reply.result.code == ResultCode::kUnsupported);
  CHECK(locked_adrc_reply.applied_mask == 0U);

  locked.InvalidateSessionWork(1U);
  locked.SetSessionActive(true, 2U);
  CHECK(locked.PublishMotorCommand(motion, 2000U).result.code ==
        ResultCode::kUnsupported);
  MotorCommand zero{};
  zero.update_mask = 0x0fU;
  CHECK(locked.PublishMotorCommand(zero, 2000U).result.ok());
  locked.RunMotorControlOnce();
  CHECK(platform.motor_armed ==
        (std::array<bool, 4>{false, false, false, false}));
  locked.ReadWorkerDiagnostics(&locked_diagnostics);
  CHECK(locked_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{5U, 4U, 3U, 3U}));

  const MotorControlConfiguration default_adrc =
      DefaultAdrcMotorControlConfiguration();
  FakePlatform capped_platform;
  ControllerRuntime capped(default_adrc);
  CHECK(capped.Configure(capped_platform.Hooks(), AxisTransform{}));
  CHECK(capped.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&capped, &capped_platform));
  capped.SetSessionActive(true, 1U);
  MotorCommand capped_command{};
  capped_command.update_mask = 1U;
  capped_command.target_rps[0] =
      mentor_pi::mcu::kMotorImplementationMaximumRps + 0.01F;
  CHECK(capped.PublishMotorCommand(capped_command, 0U).result.code ==
        ResultCode::kOutOfRange);
  WorkerDiagnostics capped_diagnostics{};
  capped.ReadWorkerDiagnostics(&capped_diagnostics);
  CHECK(capped_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{1U, 0U, 0U, 0U}));
  capped_command.target_rps[0] = 0.25F;
  CHECK(capped.PublishMotorCommand(capped_command, 0U).result.ok());
  capped.RunMotorControlOnce();
  capped.ReadWorkerDiagnostics(&capped_diagnostics);
  CHECK(capped_diagnostics.motor_command_rejections ==
        (std::array<std::uint32_t, 4>{1U, 0U, 0U, 0U}));
  for (std::uint32_t release = 1U; release <= 10U; ++release) {
    capped_platform.SetTimeMs(release);
    capped.RunMotorControlOnce();
  }
  CHECK(capped_platform.motor_armed[0]);
  CHECK(capped_platform.motor_duty[0] >=
        -mentor_pi::mcu::kMotorOutputLimitPermille);
  CHECK(capped_platform.motor_duty[0] <=
        mentor_pi::mcu::kMotorOutputLimitPermille);
  CHECK(capped_platform.critical_depth == 0U);

  FakePlatform adrc_platform;
  ControllerRuntime adrc_runtime(FullRangeTestMotorConfiguration());
  CHECK(adrc_runtime.Configure(adrc_platform.Hooks(), AxisTransform{}));
  CHECK(adrc_runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&adrc_runtime, &adrc_platform));
  adrc_runtime.SetSessionActive(true, 1U);

  SetMotorAdrcCommand adrc = DefaultMotorAdrcCommand(5U);
  for (const std::size_t index : {0U, 2U}) {
    adrc.known_velocity_decay_rate_s_inverse[index] = 1.0F;
    adrc.controller_fal_exponent[index] = 0.8F;
    adrc.controller_fal_threshold_rps[index] = 0.2F;
    adrc.observer_velocity_fal_exponent[index] = 0.7F;
    adrc.observer_disturbance_fal_exponent[index] = 0.6F;
    adrc.observer_fal_threshold_rps[index] = 0.15F;
    adrc.disturbance_leakage_s_inverse[index] = 0.5F;
    adrc.disturbance_estimate_limit_rps_per_second[index] = 20.0F;
    adrc.velocity_filter_new_weight[index] = 1.0F;
    adrc.positive_minimum_drive_permille[index] = 80U;
    adrc.negative_minimum_drive_permille[index] = 90U;
  }
  const ServiceToken canceled_adrc_token{1U, 1U};
  CHECK(adrc_runtime.DispatchMotorAdrc(canceled_adrc_token, adrc));
  CHECK(adrc_runtime.CancelMotorAdrc(canceled_adrc_token));
  adrc_runtime.RunMotorControlOnce();
  mentor_pi_mcu::app::microros::MotorAdrcReply adrc_reply{};
  CHECK(!adrc_runtime.PollMotorAdrc(canceled_adrc_token, &adrc_reply));

  const ServiceToken adrc_token{1U, 2U};
  CHECK(adrc_runtime.DispatchMotorAdrc(adrc_token, adrc));
  CHECK(!adrc_runtime.DispatchMotorAdrc({1U, 3U}, adrc));
  adrc_runtime.RunMotorControlOnce();
  CHECK(!adrc_runtime.CancelMotorAdrc(adrc_token));
  CHECK(adrc_runtime.PollMotorAdrc(adrc_token, &adrc_reply));
  CHECK(adrc_reply.result.ok());
  CHECK(adrc_reply.applied_mask == 5U);

  SetMotorAdrcCommand invalid_adrc = adrc;
  invalid_adrc.update_mask = 0U;
  const ServiceToken invalid_adrc_token{1U, 3U};
  CHECK(adrc_runtime.DispatchMotorAdrc(invalid_adrc_token, invalid_adrc));
  adrc_runtime.RunMotorControlOnce();
  CHECK(adrc_runtime.PollMotorAdrc(invalid_adrc_token, &adrc_reply));
  CHECK(adrc_reply.result.code == ResultCode::kInvalidArgument);
  CHECK(adrc_reply.applied_mask == 0U);

  SetMotorAdrcCommand invalid_expanded_adrc = adrc;
  invalid_expanded_adrc.observer_fal_threshold_rps[2] = 0.0F;
  const ServiceToken invalid_expanded_adrc_token{1U, 4U};
  CHECK(adrc_runtime.DispatchMotorAdrc(invalid_expanded_adrc_token,
                                      invalid_expanded_adrc));
  adrc_runtime.RunMotorControlOnce();
  CHECK(adrc_runtime.PollMotorAdrc(invalid_expanded_adrc_token, &adrc_reply));
  CHECK(adrc_reply.result.code == ResultCode::kOutOfRange);
  CHECK(adrc_reply.applied_mask == 0U);

  MotorCommand adrc_motion{};
  adrc_motion.update_mask = 1U;
  adrc_motion.target_rps[0] = 0.1F;
  CHECK(adrc_runtime.PublishMotorCommand(adrc_motion, 0U).result.ok());
  adrc_runtime.RunMotorControlOnce();
  const ServiceToken busy_adrc_token{1U, 5U};
  CHECK(adrc_runtime.DispatchMotorAdrc(busy_adrc_token, adrc));
  adrc_runtime.RunMotorControlOnce();
  CHECK(adrc_runtime.PollMotorAdrc(busy_adrc_token, &adrc_reply));
  CHECK(adrc_reply.result.code == ResultCode::kBusy);
  CHECK(adrc_reply.applied_mask == 0U);
  for (std::uint32_t release = 1U; release <= 10U; ++release) {
    adrc_platform.SetTimeMs(release);
    adrc_runtime.RunMotorControlOnce();
  }
  // The valid expanded update survived the rejected atomic update, and its
  // positive directional floor is applied in the semantic target direction.
  CHECK(adrc_platform.motor_duty[0] == -80);
  CHECK(adrc_platform.critical_depth == 0U);
  return true;
}

bool TestMotorControlUsesElapsedPeriod() {
  auto run_release_at = [](ControllerRuntime* runtime, FakePlatform* platform,
                           std::uint32_t now_us) {
    platform->now_us = now_us;
    platform->now_ms = now_us / 1000U;
    runtime->RunMotorControlOnce();
  };

  FakePlatform platform;
  ControllerRuntime runtime(FullRangeTestMotorConfiguration());
  CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
  CHECK(runtime.InitializeSafeBoot());
  CHECK(EstablishStartupReadiness(&runtime, &platform));
  runtime.SetSessionActive(true, 1U);

  SetMotorAdrcCommand calibration = DefaultMotorAdrcCommand(1U);
  calibration.input_gain_rps_per_second_per_permille[0] = 0.01F;
  calibration.disturbance_estimate_limit_rps_per_second[0] = 10.0F;
  calibration.velocity_filter_new_weight[0] = 1.0F;
  const ServiceToken token{1U, 1U};
  CHECK(runtime.DispatchMotorAdrc(token, calibration));
  run_release_at(&runtime, &platform, 1U);
  MotorAdrcReply reply{};
  CHECK(runtime.PollMotorAdrc(token, &reply));
  CHECK(reply.result.ok());

  MotorCommand command{};
  command.update_mask = 1U;
  command.target_rps[0] = 1.0F;
  CHECK(runtime.PublishMotorCommand(command, 1U).result.ok());
  for (std::uint32_t release = 1U; release <= 8U; ++release) {
    run_release_at(&runtime, &platform,
                   release == 8U ? 10000U : release * 1000U);
  }
  CHECK(platform.motor_duty[0] == -400);

  // The motor task passes actual elapsed time into ADRC. Refresh the command
  // after a long scheduler gap so the observer-timing guard, rather than the
  // command lease, is what fails closed.
  CHECK(runtime.PublishMotorCommand(command, 1000000U).result.ok());
  for (std::uint32_t release = 0U; release <= 10U; ++release) {
    run_release_at(&runtime, &platform, 1000000U + release * 1000U);
  }
  CHECK(platform.motor_duty[0] == 0);
  CHECK(!platform.motor_armed[0]);

  return true;
}

bool TestMotorCommandAgeDiagnostics() {
  MotorCommand zero{};
  zero.update_mask = 1U;

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);

    platform.now_us = 20000U;
    CHECK(runtime.PublishMotorCommand(zero, 0U).result.ok());
    runtime.RunMotorControlOnce();

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 1U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 0U);
    CHECK(diagnostics.motor_command_max_age_us == 20000U);

    platform.now_us = 20001U;
    CHECK(runtime.PublishMotorCommand(zero, 0U).result.ok());
    runtime.RunMotorControlOnce();
    platform.now_us = 99999U;
    CHECK(runtime.PublishMotorCommand(zero, 0U).result.ok());
    runtime.RunMotorControlOnce();
    platform.now_us = 100000U;
    CHECK(runtime.PublishMotorCommand(zero, 0U).result.ok());
    runtime.RunMotorControlOnce();

    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 4U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 3U);
    CHECK(diagnostics.motor_command_max_age_us == 100000U);

    platform.now_us = 5U;
    CHECK(runtime
              .PublishMotorCommand(
                  zero, std::numeric_limits<std::uint32_t>::max() - 4U)
              .result.ok());
    runtime.RunMotorControlOnce();
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 5U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 3U);
    CHECK(diagnostics.motor_command_max_age_us == 100000U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);

    MotorCommand first{};
    first.update_mask = 1U;
    MotorCommand second{};
    second.update_mask = 2U;
    CHECK(runtime.PublishMotorCommand(first, 1000U).result.ok());
    CHECK(runtime.PublishMotorCommand(second, 5000U).result.ok());
    platform.now_us = 10000U;
    runtime.RunMotorControlOnce();

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 1U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 0U);
    CHECK(diagnostics.motor_command_max_age_us == 9000U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    CHECK(EstablishStartupReadiness(&runtime, &platform));
    runtime.SetSessionActive(true, 1U);
    CHECK(runtime.PublishMotorCommand(zero, 0U).result.ok());

    runtime.InvalidateSessionWork(1U);
    runtime.SetSessionActive(false, 1U);
    runtime.SetSessionActive(true, 2U);
    platform.now_us = 30000U;
    runtime.RunMotorControlOnce();

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 0U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 0U);
    CHECK(diagnostics.motor_command_max_age_us == 0U);
  }

  {
    FakePlatform platform;
    ControllerRuntime runtime(FullRangeTestMotorConfiguration());
    CHECK(runtime.Configure(platform.Hooks(), AxisTransform{}));
    CHECK(runtime.InitializeSafeBoot());
    runtime.SetSessionActive(true, 1U);

    MotorCommand motion{};
    motion.update_mask = 1U;
    motion.target_rps[0] = 5.0F;
    CHECK(runtime.PublishMotorCommand(motion, 0U).result.ok());

    const ServiceToken model_token{1U, 1U};
    CHECK(runtime.DispatchMotorModel(model_token, MotorModel::kJgb528));
    platform.SetTimeMs(1U);
    runtime.RunMotorControlOnce();
    MotorModelReply model_reply{};
    CHECK(runtime.PollMotorModel(model_token, &model_reply));
    CHECK(model_reply.result.ok());

    CHECK(EstablishStartupReadiness(&runtime, &platform));
    platform.now_us = 50000U;
    runtime.RunMotorControlOnce();

    WorkerDiagnostics diagnostics{};
    runtime.ReadWorkerDiagnostics(&diagnostics);
    CHECK(diagnostics.motor_command_consumptions == 0U);
    CHECK(diagnostics.motor_command_age_over_20_ms == 0U);
    CHECK(diagnostics.motor_command_max_age_us == 0U);
    CHECK(diagnostics.motor_command_rejections[0] == 1U);
  }

  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "--motor-adrc-focused") == 0) {
    if (!TestMotorAdrcWireAdapter() || !TestMotorCalibrationGate() ||
        !TestMotorControlUsesElapsedPeriod()) {
      return 1;
    }
    std::puts("focused motor ADRC controller tests passed");
    return 0;
  }
  if (argc != 1) {
    std::fprintf(stderr, "usage: %s [--motor-adrc-focused]\n", argv[0]);
    return 2;
  }
  if (!TestMotorAdrcWireAdapter() || !TestStatusRgbSemantics() ||
      !TestStatusRgbControllerIntegration() ||
      !TestPlatformHookCompletenessContract() ||
      !TestBuzzerFailuresAreObservable() || !TestMicroRosAdapterDelegates() ||
      !TestMicroRosMicrosecondAcceptanceBoundary() ||
      !TestPwmPhysicalFrameTiming() ||
      !TestPwmChannelThreeRepeatedCommandsAndSessionTransitions() ||
      !TestImuCharacterizationSnapshot() ||
      !TestImuCharacterizationRequiresSuccessfulRawRead() ||
      !TestImuPersistentBusyIsDiagnosedAndReset() ||
      !TestImuPersistentInitializeBusyIsDiagnosedAndRecovers() ||
      !TestSensorSchedulesRemainPhaseStable() || !TestControllerIntegration() ||
      !TestBusServoServicesAndStopPreemption() || !TestSafetyStartupGrace() ||
      !TestRetainedWatchdogTaskSemantics() ||
      !TestSessionTeardownInvalidatesOwnedWork() ||
      !TestMotorSessionRevocationOrdering() ||
      !TestSessionGenerationWatermark() || !TestStartupMotorInhibit() ||
      !TestCrossSessionMergedFieldOwnership() ||
      !TestCrossSessionAppliedFieldHold() || !TestGatewayRevocationRace() ||
      !TestSafetySupervisorRevokesMotorAuthority() ||
      !TestFatalMotorOutputRevocation() || !TestMotorCalibrationGate() ||
      !TestMotorControlUsesElapsedPeriod() ||
      !TestMotorCommandAgeDiagnostics()) {
    return 1;
  }
  std::puts("controller integration tests passed");
  return 0;
}
