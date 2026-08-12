// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"
}

#include "mentor_pi_mcu/app/controller/controller_runtime.h"
#include "mentor_pi_mcu/app/controller/platform_hooks.h"
#include "mentor_pi_mcu/app/microros/runtime.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/battery_adc.h"
#include "mentor_pi_mcu/drivers/hal.h"
#include "mentor_pi_mcu/drivers/qmi8658.h"
#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/platform.h"
#include "mentor_pi_mcu/platform/stm32/status.h"
#include "mentor_pi_mcu/platform/stm32/task_entries.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"
#include "mentor_pi_mcu/platform/stm32/watchdog_retention.h"

extern "C" void Error_Handler();

// Newlib's rand() lazily allocates per-thread state with malloc(). General
// libc allocation is intentionally disabled in this firmware, so provide the
// allocation-free PRNG needed to seed the micro-ROS XRCE client key.
namespace {
std::uint32_t g_embedded_random_state{1U};
}  // namespace

extern "C" void srand(unsigned int seed) {
  g_embedded_random_state = seed == 0U ? 1U : seed;
}

extern "C" int rand() {
  std::uint32_t value = g_embedded_random_state;
  value ^= value << 13U;
  value ^= value >> 17U;
  value ^= value << 5U;
  g_embedded_random_state = value;
  return static_cast<int>(value & 0x7FFFFFFFU);
}

extern "C" {
extern std::uint8_t __ram_used_end__;
extern std::uint8_t __ccm_end__;
extern std::uint8_t __flash_image_end__;
}

namespace {

namespace controller = mentor_pi_mcu::app::controller;
namespace microros = mentor_pi_mcu::app::microros;
namespace platform = mentor_pi_mcu::platform::stm32;
static_assert(controller::kControllerTaskCount == platform::kWatchdogTaskCount,
              "retained watchdog task encoding must match controller tasks");
using mentor_pi::mcu::OkResult;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi::mcu::drivers::BatteryAdcCalibration;
using mentor_pi::mcu::drivers::IoStatus;

constexpr std::uintptr_t kFlashStart = 0x08000000U;
constexpr std::uintptr_t kSramStart = 0x20000000U;
constexpr std::uintptr_t kSramEnd = 0x20020000U;
constexpr std::uintptr_t kCcmStart = 0x10000000U;
constexpr std::uintptr_t kCcmEnd = 0x10010000U;
constexpr std::uint32_t kFlashSizeBytes = 512U * 1024U;
constexpr std::uint32_t kFactoryCalibrationMillivolts = 3300U;
constexpr std::uint32_t kAdcFullScale = 4095U;

struct TargetContext {
  std::uint32_t last_cycle_count{0U};
  std::uint64_t elapsed_cycles{0U};
  TickType_t safety_last_wake_tick{0U};
  BatteryAdcCalibration battery_calibration{};
  std::uint32_t bus_deadline_ms{0U};
  std::uint32_t rgb_deadline_us{0U};
  bool battery_conversion_started{false};
  bool bus_exchange_active{false};
  bool rgb_transfer_active{false};
  bool safety_delay_initialized{false};
};

TargetContext g_target_context{};
StaticTask_t g_idle_task_control_block{};
std::array<StackType_t, configMINIMAL_STACK_SIZE> g_idle_task_stack{};

[[noreturn]] void FailStop() {
  Error_Handler();
  for (;;) {
    __NOP();
  }
}

Result FromPlatformStatus(platform::Status status, std::uint16_t detail = 0U) {
  switch (status) {
    case platform::Status::kOk:
      return OkResult();
    case platform::Status::kInvalidArgument:
      return {ResultCode::kInvalidArgument, detail};
    case platform::Status::kBusy:
      return {ResultCode::kBusy, detail};
    case platform::Status::kTimeout:
      return {ResultCode::kTimeout, detail};
    case platform::Status::kOverflow:
      return {ResultCode::kOutOfRange, detail};
    case platform::Status::kNotInitialized:
    case platform::Status::kIoError:
      return {ResultCode::kIoError, detail};
  }
  return {ResultCode::kIoError, detail};
}

IoStatus ToIoStatus(platform::Status status) {
  switch (status) {
    case platform::Status::kOk:
      return IoStatus::kOk;
    case platform::Status::kBusy:
      return IoStatus::kBusy;
    case platform::Status::kTimeout:
      return IoStatus::kTimeout;
    case platform::Status::kInvalidArgument:
    case platform::Status::kNotInitialized:
    case platform::Status::kIoError:
    case platform::Status::kOverflow:
      return IoStatus::kIoError;
  }
  return IoStatus::kIoError;
}

TargetContext* Context(void* context) {
  return static_cast<TargetContext*>(context);
}

std::uint32_t MonotonicMilliseconds(void* context) {
  static_cast<void>(context);
  return platform::MonotonicMilliseconds();
}

std::uint32_t MonotonicMicroseconds(void* context) {
  TargetContext* const target = Context(context);
  if (target == nullptr) {
    return 0U;
  }

  // DWT is a wrapping 32-bit cycle counter. Sampling is serialized so the
  // accumulated value remains a true modulo-uint32 microsecond clock instead
  // of jumping backwards every 25.6 seconds at 168 MHz.
  const std::uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const std::uint32_t current = platform::CycleCounter();
  target->elapsed_cycles += current - target->last_cycle_count;
  target->last_cycle_count = current;
  const std::uint32_t microseconds = static_cast<std::uint32_t>(
      target->elapsed_cycles / platform::kSystemClockHz * 1000000ULL +
      target->elapsed_cycles % platform::kSystemClockHz / 168ULL);
  if (primask == 0U) {
    __enable_irq();
  }
  return microseconds;
}

bool RemainingMilliseconds(std::uint32_t deadline_ms, std::uint32_t now_ms,
                           std::uint32_t* timeout_ms) {
  if (timeout_ms == nullptr) {
    return false;
  }
  const std::uint32_t remaining = deadline_ms - now_ms;
  if (remaining == 0U || remaining >= 0x80000000U) {
    return false;
  }
  *timeout_ms = remaining > 10U ? 10U : remaining;
  return true;
}

bool RemainingMillisecondsFromMicroseconds(void* context,
                                           std::uint32_t deadline_us,
                                           std::uint32_t* timeout_ms) {
  if (timeout_ms == nullptr) {
    return false;
  }
  const std::uint32_t now_us = MonotonicMicroseconds(context);
  const std::uint32_t remaining_us = deadline_us - now_us;
  if (remaining_us == 0U || remaining_us >= 0x80000000U) {
    return false;
  }
  std::uint32_t rounded_ms = (remaining_us + 999U) / 1000U;
  if (rounded_ms > 10U) {
    rounded_ms = 10U;
  }
  *timeout_ms = rounded_ms;
  return true;
}

void WaitForTask(void* context, controller::ControllerTask task,
                 std::uint32_t maximum_ms) {
  TickType_t ticks = pdMS_TO_TICKS(maximum_ms);
  if (ticks == 0U) {
    ticks = 1U;
  }
  if (task == controller::ControllerTask::kSafetySupervisor) {
    TargetContext* const target = Context(context);
    if (target == nullptr) {
      vTaskDelay(ticks);
      return;
    }
    if (!target->safety_delay_initialized) {
      target->safety_last_wake_tick = xTaskGetTickCount();
      target->safety_delay_initialized = true;
    }
    vTaskDelayUntil(&target->safety_last_wake_tick, ticks);
    return;
  }
  // Interrupt notifications are coalesced. Workers always consume the latest
  // bounded state and never replay an ISR backlog.
  static_cast<void>(ulTaskNotifyTake(pdTRUE, ticks));
}

void EnterCritical(void* context) {
  static_cast<void>(context);
  taskENTER_CRITICAL();
}

void ExitCritical(void* context) {
  static_cast<void>(context);
  taskEXIT_CRITICAL();
}

void EmergencyStopMotors(void* context) {
  static_cast<void>(context);
  platform::EmergencyStopMotors();
}

Result RefreshWatchdog(void* context) {
  static_cast<void>(context);
  return FromPlatformStatus(platform::RefreshWatchdogFromSafetySupervisor());
}

void PersistWatchdogTask(void* context, controller::ControllerTask task) {
  static_cast<void>(context);
  platform::PersistWatchdogTask(static_cast<std::uint8_t>(task));
}

Result InitializeMotorOutputs(void* context) {
  static_cast<void>(context);
  platform::EmergencyStopMotors();
  return platform::MotorArmedMask() == 0U ? OkResult()
                                          : Result{ResultCode::kIoError, 1U};
}

Result ArmMotor(void* context, std::size_t motor_index) {
  static_cast<void>(context);
  // MotorControlTask calls this while holding the controller critical section.
  // The authority write is enclosed by transport-latch checks on both sides.
  if (platform::TransportHasFatalError()) {
    platform::EmergencyStopMotors();
    return {ResultCode::kBusy, static_cast<std::uint16_t>(motor_index + 1U)};
  }
  const platform::Status status = platform::ArmMotorOutput(motor_index);
  if (platform::TransportHasFatalError()) {
    platform::EmergencyStopMotors();
    return {ResultCode::kBusy, static_cast<std::uint16_t>(motor_index + 1U)};
  }
  return FromPlatformStatus(status,
                            static_cast<std::uint16_t>(motor_index + 1U));
}

void DisarmMotor(void* context, std::size_t motor_index) {
  static_cast<void>(context);
  platform::DisarmMotorOutput(motor_index);
}

bool ReadEncoderCounters(
    void* context,
    std::array<std::uint32_t, mentor_pi::mcu::kMotorCount>* counters) {
  static_cast<void>(context);
  if (counters == nullptr) {
    return false;
  }
  *counters = platform::ReadEncoderCounters();
  return true;
}

Result ApplyMotorDuty(
    void* context, const std::array<std::int16_t, mentor_pi::mcu::kMotorCount>&
                       duty_permille) {
  static_cast<void>(context);
  // This check and the complete four-channel update are covered by
  // MotorControlTask's controller critical section. The transport latch is
  // cleared only by OpenUsart1Transport() for a fresh physical session.
  if (platform::TransportHasFatalError()) {
    platform::EmergencyStopMotors();
    return {ResultCode::kBusy, 0U};
  }
  for (std::size_t motor = 0U; motor < duty_permille.size(); ++motor) {
    const platform::Status status =
        platform::SetMotorDutyPermille(motor, duty_permille[motor]);
    if (status != platform::Status::kOk) {
      platform::EmergencyStopMotors();
      return FromPlatformStatus(status, static_cast<std::uint16_t>(motor + 1U));
    }
  }
  // Keep the defensive post-check after the complete four-channel update. A
  // priority-6 HAL RX/TX error callback latches before motor authority can be
  // returned.
  if (platform::TransportHasFatalError()) {
    platform::EmergencyStopMotors();
    return {ResultCode::kBusy, 0U};
  }
  return OkResult();
}

Result InitializePwmServos(void* context) {
  static_cast<void>(context);
  return FromPlatformStatus(platform::StartPwmServoFrameGenerator());
}

Result SetPwmServoShadow(
    void* context,
    const std::array<std::uint16_t, mentor_pi::mcu::kPwmServoCount>&
        pulse_width_us) {
  static_cast<void>(context);
  return FromPlatformStatus(platform::SetPwmServoPulseShadow(pulse_width_us));
}

std::uint32_t PwmServoFrameSequence(void* context) {
  static_cast<void>(context);
  return platform::PwmServoFrameSequence();
}

bool ReadButtonPressed(void* context, std::size_t button_index) {
  static_cast<void>(context);
  return platform::ReadButtonPressed(button_index);
}

void SetLed(void* context, std::size_t led_index, bool on) {
  static_cast<void>(context);
  static_cast<void>(platform::SetLed(led_index, on));
}

Result SetBuzzer(void* context, std::uint16_t frequency_hz, bool on) {
  static_cast<void>(context);
  return FromPlatformStatus(
      platform::SetBuzzerFrequency(on ? frequency_hz : 0U));
}

controller::BatterySample TakeBatterySample(void* context,
                                            std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  TargetContext* const target = Context(context);
  if (target == nullptr) {
    return {{ResultCode::kInvalidArgument, 0U}, {}, false};
  }
  if (!target->battery_conversion_started) {
    const platform::Status started = platform::StartBatteryAdcConversion();
    target->battery_conversion_started =
        started == platform::Status::kOk || started == platform::Status::kBusy;
    return {FromPlatformStatus(started), {}, false};
  }

  std::array<std::uint16_t, 2> samples{};
  const platform::Status completed = platform::TakeBatteryAdcResult(&samples);
  if (completed == platform::Status::kBusy) {
    return {{ResultCode::kBusy, 0U}, {}, false};
  }
  if (completed != platform::Status::kOk) {
    target->battery_conversion_started = false;
    return {FromPlatformStatus(completed), {}, false};
  }

  const auto reading = mentor_pi::mcu::drivers::ConvertBatteryAdc(
      samples[0], samples[1], target->battery_calibration);
  const platform::Status restarted = platform::StartBatteryAdcConversion();
  target->battery_conversion_started = restarted == platform::Status::kOk ||
                                       restarted == platform::Status::kBusy;
  return {OkResult(), reading, true};
}

IoStatus RegisterI2cRead(void* context, std::uint8_t address, std::uint8_t reg,
                         std::uint8_t* data, std::size_t size,
                         std::uint32_t deadline_us) {
  std::uint32_t timeout_ms = 0U;
  if (!RemainingMillisecondsFromMicroseconds(context, deadline_us,
                                             &timeout_ms)) {
    return IoStatus::kTimeout;
  }
  return ToIoStatus(platform::ImuI2cRead(address, reg, data, size, timeout_ms));
}

IoStatus RegisterI2cWrite(void* context, std::uint8_t address, std::uint8_t reg,
                          const std::uint8_t* data, std::size_t size,
                          std::uint32_t deadline_us) {
  std::uint32_t timeout_ms = 0U;
  if (!RemainingMillisecondsFromMicroseconds(context, deadline_us,
                                             &timeout_ms)) {
    return IoStatus::kTimeout;
  }
  return ToIoStatus(
      platform::ImuI2cWrite(address, reg, data, size, timeout_ms));
}

IoStatus RawI2cWrite(void* context, std::uint8_t address,
                     const std::uint8_t* data, std::size_t size,
                     std::uint32_t deadline_ms) {
  std::uint32_t timeout_ms = 0U;
  if (!RemainingMilliseconds(deadline_ms, MonotonicMilliseconds(context),
                             &timeout_ms)) {
    return IoStatus::kTimeout;
  }
  const platform::Status status = platform::OledI2cTransmit(
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(address) << 1U),
      data, size, timeout_ms);
  if (status == platform::Status::kTimeout ||
      status == platform::Status::kIoError) {
    static_cast<void>(platform::ResetOledI2c());
  }
  return ToIoStatus(status);
}

IoStatus BeginBusUartExchange(void* context, const std::uint8_t* tx,
                              std::size_t tx_size, std::size_t max_reply_size,
                              std::uint32_t deadline_ms) {
  TargetContext* const target = Context(context);
  if (target == nullptr || target->bus_exchange_active) {
    return IoStatus::kBusy;
  }
  std::uint32_t ignored_timeout = 0U;
  if (!RemainingMilliseconds(deadline_ms, MonotonicMilliseconds(context),
                             &ignored_timeout)) {
    return IoStatus::kTimeout;
  }
  const platform::Status status =
      platform::StartBusUartExchange(tx, tx_size, max_reply_size);
  if (status == platform::Status::kOk) {
    target->bus_deadline_ms = deadline_ms;
    target->bus_exchange_active = true;
  }
  return ToIoStatus(status);
}

IoStatus PollBusUartExchange(void* context, std::uint32_t now_ms,
                             std::uint8_t* reply, std::size_t capacity,
                             std::size_t* reply_size) {
  TargetContext* const target = Context(context);
  if (target == nullptr || !target->bus_exchange_active ||
      reply_size == nullptr) {
    return IoStatus::kIoError;
  }
  const std::uint32_t remaining = target->bus_deadline_ms - now_ms;
  if (remaining == 0U || remaining >= 0x80000000U) {
    platform::ResetBusUart();
    target->bus_exchange_active = false;
    return IoStatus::kTimeout;
  }
  const platform::Status status =
      platform::TakeBusUartCompletion(reply, capacity, reply_size);
  if (status != platform::Status::kBusy) {
    target->bus_exchange_active = false;
  }
  return ToIoStatus(status);
}

void CancelBusUart(void* context) {
  TargetContext* const target = Context(context);
  platform::ResetBusUart();
  if (target != nullptr) {
    target->bus_exchange_active = false;
  }
}

IoStatus BeginRgbTransmit(void* context, const std::uint8_t* data,
                          std::size_t size, std::uint32_t deadline_us) {
  TargetContext* const target = Context(context);
  if (target == nullptr || target->rgb_transfer_active) {
    return IoStatus::kBusy;
  }
  std::uint32_t ignored_timeout = 0U;
  if (!RemainingMillisecondsFromMicroseconds(context, deadline_us,
                                             &ignored_timeout)) {
    return IoStatus::kTimeout;
  }
  const platform::Status status = platform::StartRgbTransfer(data, size);
  if (status == platform::Status::kOk) {
    target->rgb_deadline_us = deadline_us;
    target->rgb_transfer_active = true;
  }
  return ToIoStatus(status);
}

IoStatus PollRgbTransmit(void* context, std::uint32_t now_us) {
  TargetContext* const target = Context(context);
  if (target == nullptr || !target->rgb_transfer_active) {
    return IoStatus::kIoError;
  }
  const platform::Status status = platform::RgbTransferStatus();
  if (status == platform::Status::kBusy) {
    const std::uint32_t remaining = target->rgb_deadline_us - now_us;
    if (remaining != 0U && remaining < 0x80000000U) {
      return IoStatus::kBusy;
    }
    platform::CancelRgbTransfer();
    target->rgb_transfer_active = false;
    return IoStatus::kTimeout;
  }
  target->rgb_transfer_active = false;
  return ToIoStatus(status);
}

void CancelRgbTransmit(void* context) {
  TargetContext* const target = Context(context);
  platform::CancelRgbTransfer();
  if (target != nullptr) {
    target->rgb_transfer_active = false;
  }
}

std::uint32_t TaskStackHighWaterBytes(void* context,
                                      controller::ControllerTask task) {
  static_cast<void>(context);
  static_assert(static_cast<std::uint8_t>(controller::ControllerTask::kCount) ==
                static_cast<std::uint8_t>(platform::TaskId::kCount));
  const auto task_id = static_cast<platform::TaskId>(task);
  const TaskHandle_t handle = platform::GetTaskHandle(task_id);
  if (handle == nullptr) {
    return 0U;
  }
  return static_cast<std::uint32_t>(uxTaskGetStackHighWaterMark(handle)) *
         static_cast<std::uint32_t>(sizeof(StackType_t));
}

std::uint32_t FreeRegionBytes(std::uintptr_t used_end,
                              std::uintptr_t region_start,
                              std::uintptr_t region_end) {
  if (used_end < region_start || used_end > region_end) {
    return 0U;
  }
  return static_cast<std::uint32_t>(region_end - used_end);
}

void ReadMemoryMetrics(void* context,
                       std::array<std::uint32_t, 2>* free_ram_bytes,
                       std::array<std::uint32_t, 2>* minimum_free_ram_bytes) {
  static_cast<void>(context);
  if (free_ram_bytes == nullptr || minimum_free_ram_bytes == nullptr) {
    return;
  }
  const std::array<std::uint32_t, 2> free_bytes{
      FreeRegionBytes(reinterpret_cast<std::uintptr_t>(&__ram_used_end__),
                      kSramStart, kSramEnd),
      FreeRegionBytes(reinterpret_cast<std::uintptr_t>(&__ccm_end__), kCcmStart,
                      kCcmEnd)};
  *free_ram_bytes = free_bytes;
  *minimum_free_ram_bytes = free_bytes;
}

std::uint32_t FlashUsedBytes(void* context) {
  static_cast<void>(context);
  const std::uintptr_t image_end =
      reinterpret_cast<std::uintptr_t>(&__flash_image_end__);
  return image_end >= kFlashStart
             ? static_cast<std::uint32_t>(image_end - kFlashStart)
             : 0U;
}

std::uint32_t FlashTotalBytes(void* context) {
  static_cast<void>(context);
  return kFlashSizeBytes;
}

std::uint8_t LastResetReason(void* context) {
  static_cast<void>(context);
  return platform::CapturedResetReason();
}

std::uint8_t CapturedWatchdogTask(void* context) {
  static_cast<void>(context);
  return platform::CapturedWatchdogTask();
}

controller::TransportActivity ReadTransportActivity(void* context) {
  static_cast<void>(context);
  const platform::TransportSnapshot snapshot = platform::GetTransportSnapshot();
  return {snapshot.rx_wire_bytes, snapshot.tx_wire_bytes, snapshot.open};
}

controller::PlatformHooks BuildPlatformHooks() {
  controller::PlatformHooks hooks{};
  hooks.context = &g_target_context;
  hooks.monotonic_milliseconds = &MonotonicMilliseconds;
  hooks.monotonic_microseconds = &MonotonicMicroseconds;
  hooks.wait_for_task = &WaitForTask;
  hooks.enter_critical = &EnterCritical;
  hooks.exit_critical = &ExitCritical;
  hooks.emergency_stop_motors = &EmergencyStopMotors;
  hooks.refresh_watchdog = &RefreshWatchdog;
  hooks.persist_watchdog_task = &PersistWatchdogTask;
  hooks.initialize_motor_outputs = &InitializeMotorOutputs;
  hooks.arm_motor = &ArmMotor;
  hooks.disarm_motor = &DisarmMotor;
  hooks.read_encoder_counters = &ReadEncoderCounters;
  hooks.apply_motor_duty = &ApplyMotorDuty;
  hooks.initialize_pwm_servos = &InitializePwmServos;
  hooks.set_pwm_servo_shadow = &SetPwmServoShadow;
  hooks.pwm_servo_frame_sequence = &PwmServoFrameSequence;
  hooks.read_button_pressed = &ReadButtonPressed;
  hooks.set_led = &SetLed;
  hooks.set_buzzer = &SetBuzzer;
  hooks.take_battery_sample = &TakeBatterySample;
  hooks.register_i2c_read = &RegisterI2cRead;
  hooks.register_i2c_write = &RegisterI2cWrite;
  hooks.raw_i2c_write = &RawI2cWrite;
  hooks.bus_uart_begin_exchange = &BeginBusUartExchange;
  hooks.bus_uart_poll_exchange = &PollBusUartExchange;
  hooks.bus_uart_cancel = &CancelBusUart;
  hooks.rgb_spi_begin_transmit = &BeginRgbTransmit;
  hooks.rgb_spi_poll_transmit = &PollRgbTransmit;
  hooks.rgb_spi_cancel = &CancelRgbTransmit;
  hooks.read_transport_activity = &ReadTransportActivity;
  hooks.task_stack_high_water_bytes = &TaskStackHighWaterBytes;
  hooks.read_memory_metrics = &ReadMemoryMetrics;
  hooks.flash_used_bytes = &FlashUsedBytes;
  hooks.flash_total_bytes = &FlashTotalBytes;
  hooks.last_reset_reason = &LastResetReason;
  hooks.captured_watchdog_task = &CapturedWatchdogTask;
  return hooks;
}

void InitializeBatteryCalibration() {
  const std::uint32_t factory_raw = platform::FactoryVrefintCalibration();
  const std::uint32_t reference_mv =
      (factory_raw * kFactoryCalibrationMillivolts + kAdcFullScale / 2U) /
      kAdcFullScale;
  g_target_context.battery_calibration.internal_reference_mv =
      reference_mv <= UINT16_MAX ? static_cast<std::uint16_t>(reference_mv)
                                 : 0U;
}

platform::TaskHooks ConvertTaskEntries(
    const controller::ControllerTaskEntries& entries) {
  static_assert(controller::kControllerTaskCount ==
                static_cast<std::size_t>(platform::TaskId::kCount));
  platform::TaskHooks hooks{};
  for (std::size_t index = 0U; index < hooks.size(); ++index) {
    hooks[index] = {entries[index].main, entries[index].context};
  }
  return hooks;
}

mentor_pi::mcu::MotorControlConfiguration BuildMotorConfiguration() {
  auto configuration = mentor_pi::mcu::DefaultAdrcMotorControlConfiguration();
  // Passive one-wheel-at-a-time captures establish connector order as
  // M1/front-left, M2/rear-left, M3/front-right, M4/rear-right.
  configuration.channel_wiring_sign = {1, 1, 1, 1};
  return configuration;
}

}  // namespace

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t** task_control_block,
                                              StackType_t** stack_buffer,
                                              std::uint32_t* stack_size) {
  if (task_control_block == nullptr || stack_buffer == nullptr ||
      stack_size == nullptr) {
    FailStop();
  }
  *task_control_block = &g_idle_task_control_block;
  *stack_buffer = g_idle_task_stack.data();
  *stack_size = static_cast<std::uint32_t>(g_idle_task_stack.size());
}

// The linker redirects every heap-capable symbol here. Project code installs
// the fixed micro-ROS arena before entity creation; any other allocation must
// fail instead of silently creating an unbounded runtime path.
extern "C" void* __wrap_malloc(std::size_t size) {
  static_cast<void>(size);
  return nullptr;
}

extern "C" void* __wrap_calloc(std::size_t count, std::size_t size) {
  static_cast<void>(count);
  static_cast<void>(size);
  return nullptr;
}

extern "C" void* __wrap_realloc(void* pointer, std::size_t size) {
  static_cast<void>(pointer);
  static_cast<void>(size);
  return nullptr;
}

extern "C" void __wrap_free(void* pointer) { static_cast<void>(pointer); }

extern "C" void* __wrap__malloc_r(void* reentrancy, std::size_t size) {
  static_cast<void>(reentrancy);
  static_cast<void>(size);
  return nullptr;
}

extern "C" void* __wrap__calloc_r(void* reentrancy, std::size_t count,
                                  std::size_t size) {
  static_cast<void>(reentrancy);
  static_cast<void>(count);
  static_cast<void>(size);
  return nullptr;
}

extern "C" void* __wrap__realloc_r(void* reentrancy, void* pointer,
                                   std::size_t size) {
  static_cast<void>(reentrancy);
  static_cast<void>(pointer);
  static_cast<void>(size);
  return nullptr;
}

extern "C" void __wrap__free_r(void* reentrancy, void* pointer) {
  static_cast<void>(reentrancy);
  static_cast<void>(pointer);
}

extern "C" void* __wrap_pvPortMalloc(std::size_t size) {
  static_cast<void>(size);
  return nullptr;
}

extern "C" void __wrap_vPortFree(void* pointer) { static_cast<void>(pointer); }

extern "C" void __cxa_pure_virtual() { FailStop(); }

int main() {
  if (platform::InitializePlatform() != platform::Status::kOk) {
    FailStop();
  }
  g_target_context.last_cycle_count = platform::CycleCounter();
  // Share the HAL millisecond epoch used by micro-ROS callback timestamps.
  // Sub-millisecond time advances from DWT after this aligned baseline.
  g_target_context.elapsed_cycles =
      static_cast<std::uint64_t>(platform::MonotonicMilliseconds()) *
      (platform::kSystemClockHz / 1000U);
  InitializeBatteryCalibration();

  const mentor_pi::mcu::MotorControlConfiguration motor_configuration =
      BuildMotorConfiguration();
  controller::ControllerRuntime& runtime =
      controller::ControllerInstance(motor_configuration);
  mentor_pi::mcu::drivers::AxisTransform imu_transform{};
  // Six-face gravity measurements on the RRCLite board show that PCB +X is
  // sensor +Y, PCB +Y is sensor -X, and PCB +Z is sensor +Z. Keep the same
  // signed permutation for acceleration and angular velocity.
  imu_transform.output = {{{1U, 1}, {0U, -1}, {2U, 1}}};
  imu_transform.verified = true;
  if (!runtime.Configure(BuildPlatformHooks(), imu_transform) ||
      !runtime.InitializeSafeBoot()) {
    FailStop();
  }
  if (!microros::ConfigureMicroRosRuntime(runtime.BuildMicroRosHooks())) {
    FailStop();
  }

  const controller::ControllerTaskEntries entries =
      runtime.BuildTaskEntries({&MentorPiMicroRosTaskMain, nullptr});
  if (platform::CreateStaticTasks(ConvertTaskEntries(entries)) !=
      platform::Status::kOk) {
    FailStop();
  }

  vTaskStartScheduler();
  FailStop();
}
