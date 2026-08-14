// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/peripherals.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"
#include "mentor_pi_mcu/platform/stm32/memory_regions.h"
#include "mentor_pi_mcu/platform/stm32/platform.h"
#include "mentor_pi_mcu/platform/stm32/task_entries.h"

namespace mentor_pi_mcu::platform::stm32 {
namespace {

constexpr std::uint16_t kPwmServoMinimumUs = 500U;
constexpr std::uint16_t kPwmServoMaximumUs = 2500U;
constexpr std::uint16_t kPwmServoResetUs = 1500U;
constexpr std::uint32_t kPwmServoFrameUs = 20000U;
constexpr std::size_t kMaximumImuI2cTransferBytes = 32U;
constexpr std::uint8_t kWs2812ZeroCode = 0xC0U;
constexpr std::uint8_t kWs2812OneCode = 0xF8U;
constexpr std::size_t kWs2812BytesPerPixel = 24U;
constexpr std::size_t kWs2812ResetBytes = 24U;

struct TimerChannel {
  TIM_HandleTypeDef* timer;
  std::uint32_t channel;
};

struct MotorHardware {
  TimerChannel positive;
  TimerChannel negative;
  TIM_HandleTypeDef* encoder;
};

const std::array<MotorHardware, kMotorCount> kMotors{{
    {{&g_tim1, TIM_CHANNEL_4}, {&g_tim1, TIM_CHANNEL_3}, &g_tim5},
    {{&g_tim1, TIM_CHANNEL_2}, {&g_tim1, TIM_CHANNEL_1}, &g_tim2},
    {{&g_tim9, TIM_CHANNEL_1}, {&g_tim9, TIM_CHANNEL_2}, &g_tim4},
    {{&g_tim11, TIM_CHANNEL_1}, {&g_tim10, TIM_CHANNEL_1}, &g_tim3},
}};

volatile std::uint8_t g_motor_armed_mask = 0U;
volatile std::uint8_t g_motor_authority_mask = 0U;

std::array<std::uint16_t, kPwmServoCount> g_servo_shadow{
    kPwmServoResetUs, kPwmServoResetUs, kPwmServoResetUs, kPwmServoResetUs};
std::array<std::uint16_t, kPwmServoCount> g_servo_active{
    kPwmServoResetUs, kPwmServoResetUs, kPwmServoResetUs, kPwmServoResetUs};
volatile std::uint32_t g_servo_elapsed_us = 0U;
volatile std::uint32_t g_servo_interval_us = 1U;
volatile std::uint8_t g_servo_remaining_mask = 0U;
volatile bool g_servo_running = false;
volatile bool g_servo_in_frame_gap = true;
volatile std::uint32_t g_servo_frame_sequence = 0U;

volatile bool g_buzzer_running = false;

MENTOR_PI_DMA_BUFFER std::uint8_t g_rgb_dma_buffer[kRgbDmaCapacityBytes];
volatile bool g_rgb_active = false;
volatile bool g_rgb_complete = true;
volatile bool g_rgb_error = false;

// ADC DMA is configured for two half-word transfers. Keeping both samples in
// one naturally aligned word satisfies the HAL's uint32_t-pointer API without
// weakening the DMA's half-word alignment.
MENTOR_PI_DMA_BUFFER std::uint32_t g_adc_dma_word = 0U;
volatile bool g_adc_active = false;
volatile bool g_adc_ready = false;
volatile bool g_adc_error = false;

std::uint8_t g_bus_tx_buffer[kBusUartBufferCapacityBytes];
std::uint8_t g_bus_rx_buffer[kBusUartBufferCapacityBytes];
volatile std::size_t g_bus_expected_rx_length = 0U;
volatile std::size_t g_bus_received_length = 0U;

enum class BusTransferState : std::uint8_t {
  kIdle,
  kTransmitting,
  kReceiving,
  kComplete,
  kError,
};

volatile BusTransferState g_bus_state = BusTransferState::kIdle;

Status FromHalStatus(HAL_StatusTypeDef status) {
  switch (status) {
    case HAL_OK:
      return Status::kOk;
    case HAL_BUSY:
      return Status::kBusy;
    case HAL_TIMEOUT:
      return Status::kTimeout;
    case HAL_ERROR:
    default:
      return Status::kIoError;
  }
}

std::uint32_t ChannelEnableBit(std::uint32_t channel) {
  return TIM_CCER_CC1E << channel;
}

void DisableTimerChannel(const TimerChannel& output) {
  if (output.timer->Instance == nullptr) {
    return;
  }
  __HAL_TIM_SET_COMPARE(output.timer, output.channel, 0U);
  output.timer->Instance->CCER &= ~ChannelEnableBit(output.channel);
}

void EnableTimerChannel(const TimerChannel& output) {
  TIM_TypeDef* const instance = output.timer->Instance;
  if (instance == nullptr) {
    return;
  }
  instance->EGR = TIM_EGR_UG;
  instance->CR1 |= TIM_CR1_CEN;
  if (instance == TIM1) {
    instance->BDTR |= TIM_BDTR_MOE;
  }
  instance->CCER |= ChannelEnableBit(output.channel);
}

void DriveServoPinsLow() {
  GPIOA->BSRR = static_cast<std::uint32_t>(GPIO_PIN_11 | GPIO_PIN_12) << 16U;
  GPIOC->BSRR = static_cast<std::uint32_t>(GPIO_PIN_8 | GPIO_PIN_9) << 16U;
}

void DriveServoPinsHigh() {
  GPIOA->BSRR = GPIO_PIN_11 | GPIO_PIN_12;
  GPIOC->BSRR = GPIO_PIN_8 | GPIO_PIN_9;
}

void DriveSelectedServoPinsLow(std::uint8_t mask) {
  std::uint32_t port_a_mask = 0U;
  std::uint32_t port_c_mask = 0U;
  if ((mask & 0x01U) != 0U) {
    port_a_mask |= GPIO_PIN_11;
  }
  if ((mask & 0x02U) != 0U) {
    port_a_mask |= GPIO_PIN_12;
  }
  if ((mask & 0x04U) != 0U) {
    port_c_mask |= GPIO_PIN_8;
  }
  if ((mask & 0x08U) != 0U) {
    port_c_mask |= GPIO_PIN_9;
  }
  GPIOA->BSRR = port_a_mask << 16U;
  GPIOC->BSRR = port_c_mask << 16U;
}

void ScheduleTimer13(std::uint32_t delay_us) {
  const std::uint32_t bounded_delay_us = std::max<std::uint32_t>(delay_us, 1U);
  g_servo_interval_us = bounded_delay_us;
  __HAL_TIM_SET_AUTORELOAD(&g_tim13, bounded_delay_us - 1U);
  __HAL_TIM_SET_COUNTER(&g_tim13, 0U);
}

void BeginServoFrameFromIsr() {
  for (std::size_t index = 0; index < g_servo_active.size(); ++index) {
    g_servo_active[index] = g_servo_shadow[index];
  }
  g_servo_elapsed_us = 0U;
  g_servo_remaining_mask = 0x0FU;
  g_servo_in_frame_gap = false;
  ++g_servo_frame_sequence;
  DriveServoPinsHigh();

  std::uint16_t first_edge_us = kPwmServoMaximumUs;
  for (std::uint16_t pulse_width_us : g_servo_active) {
    first_edge_us = std::min(first_edge_us, pulse_width_us);
  }
  ScheduleTimer13(first_edge_us);
}

void SetImuScl(bool high) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SetImuSda(bool high) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool ReadImuSda() {
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET;
}

void DelayMicroseconds(std::uint32_t delay_us) {
  const std::uint32_t start = CycleCounter();
  const std::uint32_t delay_cycles = delay_us * 168U;
  while (CycleCounter() - start < delay_cycles) {
    __NOP();
  }
}

bool I2cDeadlineExpired(std::uint32_t start_ms, std::uint32_t timeout_ms) {
  return MonotonicMilliseconds() - start_ms >= timeout_ms;
}

Status ClearImuI2cBus(std::uint32_t start_ms, std::uint32_t timeout_ms) {
  // UM10204 bus clear: a target that is holding SDA low must be given up to
  // nine SCL pulses so it can finish the interrupted byte. End with a STOP
  // condition before the caller retries START. This is bounded to less than
  // one normal transaction deadline and runs only after an observed stuck SDA.
  SetImuSda(true);
  for (std::size_t pulse = 0U; pulse < 9U && !ReadImuSda(); ++pulse) {
    SetImuScl(false);
    DelayMicroseconds(2U);
    SetImuScl(true);
    DelayMicroseconds(2U);
    if (I2cDeadlineExpired(start_ms, timeout_ms)) {
      return Status::kTimeout;
    }
  }
  if (!ReadImuSda()) {
    return Status::kBusy;
  }

  SetImuSda(false);
  DelayMicroseconds(2U);
  SetImuScl(true);
  DelayMicroseconds(2U);
  SetImuSda(true);
  DelayMicroseconds(2U);
  return I2cDeadlineExpired(start_ms, timeout_ms) ? Status::kTimeout
                                                  : Status::kOk;
}

Status ImuI2cStart(std::uint32_t start_ms, std::uint32_t timeout_ms) {
  SetImuSda(true);
  SetImuScl(true);
  DelayMicroseconds(2U);
  if (!ReadImuSda()) {
    const Status recovered = ClearImuI2cBus(start_ms, timeout_ms);
    if (recovered != Status::kOk) {
      return recovered;
    }
  }
  if (I2cDeadlineExpired(start_ms, timeout_ms)) {
    return Status::kTimeout;
  }
  SetImuSda(false);
  DelayMicroseconds(2U);
  SetImuScl(false);
  return Status::kOk;
}

void ImuI2cStop() {
  SetImuSda(false);
  DelayMicroseconds(2U);
  SetImuScl(true);
  DelayMicroseconds(2U);
  SetImuSda(true);
  DelayMicroseconds(2U);
}

Status ImuI2cWriteByte(std::uint8_t value, std::uint32_t start_ms,
                       std::uint32_t timeout_ms) {
  for (std::uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    SetImuSda((value & mask) != 0U);
    DelayMicroseconds(2U);
    SetImuScl(true);
    DelayMicroseconds(2U);
    SetImuScl(false);
    if (I2cDeadlineExpired(start_ms, timeout_ms)) {
      return Status::kTimeout;
    }
  }
  SetImuSda(true);
  DelayMicroseconds(2U);
  SetImuScl(true);
  DelayMicroseconds(2U);
  const bool acknowledged = !ReadImuSda();
  SetImuScl(false);
  return acknowledged ? Status::kOk : Status::kIoError;
}

Status ImuI2cReadByte(std::uint8_t* value, bool acknowledge,
                      std::uint32_t start_ms, std::uint32_t timeout_ms) {
  std::uint8_t received = 0U;
  SetImuSda(true);
  for (std::size_t bit = 0; bit < 8U; ++bit) {
    received = static_cast<std::uint8_t>(received << 1U);
    SetImuScl(true);
    DelayMicroseconds(2U);
    if (ReadImuSda()) {
      received |= 1U;
    }
    SetImuScl(false);
    DelayMicroseconds(2U);
    if (I2cDeadlineExpired(start_ms, timeout_ms)) {
      return Status::kTimeout;
    }
  }
  SetImuSda(!acknowledge);
  SetImuScl(true);
  DelayMicroseconds(2U);
  SetImuScl(false);
  SetImuSda(true);
  *value = received;
  return Status::kOk;
}

bool ValidI2cArguments(const void* data, std::size_t length,
                       std::uint32_t timeout_ms) {
  return data != nullptr && length > 0U &&
         length <= kMaximumImuI2cTransferBytes && timeout_ms >= 1U &&
         timeout_ms <= 10U;
}

}  // namespace

Status ArmMotorOutput(std::size_t motor_index) {
  if (motor_index >= kMotorCount) {
    return Status::kInvalidArgument;
  }
  if (kMotors[motor_index].positive.timer->Instance == nullptr ||
      kMotors[motor_index].negative.timer->Instance == nullptr) {
    return Status::kNotInitialized;
  }
  taskENTER_CRITICAL();
  g_motor_authority_mask |= static_cast<std::uint8_t>(1U << motor_index);
  taskEXIT_CRITICAL();
  return Status::kOk;
}

void DisarmMotorOutput(std::size_t motor_index) {
  if (motor_index >= kMotorCount) {
    return;
  }
  taskENTER_CRITICAL();
  const MotorHardware& motor = kMotors[motor_index];
  DisableTimerChannel(motor.positive);
  DisableTimerChannel(motor.negative);
  const std::uint8_t bit = static_cast<std::uint8_t>(1U << motor_index);
  g_motor_authority_mask &= static_cast<std::uint8_t>(~bit);
  g_motor_armed_mask &= static_cast<std::uint8_t>(~bit);
  taskEXIT_CRITICAL();
}

Status SetMotorDutyPermille(std::size_t motor_index,
                            std::int16_t duty_permille) {
  if (motor_index >= kMotors.size() || duty_permille < -1000 ||
      duty_permille > 1000) {
    return Status::kInvalidArgument;
  }
  const MotorHardware& motor = kMotors[motor_index];
  if (motor.positive.timer->Instance == nullptr ||
      motor.negative.timer->Instance == nullptr) {
    return Status::kNotInitialized;
  }

  // Break-before-make is mandatory for the dual-input motor stage.
  // The short critical section also prevents SafetySupervisorTask from
  // stopping a channel between break-before-make and the final enable.
  taskENTER_CRITICAL();
  DisableTimerChannel(motor.positive);
  DisableTimerChannel(motor.negative);
  g_motor_armed_mask &= static_cast<std::uint8_t>(~(1U << motor_index));
  if (duty_permille == 0) {
    taskEXIT_CRITICAL();
    return Status::kOk;
  }
  const std::uint8_t bit = static_cast<std::uint8_t>(1U << motor_index);
  if ((g_motor_authority_mask & bit) == 0U) {
    taskEXIT_CRITICAL();
    return Status::kBusy;
  }

  const std::int32_t signed_duty = duty_permille;
  const std::uint32_t magnitude =
      static_cast<std::uint32_t>(signed_duty < 0 ? -signed_duty : signed_duty);
  const TimerChannel& active_output =
      signed_duty > 0 ? motor.positive : motor.negative;
  const std::uint32_t compare =
      (magnitude * active_output.timer->Init.Period) / 1000U;
  __HAL_TIM_SET_COMPARE(active_output.timer, active_output.channel, compare);
  EnableTimerChannel(active_output);
  g_motor_armed_mask |= bit;
  taskEXIT_CRITICAL();
  return Status::kOk;
}

std::array<std::uint32_t, kMotorCount> ReadEncoderCounters() {
  std::array<std::uint32_t, kMotorCount> counters{};
  for (std::size_t index = 0; index < kMotors.size(); ++index) {
    TIM_HandleTypeDef* const encoder = kMotors[index].encoder;
    if (encoder->Instance != nullptr) {
      counters[index] = __HAL_TIM_GET_COUNTER(encoder);
    }
  }
  return counters;
}

void EmergencyStopMotors() {
  for (const MotorHardware& motor : kMotors) {
    DisableTimerChannel(motor.positive);
    DisableTimerChannel(motor.negative);
  }
  if (g_tim1.Instance != nullptr) {
    g_tim1.Instance->BDTR &= ~TIM_BDTR_MOE;
  }
  g_motor_authority_mask = 0U;
  g_motor_armed_mask = 0U;
}

std::uint8_t MotorArmedMask() { return g_motor_armed_mask; }

Status SetPwmServoPulseShadow(
    const std::array<std::uint16_t, kPwmServoCount>& pulse_width_us) {
  for (std::uint16_t value : pulse_width_us) {
    if (value < kPwmServoMinimumUs || value > kPwmServoMaximumUs) {
      return Status::kInvalidArgument;
    }
  }
  // This is also called during safe boot, before the scheduler initializes
  // FreeRTOS's critical-nesting state. Preserve PRIMASK directly so the same
  // constant-time shadow copy is valid before and after scheduler start.
  const std::uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_servo_shadow = pulse_width_us;
  if (primask == 0U) {
    __enable_irq();
  }
  return Status::kOk;
}

Status StartPwmServoFrameGenerator() {
  if (g_tim13.Instance == nullptr) {
    return Status::kNotInitialized;
  }
  if (g_servo_running) {
    return Status::kBusy;
  }
  DriveServoPinsLow();
  g_servo_running = true;
  g_servo_in_frame_gap = true;
  ScheduleTimer13(1U);
  __HAL_TIM_CLEAR_FLAG(&g_tim13, TIM_FLAG_UPDATE);
  const HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&g_tim13);
  if (status != HAL_OK) {
    g_servo_running = false;
    g_servo_remaining_mask = 0U;
    DriveServoPinsLow();
  }
  return FromHalStatus(status);
}

void StopPwmServoFrameGenerator() {
  if (g_tim13.Instance != nullptr) {
    static_cast<void>(HAL_TIM_Base_Stop_IT(&g_tim13));
  }
  g_servo_running = false;
  g_servo_remaining_mask = 0U;
  DriveServoPinsLow();
}

std::uint32_t PwmServoFrameSequence() { return g_servo_frame_sequence; }

Status SetLed(std::size_t led_index, bool on) {
  if (led_index >= kLedCount) {
    return Status::kInvalidArgument;
  }
  constexpr std::array<std::uint16_t, kLedCount> kPins{GPIO_PIN_9, GPIO_PIN_10,
                                                       GPIO_PIN_11};
  const bool active_low = led_index < 2U;
  const GPIO_PinState state =
      (on != active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(GPIOD, kPins[led_index], state);
  return Status::kOk;
}

bool ReadButtonPressed(std::size_t button_index) {
  if (button_index >= kButtonCount) {
    return false;
  }
  constexpr std::array<std::uint16_t, kButtonCount> kPins{GPIO_PIN_1,
                                                          GPIO_PIN_0};
  return HAL_GPIO_ReadPin(GPIOE, kPins[button_index]) == GPIO_PIN_RESET;
}

Status SetBuzzerFrequency(std::uint16_t frequency_hz) {
  if (frequency_hz != 0U && (frequency_hz < 10U || frequency_hz > 20000U)) {
    return Status::kInvalidArgument;
  }
  if (g_tim12.Instance == nullptr) {
    return Status::kNotInitialized;
  }
  static_cast<void>(HAL_TIM_Base_Stop_IT(&g_tim12));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  g_buzzer_running = false;
  if (frequency_hz == 0U) {
    return Status::kOk;
  }

  const std::uint32_t half_period_us = 500000U / frequency_hz;
  __HAL_TIM_SET_AUTORELOAD(&g_tim12, half_period_us - 1U);
  __HAL_TIM_SET_COUNTER(&g_tim12, 0U);
  __HAL_TIM_CLEAR_FLAG(&g_tim12, TIM_FLAG_UPDATE);
  g_buzzer_running = true;
  const HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&g_tim12);
  if (status != HAL_OK) {
    g_buzzer_running = false;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  }
  return FromHalStatus(status);
}

Status StartRgbTransfer(const std::uint8_t* data, std::size_t length) {
  if (data == nullptr || length == 0U || length > sizeof(g_rgb_dma_buffer)) {
    return Status::kInvalidArgument;
  }
  if (g_rgb_active) {
    return Status::kBusy;
  }
  std::memcpy(g_rgb_dma_buffer, data, length);
  g_rgb_complete = false;
  g_rgb_error = false;
  g_rgb_active = true;
  const HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(
      &g_spi1, g_rgb_dma_buffer, static_cast<std::uint16_t>(length));
  if (status != HAL_OK) {
    g_rgb_active = false;
    g_rgb_error = true;
  }
  return FromHalStatus(status);
}

Status StartRgbPixels(const std::array<std::uint8_t, 2>& red,
                      const std::array<std::uint8_t, 2>& green,
                      const std::array<std::uint8_t, 2>& blue) {
  constexpr std::size_t kFrameSize =
      2U * kWs2812BytesPerPixel + kWs2812ResetBytes;
  std::array<std::uint8_t, kFrameSize> encoded{};
  for (std::size_t pixel = 0; pixel < 2U; ++pixel) {
    const std::array<std::uint8_t, 3> grb{green[pixel], red[pixel],
                                          blue[pixel]};
    for (std::size_t component = 0; component < grb.size(); ++component) {
      for (std::size_t bit = 0; bit < 8U; ++bit) {
        const std::uint8_t mask = static_cast<std::uint8_t>(0x80U >> bit);
        encoded[pixel * kWs2812BytesPerPixel + component * 8U + bit] =
            (grb[component] & mask) != 0U ? kWs2812OneCode : kWs2812ZeroCode;
      }
    }
  }
  // The zero-initialized tail holds MOSI low long enough to latch both pixels.
  return StartRgbTransfer(encoded.data(), encoded.size());
}

bool IsRgbTransferComplete() { return g_rgb_complete && !g_rgb_error; }

Status RgbTransferStatus() {
  if (g_rgb_active) {
    return Status::kBusy;
  }
  return g_rgb_error ? Status::kIoError : Status::kOk;
}

void CancelRgbTransfer() {
  if (g_spi1.Instance != nullptr) {
    static_cast<void>(HAL_SPI_Abort(&g_spi1));
  }
  g_rgb_active = false;
  g_rgb_complete = false;
  g_rgb_error = true;
}

Status StartBatteryAdcConversion() {
  if (g_adc_active) {
    return Status::kBusy;
  }
  g_adc_ready = false;
  g_adc_error = false;
  g_adc_active = true;
  const HAL_StatusTypeDef status =
      HAL_ADC_Start_DMA(&g_adc1, &g_adc_dma_word, 2U);
  if (status != HAL_OK) {
    g_adc_active = false;
    g_adc_error = true;
  }
  return FromHalStatus(status);
}

Status TakeBatteryAdcResult(std::array<std::uint16_t, 2>* samples) {
  if (samples == nullptr) {
    return Status::kInvalidArgument;
  }
  if (g_adc_error) {
    return Status::kIoError;
  }
  if (g_adc_active || !g_adc_ready) {
    return Status::kBusy;
  }
  const std::uint32_t dma_word = g_adc_dma_word;
  (*samples)[0] = static_cast<std::uint16_t>(dma_word & 0xFFFFU);
  (*samples)[1] = static_cast<std::uint16_t>(dma_word >> 16U);
  g_adc_ready = false;
  return Status::kOk;
}

std::uint16_t FactoryVrefintCalibration() { return *VREFINT_CAL_ADDR; }

Status OledI2cTransmit(std::uint16_t device_address, const std::uint8_t* data,
                       std::size_t length, std::uint32_t timeout_ms) {
  if (data == nullptr || length == 0U ||
      length > std::numeric_limits<std::uint16_t>::max() || timeout_ms < 1U ||
      timeout_ms > 10U) {
    return Status::kInvalidArgument;
  }
  return FromHalStatus(HAL_I2C_Master_Transmit(
      &g_i2c1, device_address, const_cast<std::uint8_t*>(data),
      static_cast<std::uint16_t>(length), timeout_ms));
}

Status ResetOledI2c() {
  Status status = FromHalStatus(HAL_I2C_DeInit(&g_i2c1));
  if (status == Status::kOk) {
    status = FromHalStatus(HAL_I2C_Init(&g_i2c1));
  }
  return status;
}

Status ImuI2cWrite(std::uint8_t device_address, std::uint8_t register_address,
                   const std::uint8_t* data, std::size_t length,
                   std::uint32_t timeout_ms) {
  if (device_address > 0x7FU || !ValidI2cArguments(data, length, timeout_ms)) {
    return Status::kInvalidArgument;
  }
  const std::uint32_t start_ms = MonotonicMilliseconds();
  Status status = ImuI2cStart(start_ms, timeout_ms);
  if (status == Status::kOk) {
    status = ImuI2cWriteByte(static_cast<std::uint8_t>(device_address << 1U),
                             start_ms, timeout_ms);
  }
  if (status == Status::kOk) {
    status = ImuI2cWriteByte(register_address, start_ms, timeout_ms);
  }
  for (std::size_t index = 0; index < length && status == Status::kOk;
       ++index) {
    status = ImuI2cWriteByte(data[index], start_ms, timeout_ms);
  }
  ImuI2cStop();
  return status;
}

Status ImuI2cRead(std::uint8_t device_address, std::uint8_t register_address,
                  std::uint8_t* data, std::size_t length,
                  std::uint32_t timeout_ms) {
  if (device_address > 0x7FU || !ValidI2cArguments(data, length, timeout_ms)) {
    return Status::kInvalidArgument;
  }
  const std::uint32_t start_ms = MonotonicMilliseconds();
  Status status = ImuI2cStart(start_ms, timeout_ms);
  if (status == Status::kOk) {
    status = ImuI2cWriteByte(static_cast<std::uint8_t>(device_address << 1U),
                             start_ms, timeout_ms);
  }
  if (status == Status::kOk) {
    status = ImuI2cWriteByte(register_address, start_ms, timeout_ms);
  }
  if (status == Status::kOk) {
    status = ImuI2cStart(start_ms, timeout_ms);
  }
  if (status == Status::kOk) {
    status =
        ImuI2cWriteByte(static_cast<std::uint8_t>((device_address << 1U) | 1U),
                        start_ms, timeout_ms);
  }
  for (std::size_t index = 0; index < length && status == Status::kOk;
       ++index) {
    status =
        ImuI2cReadByte(&data[index], index + 1U < length, start_ms, timeout_ms);
  }
  ImuI2cStop();
  return status;
}

Status StartBusUartExchange(const std::uint8_t* data, std::size_t length,
                            std::size_t response_length) {
  if (data == nullptr || length == 0U || length > sizeof(g_bus_tx_buffer) ||
      response_length > sizeof(g_bus_rx_buffer)) {
    return Status::kInvalidArgument;
  }
  if (g_bus_state != BusTransferState::kIdle &&
      g_bus_state != BusTransferState::kComplete) {
    return Status::kBusy;
  }
  std::memcpy(g_bus_tx_buffer, data, length);
  g_bus_expected_rx_length = response_length;
  g_bus_received_length = 0U;
  g_bus_state = BusTransferState::kTransmitting;
  const HAL_StatusTypeDef direction_status =
      HAL_HalfDuplex_EnableTransmitter(&g_uart5);
  if (direction_status != HAL_OK) {
    g_bus_state = BusTransferState::kError;
    return FromHalStatus(direction_status);
  }
  const HAL_StatusTypeDef status = HAL_UART_Transmit_IT(
      &g_uart5, g_bus_tx_buffer, static_cast<std::uint16_t>(length));
  if (status != HAL_OK) {
    g_bus_state = BusTransferState::kError;
  }
  return FromHalStatus(status);
}

Status TakeBusUartCompletion(std::uint8_t* destination,
                             std::size_t destination_capacity,
                             std::size_t* received_length) {
  if (received_length == nullptr) {
    return Status::kInvalidArgument;
  }
  if (g_bus_state == BusTransferState::kError) {
    g_bus_state = BusTransferState::kIdle;
    return Status::kIoError;
  }
  if (g_bus_state != BusTransferState::kComplete) {
    return Status::kBusy;
  }
  if (g_bus_received_length > destination_capacity ||
      (g_bus_received_length != 0U && destination == nullptr)) {
    return Status::kInvalidArgument;
  }
  if (g_bus_received_length != 0U) {
    std::memcpy(destination, g_bus_rx_buffer, g_bus_received_length);
  }
  *received_length = g_bus_received_length;
  g_bus_received_length = 0U;
  g_bus_state = BusTransferState::kIdle;
  return Status::kOk;
}

void ResetBusUart() {
  static_cast<void>(HAL_UART_Abort(&g_uart5));
  g_bus_expected_rx_length = 0U;
  g_bus_received_length = 0U;
  g_bus_state = BusTransferState::kIdle;
}

void HandleMotorReleaseFromIsr() { NotifyTaskFromIsr(TaskId::kMotorControl); }

void HandlePwmServoTimerFromIsr() {
  if (!g_servo_running) {
    DriveServoPinsLow();
    return;
  }
  if (g_servo_in_frame_gap) {
    BeginServoFrameFromIsr();
    return;
  }

  g_servo_elapsed_us += g_servo_interval_us;
  std::uint8_t falling_mask = 0U;
  std::uint16_t next_edge_us = kPwmServoMaximumUs;
  bool next_edge_found = false;
  for (std::size_t index = 0; index < g_servo_active.size(); ++index) {
    const std::uint8_t bit = static_cast<std::uint8_t>(1U << index);
    if ((g_servo_remaining_mask & bit) == 0U) {
      continue;
    }
    if (g_servo_active[index] <= g_servo_elapsed_us) {
      falling_mask |= bit;
      g_servo_remaining_mask &= static_cast<std::uint8_t>(~bit);
    } else if (!next_edge_found || g_servo_active[index] < next_edge_us) {
      next_edge_us = g_servo_active[index];
      next_edge_found = true;
    }
  }
  DriveSelectedServoPinsLow(falling_mask);
  if (g_servo_remaining_mask == 0U) {
    g_servo_in_frame_gap = true;
    ScheduleTimer13(kPwmServoFrameUs - g_servo_elapsed_us);
  } else {
    ScheduleTimer13(static_cast<std::uint32_t>(next_edge_us) -
                    g_servo_elapsed_us);
  }
}

void HandleBuzzerTimerFromIsr() {
  if (!g_buzzer_running) {
    GPIOA->BSRR = static_cast<std::uint32_t>(GPIO_PIN_6) << 16U;
    return;
  }
  if ((GPIOA->ODR & GPIO_PIN_6) == 0U) {
    GPIOA->BSRR = GPIO_PIN_6;
  } else {
    GPIOA->BSRR = static_cast<std::uint32_t>(GPIO_PIN_6) << 16U;
  }
}

void HandleImuDataReadyFromIsr() { NotifyTaskFromIsr(TaskId::kSensor); }

void HandleRgbCompleteFromIsr() {
  g_rgb_active = false;
  g_rgb_complete = true;
  NotifyTaskFromIsr(TaskId::kPeripheral);
}

void HandleRgbErrorFromIsr() {
  g_rgb_active = false;
  g_rgb_complete = false;
  g_rgb_error = true;
  NotifyTaskFromIsr(TaskId::kPeripheral);
}

void HandleAdcCompleteFromIsr() {
  g_adc_active = false;
  g_adc_ready = true;
  NotifyTaskFromIsr(TaskId::kSensor);
}

void HandleAdcErrorFromIsr() {
  g_adc_active = false;
  g_adc_ready = false;
  g_adc_error = true;
  NotifyTaskFromIsr(TaskId::kSensor);
}

void HandleBusUartTxCompleteFromIsr() {
  g_bus_received_length = 0U;
  if (g_bus_expected_rx_length != 0U) {
    g_bus_state = BusTransferState::kReceiving;
    HAL_StatusTypeDef status = HAL_HalfDuplex_EnableReceiver(&g_uart5);
    if (status == HAL_OK) {
      status = HAL_UART_Receive_IT(
          &g_uart5, g_bus_rx_buffer,
          static_cast<std::uint16_t>(g_bus_expected_rx_length));
    }
    if (status != HAL_OK) {
      g_bus_state = BusTransferState::kError;
      NotifyTaskFromIsr(TaskId::kBusServo);
    }
    return;
  }
  g_bus_state = BusTransferState::kComplete;
  NotifyTaskFromIsr(TaskId::kBusServo);
}

void HandleBusUartRxCompleteFromIsr() {
  g_bus_received_length = g_bus_expected_rx_length;
  g_bus_state = BusTransferState::kComplete;
  NotifyTaskFromIsr(TaskId::kBusServo);
}

void HandleBusUartErrorFromIsr() {
  g_bus_state = BusTransferState::kError;
  NotifyTaskFromIsr(TaskId::kBusServo);
}

}  // namespace mentor_pi_mcu::platform::stm32
