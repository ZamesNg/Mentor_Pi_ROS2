// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/platform.h"

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"
#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"
#include "mentor_pi_mcu/platform/stm32/watchdog_retention.h"

extern "C" {
__attribute__((
    section(".noinit.watchdog_retention"), used,
    aligned(
        4))) volatile mentor_pi_mcu::platform::stm32::WatchdogRetentionRecord
    g_watchdog_retention_record;
}

namespace mentor_pi_mcu::platform::stm32 {
namespace {

static_assert(HSE_VALUE == 16000000U,
              "RRCLite V1.0 is populated with a 16 MHz HSE crystal");

constexpr std::uint32_t kMotorPwmPrescaler = 839U;
constexpr std::uint32_t kMotorPwmPeriod = 999U;
constexpr std::uint32_t kOneMegahertzPrescaler = 83U;
constexpr std::uint32_t kMotorReleasePeriod = 999U;
constexpr std::uint32_t kInitialSchedulerPeriod = 19999U;

bool g_platform_initialized = false;
bool g_watchdog_initialized = false;
bool g_msp_error = false;
std::uint8_t g_reset_reason = 255U;
std::uint8_t g_captured_watchdog_task = kNoWatchdogTask;
bool g_watchdog_task_persisted = false;

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

void ClearWatchdogRetentionRecord() {
  // Invalidate first so a reset during clearing can never expose a payload.
  g_watchdog_retention_record.magic = 0U;
  __DSB();
  g_watchdog_retention_record.payload = 0U;
  g_watchdog_retention_record.payload_complement = 0U;
  __DMB();
}

void CaptureAndClearResetReason() {
  // Priority follows the most specific reset sources. RCC flags are sticky and
  // are captured once before __HAL_RCC_CLEAR_RESET_FLAGS().
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
    g_reset_reason = 3U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
    g_reset_reason = 4U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
    g_reset_reason = 2U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
    g_reset_reason = 6U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
    g_reset_reason = 0U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
    // BORRSTF is also asserted after a power-on reset, so POR must win above.
    g_reset_reason = 5U;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
    g_reset_reason = 1U;
  } else {
    g_reset_reason = 255U;
  }

  const WatchdogRetentionRecord retained{
      g_watchdog_retention_record.magic, g_watchdog_retention_record.payload,
      g_watchdog_retention_record.payload_complement};
  g_captured_watchdog_task = WatchdogTaskForReset(retained, g_reset_reason);
  // Consume the retained evidence exactly once, even when malformed or when
  // RCC says the reset was not caused by IWDG.
  ClearWatchdogRetentionRecord();
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void EnableAllGpioClocks() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

void ConfigureSafeGpio() {
  EnableAllGpioClocks();

  // Apply output latches before changing modes. All bridge inputs, PWM-servo
  // signals, buzzer, and RGB lines are low. LED1/2 are active-low and therefore
  // use high as their off latch; LED3 is active-high and uses low.
  HAL_GPIO_WritePin(GPIOE,
                    GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11 |
                        GPIO_PIN_13 | GPIO_PIN_14,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_11 | GPIO_PIN_12,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 |
             GPIO_PIN_14;
  HAL_GPIO_Init(GPIOE, &gpio);
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOD, &gpio);

  // The old PA8 buzzer output is explicitly inactive in v2.
  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &gpio);

  // Software-I2C idles released high. Open-drain mode prevents contention with
  // the QMI8658 and any board pull-ups.
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &gpio);

  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &gpio);
}

Status ConfigureSystemClock() {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitTypeDef oscillator{};
  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM = 8U;
  oscillator.PLL.PLLN = 168U;
  oscillator.PLL.PLLP = RCC_PLLP_DIV2;
  oscillator.PLL.PLLQ = 7U;
  Status status = FromHalStatus(HAL_RCC_OscConfig(&oscillator));
  if (status != Status::kOk) {
    return status;
  }

  RCC_ClkInitTypeDef clocks{};
  clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV4;
  clocks.APB2CLKDivider = RCC_HCLK_DIV2;
  status = FromHalStatus(HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_5));
  if (status == Status::kOk) {
    HAL_RCC_EnableCSS();
    SystemCoreClockUpdate();
    if (SystemCoreClock != kSystemClockHz ||
        HAL_RCC_GetSysClockFreq() != kSystemClockHz ||
        HAL_RCC_GetHCLKFreq() != kSystemClockHz ||
        HAL_RCC_GetPCLK1Freq() != kApb1ClockHz ||
        HAL_RCC_GetPCLK2Freq() != kApb2ClockHz) {
      status = Status::kIoError;
    }
  }
  return status;
}

Status ConfigureMotorPwmTimer(TIM_HandleTypeDef* timer, TIM_TypeDef* instance,
                              std::size_t channel_count) {
  timer->Instance = instance;
  timer->Init.Prescaler = kMotorPwmPrescaler;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = kMotorPwmPeriod;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (instance == TIM1) {
    timer->Init.RepetitionCounter = 0U;
  }
  Status status = FromHalStatus(HAL_TIM_PWM_Init(timer));
  if (status != Status::kOk) {
    return status;
  }

  TIM_OC_InitTypeDef channel{};
  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0U;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;
  channel.OCIdleState = TIM_OCIDLESTATE_RESET;
  channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  constexpr std::array<std::uint32_t, 4> kChannels{
      TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
  for (std::size_t index = 0; index < channel_count; ++index) {
    status = FromHalStatus(
        HAL_TIM_PWM_ConfigChannel(timer, &channel, kChannels[index]));
    if (status != Status::kOk) {
      return status;
    }
  }

  if (instance == TIM1) {
    TIM_BreakDeadTimeConfigTypeDef break_config{};
    break_config.OffStateRunMode = TIM_OSSR_DISABLE;
    break_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
    break_config.LockLevel = TIM_LOCKLEVEL_OFF;
    break_config.DeadTime = 0U;
    break_config.BreakState = TIM_BREAK_DISABLE;
    break_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    break_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    status = FromHalStatus(HAL_TIMEx_ConfigBreakDeadTime(timer, &break_config));
  }
  return status;
}

Status ConfigureEncoder(TIM_HandleTypeDef* timer, TIM_TypeDef* instance) {
  timer->Instance = instance;
  timer->Init.Prescaler = 0U;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period =
      (instance == TIM2 || instance == TIM5) ? UINT32_MAX : 65535U;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  TIM_Encoder_InitTypeDef encoder{};
  encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 0U;
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 0U;
  return FromHalStatus(HAL_TIM_Encoder_Init(timer, &encoder));
}

Status ConfigureBaseTimer(TIM_HandleTypeDef* timer, TIM_TypeDef* instance,
                          std::uint32_t period) {
  timer->Instance = instance;
  timer->Init.Prescaler = kOneMegahertzPrescaler;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = period;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  return FromHalStatus(HAL_TIM_Base_Init(timer));
}

void ConfigureMotorAlternatePins() {
  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &gpio);
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  gpio.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOE, &gpio);
  gpio.Pin = GPIO_PIN_8;
  gpio.Alternate = GPIO_AF3_TIM10;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_9;
  gpio.Alternate = GPIO_AF3_TIM11;
  HAL_GPIO_Init(GPIOB, &gpio);
}

Status ConfigureTimers() {
  Status status = ConfigureMotorPwmTimer(&g_tim1, TIM1, 4U);
  if (status == Status::kOk) {
    status = ConfigureMotorPwmTimer(&g_tim9, TIM9, 2U);
  }
  if (status == Status::kOk) {
    status = ConfigureMotorPwmTimer(&g_tim10, TIM10, 1U);
  }
  if (status == Status::kOk) {
    status = ConfigureMotorPwmTimer(&g_tim11, TIM11, 1U);
  }
  if (status == Status::kOk) {
    ConfigureMotorAlternatePins();
    EmergencyStopMotors();
  }
  if (status == Status::kOk) {
    status = ConfigureEncoder(&g_tim2, TIM2);
  }
  if (status == Status::kOk) {
    status = ConfigureEncoder(&g_tim3, TIM3);
  }
  if (status == Status::kOk) {
    status = ConfigureEncoder(&g_tim4, TIM4);
  }
  if (status == Status::kOk) {
    status = ConfigureEncoder(&g_tim5, TIM5);
  }
  if (status == Status::kOk) {
    status = ConfigureBaseTimer(&g_tim7, TIM7, kMotorReleasePeriod);
  }
  if (status == Status::kOk) {
    status = ConfigureBaseTimer(&g_tim12, TIM12, kInitialSchedulerPeriod);
  }
  if (status == Status::kOk) {
    status = ConfigureBaseTimer(&g_tim13, TIM13, kInitialSchedulerPeriod);
  }
  return status;
}

Status ConfigureUsart1() {
  g_usart1.Instance = USART1;
  g_usart1.Init.BaudRate = kUsart1Baud;
  g_usart1.Init.WordLength = UART_WORDLENGTH_8B;
  g_usart1.Init.StopBits = UART_STOPBITS_1;
  g_usart1.Init.Parity = UART_PARITY_NONE;
  g_usart1.Init.Mode = UART_MODE_TX_RX;
  g_usart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  g_usart1.Init.OverSampling = UART_OVERSAMPLING_16;
  Status status = FromHalStatus(HAL_UART_Init(&g_usart1));
  return status;
}

Status ConfigureUsarts() {
  Status status = ConfigureUsart1();
  if (status != Status::kOk) {
    return status;
  }

  g_uart5.Instance = UART5;
  g_uart5.Init.BaudRate = 115200U;
  g_uart5.Init.WordLength = UART_WORDLENGTH_8B;
  g_uart5.Init.StopBits = UART_STOPBITS_1;
  g_uart5.Init.Parity = UART_PARITY_NONE;
  g_uart5.Init.Mode = UART_MODE_TX_RX;
  g_uart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  g_uart5.Init.OverSampling = UART_OVERSAMPLING_16;
  return FromHalStatus(HAL_HalfDuplex_Init(&g_uart5));
}

Status ConfigureSpi() {
  g_spi1.Instance = SPI1;
  g_spi1.Init.Mode = SPI_MODE_MASTER;
  g_spi1.Init.Direction = SPI_DIRECTION_2LINES;
  g_spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  g_spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  g_spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  g_spi1.Init.NSS = SPI_NSS_SOFT;
  g_spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  g_spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  g_spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  g_spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  g_spi1.Init.CRCPolynomial = 10U;
  return FromHalStatus(HAL_SPI_Init(&g_spi1));
}

Status ConfigureI2c() {
  g_i2c1.Instance = I2C1;
  g_i2c1.Init.ClockSpeed = 100000U;
  g_i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  g_i2c1.Init.OwnAddress1 = 0U;
  g_i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  g_i2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  g_i2c1.Init.OwnAddress2 = 0U;
  g_i2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  g_i2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  return FromHalStatus(HAL_I2C_Init(&g_i2c1));
}

Status ConfigureAdc() {
  g_adc1.Instance = ADC1;
  g_adc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  g_adc1.Init.Resolution = ADC_RESOLUTION_12B;
  g_adc1.Init.ScanConvMode = ENABLE;
  g_adc1.Init.ContinuousConvMode = DISABLE;
  g_adc1.Init.DiscontinuousConvMode = DISABLE;
  g_adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  g_adc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  g_adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  g_adc1.Init.NbrOfConversion = 2U;
  g_adc1.Init.DMAContinuousRequests = DISABLE;
  g_adc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  Status status = FromHalStatus(HAL_ADC_Init(&g_adc1));
  if (status != Status::kOk) {
    return status;
  }

  ADC_ChannelConfTypeDef channel{};
  channel.Channel = ADC_CHANNEL_VREFINT;
  channel.Rank = 1U;
  channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  status = FromHalStatus(HAL_ADC_ConfigChannel(&g_adc1, &channel));
  if (status != Status::kOk) {
    return status;
  }
  channel.Channel = ADC_CHANNEL_8;
  channel.Rank = 2U;
  return FromHalStatus(HAL_ADC_ConfigChannel(&g_adc1, &channel));
}

Status ConfigureWatchdog() {
  g_iwdg.Instance = IWDG;
  g_iwdg.Init.Prescaler = IWDG_PRESCALER_64;
  g_iwdg.Init.Reload = 249U;
  const Status status = FromHalStatus(HAL_IWDG_Init(&g_iwdg));
  g_watchdog_initialized = status == Status::kOk;
  return status;
}

}  // namespace

Status InitializePlatform() {
  if (g_platform_initialized) {
    return Status::kBusy;
  }

  HAL_Init();
  CaptureAndClearResetReason();
  ConfigureSafeGpio();
  EmergencyStopMotors();

  Status status = ConfigureSystemClock();
  if (status == Status::kOk) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __HAL_RCC_DMA2_CLK_ENABLE();
    status = ConfigureTimers();
  }
  if (status == Status::kOk) {
    status = ConfigureUsarts();
  }
  if (status == Status::kOk) {
    status = ConfigureSpi();
  }
  if (status == Status::kOk) {
    status = ConfigureI2c();
  }
  if (status == Status::kOk) {
    status = ConfigureAdc();
  }
  if (status == Status::kOk && g_msp_error) {
    status = Status::kIoError;
  }
  if (status == Status::kOk) {
    status = ConfigureWatchdog();
  }

  if (status != Status::kOk) {
    EmergencyStopMotors();
    return status;
  }
  g_platform_initialized = true;
  return Status::kOk;
}

Status StartControlTiming() {
  if (!g_platform_initialized) {
    return Status::kNotInitialized;
  }
  constexpr std::array<TIM_HandleTypeDef*, kMotorCount> kEncoderTimers{
      &g_tim5, &g_tim2, &g_tim4, &g_tim3};
  for (TIM_HandleTypeDef* timer : kEncoderTimers) {
    const Status status =
        FromHalStatus(HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL));
    if (status != Status::kOk) {
      EmergencyStopMotors();
      return status;
    }
  }
  return FromHalStatus(HAL_TIM_Base_Start_IT(&g_tim7));
}

std::uint32_t MonotonicMilliseconds() { return HAL_GetTick(); }

std::uint32_t CycleCounter() { return DWT->CYCCNT; }

Status RefreshWatchdogFromSafetySupervisor() {
  if (!g_watchdog_initialized) {
    return Status::kNotInitialized;
  }
  return FromHalStatus(HAL_IWDG_Refresh(&g_iwdg));
}

bool WatchdogIsRunning() { return g_watchdog_initialized; }

void PersistWatchdogTask(std::uint8_t task) {
  if (g_watchdog_task_persisted || task >= kWatchdogTaskCount) {
    return;
  }
  g_watchdog_task_persisted = true;
  const std::uint32_t payload = EncodeWatchdogTaskPayload(task);

  // A valid magic is published last. Any reset before the final store leaves
  // either an invalid magic or a complement mismatch and is rejected at boot.
  g_watchdog_retention_record.magic = 0U;
  __DSB();
  g_watchdog_retention_record.payload = payload;
  g_watchdog_retention_record.payload_complement = ~payload;
  __DMB();
  g_watchdog_retention_record.magic = kWatchdogRetentionMagic;
  __DSB();
}

std::uint8_t CapturedResetReason() { return g_reset_reason; }

std::uint8_t CapturedWatchdogTask() { return g_captured_watchdog_task; }

// MSP callbacks set this sticky bit because their HAL signatures cannot return
// an error to the caller.
void RecordMspError() { g_msp_error = true; }

}  // namespace mentor_pi_mcu::platform::stm32
