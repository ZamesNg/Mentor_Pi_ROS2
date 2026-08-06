// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_TARGET_STM32_INCLUDE_STM32F4XX_HAL_CONF_H_
#define MENTOR_PI_MCU_TARGET_STM32_INCLUDE_STM32F4XX_HAL_CONF_H_

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#ifndef HSE_VALUE
#define HSE_VALUE 16000000U
#endif
#ifndef HSE_STARTUP_TIMEOUT
#define HSE_STARTUP_TIMEOUT 100U
#endif
#ifndef HSI_VALUE
#define HSI_VALUE 16000000U
#endif
#ifndef LSI_VALUE
#define LSI_VALUE 32000U
#endif
#ifndef LSE_VALUE
#define LSE_VALUE 32768U
#endif
#ifndef EXTERNAL_CLOCK_VALUE
#define EXTERNAL_CLOCK_VALUE 12288000U
#endif

#define VDD_VALUE 3300U
#define TICK_INT_PRIORITY 15U
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE 1U

#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U
#define USE_HAL_UART_REGISTER_CALLBACKS 0U

// STM32 HAL headers are order-dependent: ADC embeds DMA_HandleTypeDef. Keep
// DMA first even when repository formatting sorts ordinary include blocks.
// clang-format off
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_iwdg.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
// clang-format on

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#define assert_param(expression) \
  ((expression) ? (void)0U : assert_failed((uint8_t*)__FILE__, __LINE__))
#else
#define assert_param(expression) ((void)0U)
#endif

#endif  // MENTOR_PI_MCU_TARGET_STM32_INCLUDE_STM32F4XX_HAL_CONF_H_
