// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_HAL_HANDLES_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_HAL_HANDLES_H_

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace mentor_pi_mcu::platform::stm32 {

extern ADC_HandleTypeDef g_adc1;
extern DMA_HandleTypeDef g_dma_adc1;
extern DMA_HandleTypeDef g_dma_spi1_tx;
extern DMA_HandleTypeDef g_dma_usart1_rx;
extern DMA_HandleTypeDef g_dma_usart1_tx;
extern I2C_HandleTypeDef g_i2c1;
extern IWDG_HandleTypeDef g_iwdg;
extern SPI_HandleTypeDef g_spi1;
extern TIM_HandleTypeDef g_tim1;
extern TIM_HandleTypeDef g_tim2;
extern TIM_HandleTypeDef g_tim3;
extern TIM_HandleTypeDef g_tim4;
extern TIM_HandleTypeDef g_tim5;
extern TIM_HandleTypeDef g_tim7;
extern TIM_HandleTypeDef g_tim9;
extern TIM_HandleTypeDef g_tim10;
extern TIM_HandleTypeDef g_tim11;
extern TIM_HandleTypeDef g_tim12;
extern TIM_HandleTypeDef g_tim13;
extern TIM_HandleTypeDef g_tim14;
extern UART_HandleTypeDef g_uart5;
extern UART_HandleTypeDef g_usart1;

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_HAL_HANDLES_H_
