// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"

namespace mentor_pi_mcu::platform::stm32 {

ADC_HandleTypeDef g_adc1{};
DMA_HandleTypeDef g_dma_adc1{};
DMA_HandleTypeDef g_dma_spi1_tx{};
DMA_HandleTypeDef g_dma_usart1_rx{};
DMA_HandleTypeDef g_dma_usart1_tx{};
I2C_HandleTypeDef g_i2c1{};
IWDG_HandleTypeDef g_iwdg{};
SPI_HandleTypeDef g_spi1{};
TIM_HandleTypeDef g_tim1{};
TIM_HandleTypeDef g_tim2{};
TIM_HandleTypeDef g_tim3{};
TIM_HandleTypeDef g_tim4{};
TIM_HandleTypeDef g_tim5{};
TIM_HandleTypeDef g_tim7{};
TIM_HandleTypeDef g_tim9{};
TIM_HandleTypeDef g_tim10{};
TIM_HandleTypeDef g_tim11{};
TIM_HandleTypeDef g_tim12{};
TIM_HandleTypeDef g_tim13{};
TIM_HandleTypeDef g_tim14{};
UART_HandleTypeDef g_uart5{};
UART_HandleTypeDef g_usart1{};

}  // namespace mentor_pi_mcu::platform::stm32
