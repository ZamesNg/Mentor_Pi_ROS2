// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/interrupts.h"

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"
#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"

namespace stm32_platform = mentor_pi_mcu::platform::stm32;

extern "C" void DMA2_Stream0_IRQHandler() {
  HAL_DMA_IRQHandler(&stm32_platform::g_dma_adc1);
}

extern "C" void DMA2_Stream2_IRQHandler() {
  HAL_DMA_IRQHandler(&stm32_platform::g_dma_usart1_rx);
}

extern "C" void DMA2_Stream3_IRQHandler() {
  HAL_DMA_IRQHandler(&stm32_platform::g_dma_spi1_tx);
}

extern "C" void DMA2_Stream7_IRQHandler() {
  HAL_DMA_IRQHandler(&stm32_platform::g_dma_usart1_tx);
}

extern "C" void USART1_IRQHandler() {
  HAL_UART_IRQHandler(&stm32_platform::g_usart1);
}

extern "C" void UART5_IRQHandler() {
  HAL_UART_IRQHandler(&stm32_platform::g_uart5);
}

extern "C" void SPI1_IRQHandler() {
  HAL_SPI_IRQHandler(&stm32_platform::g_spi1);
}

extern "C" void ADC_IRQHandler() {
  HAL_ADC_IRQHandler(&stm32_platform::g_adc1);
}

extern "C" void EXTI15_10_IRQHandler() {
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    stm32_platform::HandleImuDataReadyFromIsr();
  }
}

extern "C" void TIM7_IRQHandler() {
  if (__HAL_TIM_GET_FLAG(&stm32_platform::g_tim7, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&stm32_platform::g_tim7, TIM_IT_UPDATE) !=
          RESET) {
    __HAL_TIM_CLEAR_IT(&stm32_platform::g_tim7, TIM_IT_UPDATE);
    stm32_platform::HandleMotorReleaseFromIsr();
  }
}

extern "C" void TIM8_BRK_TIM12_IRQHandler() {
  if (__HAL_TIM_GET_FLAG(&stm32_platform::g_tim12, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&stm32_platform::g_tim12, TIM_IT_UPDATE) !=
          RESET) {
    __HAL_TIM_CLEAR_IT(&stm32_platform::g_tim12, TIM_IT_UPDATE);
    stm32_platform::HandleBuzzerTimerFromIsr();
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler() {
  if (__HAL_TIM_GET_FLAG(&stm32_platform::g_tim13, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&stm32_platform::g_tim13, TIM_IT_UPDATE) !=
          RESET) {
    __HAL_TIM_CLEAR_IT(&stm32_platform::g_tim13, TIM_IT_UPDATE);
    stm32_platform::HandlePwmServoTimerFromIsr();
  }
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler() {
  if (__HAL_TIM_GET_FLAG(&stm32_platform::g_tim14, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&stm32_platform::g_tim14, TIM_IT_UPDATE) !=
          RESET) {
    __HAL_TIM_CLEAR_IT(&stm32_platform::g_tim14, TIM_IT_UPDATE);
    HAL_IncTick();
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    stm32_platform::HandleUsart1RxDmaBoundaryFromIsr();
  } else if (uart->Instance == UART5) {
    stm32_platform::HandleBusUartRxCompleteFromIsr();
  }
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    stm32_platform::HandleUsart1RxDmaBoundaryFromIsr();
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    stm32_platform::HandleUsart1TxCompleteFromIsr();
  } else if (uart->Instance == UART5) {
    stm32_platform::HandleBusUartTxCompleteFromIsr();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    stm32_platform::HandleUsart1ErrorFromIsr(uart->ErrorCode);
  } else if (uart->Instance == UART5) {
    stm32_platform::HandleBusUartErrorFromIsr();
  }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* spi) {
  if (spi->Instance == SPI1) {
    stm32_platform::HandleRgbCompleteFromIsr();
  }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* spi) {
  if (spi->Instance == SPI1) {
    stm32_platform::HandleRgbErrorFromIsr();
  }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc) {
  if (adc->Instance == ADC1) {
    stm32_platform::HandleAdcCompleteFromIsr();
  }
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* adc) {
  if (adc->Instance == ADC1) {
    stm32_platform::HandleAdcErrorFromIsr();
  }
}
