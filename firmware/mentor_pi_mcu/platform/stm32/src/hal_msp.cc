// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"

extern "C" {
#include "FreeRTOS.h"
}

#include <cstdint>

namespace mentor_pi_mcu::platform::stm32 {
void RecordMspError();
}  // namespace mentor_pi_mcu::platform::stm32

namespace {

using mentor_pi_mcu::platform::stm32::g_dma_adc1;
using mentor_pi_mcu::platform::stm32::g_dma_spi1_tx;
using mentor_pi_mcu::platform::stm32::g_dma_usart1_rx;
using mentor_pi_mcu::platform::stm32::g_dma_usart1_tx;
using mentor_pi_mcu::platform::stm32::RecordMspError;

constexpr std::uint32_t kMotorReleaseIrqPriority = 5U;
constexpr std::uint32_t kTransportIrqPriority = 6U;
constexpr std::uint32_t kBusServoIrqPriority = 7U;
constexpr std::uint32_t kPeripheralIrqPriority = 8U;

#if defined(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
static_assert(kMotorReleaseIrqPriority >=
                  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
              "TIM7 must be callable through FreeRTOS FromISR APIs");
static_assert(kTransportIrqPriority >=
                  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
              "USART1 HAL DMA callbacks must be FreeRTOS FromISR-safe");
#endif

bool InitializeDma(DMA_HandleTypeDef* dma) {
  if (HAL_DMA_Init(dma) != HAL_OK) {
    RecordMspError();
    return false;
  }
  return true;
}

}  // namespace

extern "C" void HAL_MspInit() {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(PendSV_IRQn, 15U, 0U);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, kPeripheralIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    g_dma_usart1_tx.Instance = DMA2_Stream7;
    g_dma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    g_dma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_dma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_dma_usart1_tx.Init.Mode = DMA_NORMAL;
    g_dma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    g_dma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (InitializeDma(&g_dma_usart1_tx)) {
      __HAL_LINKDMA(uart, hdmatx, g_dma_usart1_tx);
    }

    g_dma_usart1_rx.Instance = DMA2_Stream2;
    g_dma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    g_dma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_dma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_dma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    g_dma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    g_dma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (InitializeDma(&g_dma_usart1_rx)) {
      __HAL_LINKDMA(uart, hdmarx, g_dma_usart1_rx);
    }

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, kTransportIrqPriority, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, kTransportIrqPriority, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, kTransportIrqPriority, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    return;
  }

  if (uart->Instance == UART5) {
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_NVIC_SetPriority(UART5_IRQn, kBusServoIrqPriority, 0U);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  }
}

extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* uart) {
  if (uart->Instance == USART1) {
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    HAL_DMA_DeInit(&g_dma_usart1_rx);
    HAL_DMA_DeInit(&g_dma_usart1_tx);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    __HAL_RCC_USART1_CLK_DISABLE();
  } else if (uart->Instance == UART5) {
    HAL_NVIC_DisableIRQ(UART5_IRQn);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
    __HAL_RCC_UART5_CLK_DISABLE();
  }
}

extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* spi) {
  if (spi->Instance != SPI1) {
    return;
  }
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);

  g_dma_spi1_tx.Instance = DMA2_Stream3;
  g_dma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
  g_dma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  g_dma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  g_dma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  g_dma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  g_dma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  g_dma_spi1_tx.Init.Mode = DMA_NORMAL;
  g_dma_spi1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
  g_dma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (InitializeDma(&g_dma_spi1_tx)) {
    __HAL_LINKDMA(spi, hdmatx, g_dma_spi1_tx);
  }

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, kPeripheralIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority(SPI1_IRQn, kPeripheralIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

extern "C" void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spi) {
  if (spi->Instance != SPI1) {
    return;
  }
  HAL_NVIC_DisableIRQ(SPI1_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
  HAL_DMA_DeInit(&g_dma_spi1_tx);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_7);
  __HAL_RCC_SPI1_CLK_DISABLE();
}

extern "C" void HAL_I2C_MspInit(I2C_HandleTypeDef* i2c) {
  if (i2c->Instance != I2C1) {
    return;
  }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);
}

extern "C" void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2c) {
  if (i2c->Instance != I2C1) {
    return;
  }
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  __HAL_RCC_I2C1_CLK_DISABLE();
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef* adc) {
  if (adc->Instance != ADC1) {
    return;
  }
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  g_dma_adc1.Instance = DMA2_Stream0;
  g_dma_adc1.Init.Channel = DMA_CHANNEL_0;
  g_dma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  g_dma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  g_dma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  g_dma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  g_dma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  g_dma_adc1.Init.Mode = DMA_NORMAL;
  g_dma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  g_dma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (InitializeDma(&g_dma_adc1)) {
    __HAL_LINKDMA(adc, DMA_Handle, g_dma_adc1);
  }

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, kPeripheralIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(ADC_IRQn, kPeripheralIrqPriority, 0U);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adc) {
  if (adc->Instance != ADC1) {
    return;
  }
  HAL_NVIC_DisableIRQ(ADC_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  HAL_DMA_DeInit(&g_dma_adc1);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
  __HAL_RCC_ADC1_CLK_DISABLE();
}

extern "C" void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* timer) {
  if (timer->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  } else if (timer->Instance == TIM9) {
    __HAL_RCC_TIM9_CLK_ENABLE();
  } else if (timer->Instance == TIM10) {
    __HAL_RCC_TIM10_CLK_ENABLE();
  } else if (timer->Instance == TIM11) {
    __HAL_RCC_TIM11_CLK_ENABLE();
  }
}

extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* timer) {
  IRQn_Type irq = NonMaskableInt_IRQn;
  std::uint32_t priority = kPeripheralIrqPriority;
  if (timer->Instance == TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
    irq = TIM7_IRQn;
    priority = kMotorReleaseIrqPriority;
  } else if (timer->Instance == TIM12) {
    __HAL_RCC_TIM12_CLK_ENABLE();
    irq = TIM8_BRK_TIM12_IRQn;
  } else if (timer->Instance == TIM13) {
    __HAL_RCC_TIM13_CLK_ENABLE();
    irq = TIM8_UP_TIM13_IRQn;
  } else if (timer->Instance == TIM14) {
    __HAL_RCC_TIM14_CLK_ENABLE();
    irq = TIM8_TRG_COM_TIM14_IRQn;
  } else {
    return;
  }
  HAL_NVIC_SetPriority(irq, priority, 0U);
  HAL_NVIC_EnableIRQ(irq);
}

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* timer) {
  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  if (timer->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_15;
    gpio.Pull = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio);
  } else if (timer->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);
  } else if (timer->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &gpio);
  } else if (timer->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Pull = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);
  }
}
