// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_INTERRUPTS_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_INTERRUPTS_H_

// The startup file supplies the vector table. These C-linkage definitions are
// deliberately kept as thin dispatch shims.
extern "C" void ADC_IRQHandler();
extern "C" void DMA2_Stream0_IRQHandler();
extern "C" void DMA2_Stream2_IRQHandler();
extern "C" void DMA2_Stream3_IRQHandler();
extern "C" void DMA2_Stream7_IRQHandler();
extern "C" void EXTI15_10_IRQHandler();
extern "C" void SPI1_IRQHandler();
extern "C" void TIM7_IRQHandler();
extern "C" void TIM8_BRK_TIM12_IRQHandler();
extern "C" void TIM8_UP_TIM13_IRQHandler();
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler();
extern "C" void UART5_IRQHandler();
extern "C" void USART1_IRQHandler();

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_INTERRUPTS_H_
