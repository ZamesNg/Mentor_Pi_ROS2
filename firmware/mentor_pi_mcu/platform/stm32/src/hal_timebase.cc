// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstdint>

#include "mentor_pi_mcu/platform/stm32/hal_handles.h"

namespace stm32_platform = mentor_pi_mcu::platform::stm32;

extern "C" HAL_StatusTypeDef HAL_InitTick(std::uint32_t tick_priority) {
  if (tick_priority >= (1UL << __NVIC_PRIO_BITS)) {
    return HAL_ERROR;
  }

  RCC_ClkInitTypeDef clocks{};
  std::uint32_t flash_latency = 0U;
  HAL_RCC_GetClockConfig(&clocks, &flash_latency);

  std::uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) {
    timer_clock_hz *= 2U;
  }
  const std::uint32_t prescaler = timer_clock_hz / 1000000U;
  if (prescaler == 0U || timer_clock_hz % 1000000U != 0U) {
    return HAL_ERROR;
  }

  // HAL_Init() configures this timer once on the reset HSI clock. Later,
  // HAL_RCC_ClockConfig() calls HAL_InitTick(uwTickPrio) after selecting the
  // PLL clock. Stop the first configuration before replacing its prescaler so
  // no update can race the clock-tree transition.
  if (stm32_platform::g_tim14.Instance == TIM14) {
    __HAL_TIM_DISABLE_IT(&stm32_platform::g_tim14, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&stm32_platform::g_tim14);
  }

  stm32_platform::g_tim14.Instance = TIM14;
  stm32_platform::g_tim14.Init.Prescaler = prescaler - 1U;
  stm32_platform::g_tim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  stm32_platform::g_tim14.Init.Period = 999U;
  stm32_platform::g_tim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  stm32_platform::g_tim14.Init.AutoReloadPreload =
      TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&stm32_platform::g_tim14) != HAL_OK) {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(&stm32_platform::g_tim14, 0U);
  __HAL_TIM_CLEAR_FLAG(&stm32_platform::g_tim14, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM8_TRG_COM_TIM14_IRQn);
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, tick_priority, 0U);
  const HAL_StatusTypeDef status =
      HAL_TIM_Base_Start_IT(&stm32_platform::g_tim14);
  if (status == HAL_OK) {
    // STM32 HAL deliberately reuses this value when the clock tree changes.
    // Leaving the reset sentinel here makes the second initialization fail and
    // accelerates the 1 ms tick by 84 MHz / 16 MHz = 5.25.
    uwTickPrio = tick_priority;
  }
  return status;
}

extern "C" void HAL_SuspendTick() {
  __HAL_TIM_DISABLE_IT(&stm32_platform::g_tim14, TIM_IT_UPDATE);
}

extern "C" void HAL_ResumeTick() {
  __HAL_TIM_ENABLE_IT(&stm32_platform::g_tim14, TIM_IT_UPDATE);
}
