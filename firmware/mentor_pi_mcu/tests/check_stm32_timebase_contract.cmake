# Copyright 2026 Mentor Pi contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

if(NOT DEFINED TIMEBASE_SOURCE)
  message(FATAL_ERROR "TIMEBASE_SOURCE is required")
endif()

file(READ "${TIMEBASE_SOURCE}" timebase_source)

foreach(required_fragment
    "HAL_RCC_GetPCLK1Freq()"
    "timer_clock_hz *= 2U"
    "timer_clock_hz % 1000000U != 0U"
    "__HAL_TIM_DISABLE_IT(&stm32_platform::g_tim14, TIM_IT_UPDATE)"
    "__HAL_TIM_SET_COUNTER(&stm32_platform::g_tim14, 0U)"
    "HAL_NVIC_ClearPendingIRQ(TIM8_TRG_COM_TIM14_IRQn)"
    "uwTickPrio = tick_priority")
  string(FIND "${timebase_source}" "${required_fragment}" position)
  if(position EQUAL -1)
    message(FATAL_ERROR
            "STM32 HAL timebase lacks required fragment: ${required_fragment}")
  endif()
endforeach()

string(FIND "${timebase_source}"
       "HAL_TIM_Base_Start_IT(&stm32_platform::g_tim14)" start_position)
string(FIND "${timebase_source}" "uwTickPrio = tick_priority"
       priority_position)
if(start_position EQUAL -1 OR priority_position LESS_EQUAL start_position)
  message(FATAL_ERROR
          "STM32 HAL timebase stores uwTickPrio before the timer starts")
endif()

message(STATUS "STM32 HAL timebase source contract passed")
