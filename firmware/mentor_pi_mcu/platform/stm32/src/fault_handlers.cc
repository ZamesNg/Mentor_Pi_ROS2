// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"
}

#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/platform.h"

namespace {

[[noreturn]] void StopMotorsAndReset() {
  __disable_irq();
  mentor_pi_mcu::platform::stm32::EmergencyStopMotors();
  __DSB();
  if (!mentor_pi_mcu::platform::stm32::WatchdogIsRunning()) {
    NVIC_SystemReset();
  }
  // Once IWDG is active, deliberately stop refreshing and let it establish
  // the characterized reset reason rather than hiding failure with a soft
  // reset.
  while (true) {
    __NOP();
  }
}

}  // namespace

extern "C" void Error_Handler() { StopMotorsAndReset(); }
extern "C" void NMI_Handler() { StopMotorsAndReset(); }
extern "C" void HardFault_Handler() { StopMotorsAndReset(); }
extern "C" void MemManage_Handler() { StopMotorsAndReset(); }
extern "C" void BusFault_Handler() { StopMotorsAndReset(); }
extern "C" void UsageFault_Handler() { StopMotorsAndReset(); }

extern "C" void vApplicationStackOverflowHook(TaskHandle_t task,
                                              char* task_name) {
  static_cast<void>(task);
  static_cast<void>(task_name);
  StopMotorsAndReset();
}

extern "C" void vApplicationMallocFailedHook() { StopMotorsAndReset(); }

extern "C" void MentorPiFreeRtosAssertFailure() { StopMotorsAndReset(); }

#if defined(USE_FULL_ASSERT)
extern "C" void assert_failed(std::uint8_t* file, std::uint32_t line) {
  static_cast<void>(file);
  static_cast<void>(line);
  StopMotorsAndReset();
}
#endif
