// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_TASK_ENTRIES_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_TASK_ENTRIES_H_

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "mentor_pi_mcu/platform/stm32/status.h"

namespace mentor_pi_mcu::platform::stm32 {

enum class TaskId : std::uint8_t {
  kSafetySupervisor = 0,
  kMotorControl,
  kMicroRos,
  kBusServo,
  kSensor,
  kPeripheral,
  kCount,
};

using TaskMain = void (*)(void* context);

struct TaskHook {
  TaskMain main;
  void* context;
};

using TaskHooks =
    std::array<TaskHook, static_cast<std::size_t>(TaskId::kCount)>;

// Creates all six tasks with xTaskCreateStatic and the locked byte budgets and
// priorities. Every entry must be non-null and must not return.
Status CreateStaticTasks(const TaskHooks& hooks);
TaskHandle_t GetTaskHandle(TaskId task_id);

// Bounded ISR notifications. The corresponding owner decides what work to do.
void NotifyTaskFromIsr(TaskId task_id);

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_TASK_ENTRIES_H_
