// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/task_entries.h"

#include <array>
#include <cstddef>

#include "mentor_pi_mcu/platform/stm32/memory_regions.h"
#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/platform.h"

#if configSUPPORT_STATIC_ALLOCATION != 1
#error "RRCLite requires configSUPPORT_STATIC_ALLOCATION=1"
#endif

#if configMAX_PRIORITIES < 8
#error "RRCLite requires configMAX_PRIORITIES >= 8"
#endif

static_assert(configTICK_RATE_HZ == static_cast<TickType_t>(1000U),
              "RRCLite bounded millisecond waits require a 1 kHz RTOS tick");

namespace mentor_pi_mcu::platform::stm32 {
namespace {

constexpr std::size_t StackElements(std::size_t stack_bytes) {
  return (stack_bytes + sizeof(StackType_t) - 1U) / sizeof(StackType_t);
}

constexpr std::size_t kSafetyStackElements = StackElements(1024U);
constexpr std::size_t kMotorStackElements = StackElements(2048U);
constexpr std::size_t kMicroRosStackElements = StackElements(16U * 1024U);
constexpr std::size_t kBusServoStackElements = StackElements(3U * 1024U);
constexpr std::size_t kSensorStackElements = StackElements(4U * 1024U);
constexpr std::size_t kPeripheralStackElements = StackElements(4U * 1024U);
constexpr std::size_t kTaskCount = static_cast<std::size_t>(TaskId::kCount);

struct TaskContext {
  TaskHook hook;
  TaskId id;
};

StaticTask_t g_safety_task_control_block{};
StaticTask_t g_motor_task_control_block{};
StaticTask_t g_micro_ros_task_control_block{};
StaticTask_t g_bus_servo_task_control_block{};
StaticTask_t g_sensor_task_control_block{};
StaticTask_t g_peripheral_task_control_block{};

std::array<StackType_t, kSafetyStackElements> g_safety_stack{};
// The motor task never supplies its stack to DMA. Keep this CPU-only stack in
// CCM so both independently budgeted SRAM banks retain release headroom.
MENTOR_PI_CCM_BUFFER std::array<StackType_t, kMotorStackElements>
    g_motor_stack{};
std::array<StackType_t, kMicroRosStackElements> g_micro_ros_stack{};
std::array<StackType_t, kBusServoStackElements> g_bus_servo_stack{};
std::array<StackType_t, kSensorStackElements> g_sensor_stack{};
std::array<StackType_t, kPeripheralStackElements> g_peripheral_stack{};

std::array<TaskContext, kTaskCount> g_task_contexts{};
std::array<TaskHandle_t, kTaskCount> g_task_handles{};
bool g_tasks_created = false;

void TaskEntry(void* argument) {
  auto* const context = static_cast<TaskContext*>(argument);
  if (context == nullptr || context->hook.main == nullptr) {
    EmergencyStopMotors();
    vTaskSuspend(nullptr);
    return;
  }

  // TIM7's ISR uses FreeRTOS notification APIs, so it must not be enabled
  // before the scheduler is running. MotorControlTask starts it here, before
  // entering the owner's non-returning loop. A failure leaves every bridge
  // disabled; the absent motor heartbeat then makes SafetySupervisorTask
  // withhold the watchdog refresh after its bounded startup grace.
  if (context->id == TaskId::kMotorControl &&
      StartControlTiming() != Status::kOk) {
    EmergencyStopMotors();
    vTaskSuspend(nullptr);
    return;
  }

  context->hook.main(context->hook.context);

  // A production task returning is a fatal task stall, not normal shutdown.
  EmergencyStopMotors();
  vTaskSuspend(nullptr);
}

TaskHandle_t CreateOneTask(TaskId id, const char* name, UBaseType_t priority,
                           StackType_t* stack, std::uint32_t stack_elements,
                           StaticTask_t* control_block) {
  const std::size_t index = static_cast<std::size_t>(id);
  return xTaskCreateStatic(TaskEntry, name, stack_elements,
                           &g_task_contexts[index], priority, stack,
                           control_block);
}

}  // namespace

Status CreateStaticTasks(const TaskHooks& hooks) {
  if (g_tasks_created) {
    return Status::kBusy;
  }
  for (const TaskHook& hook : hooks) {
    if (hook.main == nullptr) {
      return Status::kInvalidArgument;
    }
  }

  for (std::size_t index = 0; index < kTaskCount; ++index) {
    g_task_contexts[index] =
        TaskContext{hooks[index], static_cast<TaskId>(index)};
  }

  g_task_handles[static_cast<std::size_t>(TaskId::kSafetySupervisor)] =
      CreateOneTask(TaskId::kSafetySupervisor, "SafetySupervisorTask", 7U,
                    g_safety_stack.data(),
                    static_cast<std::uint32_t>(g_safety_stack.size()),
                    &g_safety_task_control_block);
  g_task_handles[static_cast<std::size_t>(TaskId::kMotorControl)] =
      CreateOneTask(TaskId::kMotorControl, "MotorControlTask", 6U,
                    g_motor_stack.data(),
                    static_cast<std::uint32_t>(g_motor_stack.size()),
                    &g_motor_task_control_block);
  g_task_handles[static_cast<std::size_t>(TaskId::kMicroRos)] = CreateOneTask(
      TaskId::kMicroRos, "MicroRosTask", 5U, g_micro_ros_stack.data(),
      static_cast<std::uint32_t>(g_micro_ros_stack.size()),
      &g_micro_ros_task_control_block);
  g_task_handles[static_cast<std::size_t>(TaskId::kBusServo)] = CreateOneTask(
      TaskId::kBusServo, "BusServoTask", 4U, g_bus_servo_stack.data(),
      static_cast<std::uint32_t>(g_bus_servo_stack.size()),
      &g_bus_servo_task_control_block);
  g_task_handles[static_cast<std::size_t>(TaskId::kSensor)] =
      CreateOneTask(TaskId::kSensor, "SensorTask", 3U, g_sensor_stack.data(),
                    static_cast<std::uint32_t>(g_sensor_stack.size()),
                    &g_sensor_task_control_block);
  g_task_handles[static_cast<std::size_t>(TaskId::kPeripheral)] = CreateOneTask(
      TaskId::kPeripheral, "PeripheralTask", 2U, g_peripheral_stack.data(),
      static_cast<std::uint32_t>(g_peripheral_stack.size()),
      &g_peripheral_task_control_block);

  for (TaskHandle_t handle : g_task_handles) {
    if (handle == nullptr) {
      EmergencyStopMotors();
      return Status::kIoError;
    }
  }
  g_tasks_created = true;
  return Status::kOk;
}

TaskHandle_t GetTaskHandle(TaskId task_id) {
  const std::size_t index = static_cast<std::size_t>(task_id);
  if (index >= g_task_handles.size()) {
    return nullptr;
  }
  return g_task_handles[index];
}

void NotifyTaskFromIsr(TaskId task_id) {
  TaskHandle_t const task = GetTaskHandle(task_id);
  if (task == nullptr) {
    return;
  }
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

}  // namespace mentor_pi_mcu::platform::stm32
