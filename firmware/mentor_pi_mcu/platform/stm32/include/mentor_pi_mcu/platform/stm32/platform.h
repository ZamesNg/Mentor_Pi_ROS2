// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_PLATFORM_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_PLATFORM_H_

#include <cstdint>

#include "mentor_pi_mcu/platform/stm32/status.h"

namespace mentor_pi_mcu::platform::stm32 {

inline constexpr std::uint32_t kSystemClockHz = 168000000U;
inline constexpr std::uint32_t kApb1ClockHz = 42000000U;
inline constexpr std::uint32_t kApb2ClockHz = 84000000U;
inline constexpr std::uint32_t kApb1TimerClockHz = 84000000U;
inline constexpr std::uint32_t kApb2TimerClockHz = 168000000U;

// Initializes HAL, safe GPIO latches, the 168 MHz clock tree, and every
// retained peripheral. It does not start motor PWM, servo pulses, bus traffic,
// or USART1 RX. Failure leaves motor outputs disabled.
Status InitializePlatform();

// Starts the four encoder counters and the 1 kHz TIM7 release from
// MotorControlTask after the scheduler is running. TIM7's ISR uses FreeRTOS
// notification APIs, so calling this before the scheduler starts is forbidden.
// Motor drive outputs remain disabled.
Status StartControlTiming();

// Monotonic modulo-uint32 time. ROS epoch synchronization must never feed these
// functions or any actuator deadline.
std::uint32_t MonotonicMilliseconds();
std::uint32_t CycleCounter();

// Exactly one production caller is permitted: SafetySupervisorTask.
Status RefreshWatchdogFromSafetySupervisor();
bool WatchdogIsRunning();

// Writes the first concrete task offender of this boot into a torn-write-safe
// retained record. Later calls and invalid task values are ignored.
void PersistWatchdogTask(std::uint8_t task);

// Captures RCC reset flags before clearing them. Values intentionally use the
// public ControllerDiagnostics reset-reason constants.
std::uint8_t CapturedResetReason();
// Returns a retained task only when the prior reset was caused by IWDG and the
// record passed magic, complement, version, reserved-bit, and range checks.
std::uint8_t CapturedWatchdogTask();

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_PLATFORM_H_
