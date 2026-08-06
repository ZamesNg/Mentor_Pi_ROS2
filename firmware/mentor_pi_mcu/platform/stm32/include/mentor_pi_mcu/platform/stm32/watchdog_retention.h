// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_WATCHDOG_RETENTION_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_WATCHDOG_RETENTION_H_

#include <cstdint>

namespace mentor_pi_mcu::platform::stm32 {

inline constexpr std::uint32_t kWatchdogRetentionMagic = 0x52525732U;
inline constexpr std::uint8_t kWatchdogRetentionVersion = 1U;
inline constexpr std::uint8_t kWatchdogTaskCount = 6U;
inline constexpr std::uint8_t kNoWatchdogTask = 255U;
inline constexpr std::uint8_t kIndependentWatchdogResetReason = 3U;

struct alignas(std::uint32_t) WatchdogRetentionRecord {
  std::uint32_t magic;
  std::uint32_t payload;
  std::uint32_t payload_complement;
};

static_assert(sizeof(WatchdogRetentionRecord) == 12U);
static_assert(alignof(WatchdogRetentionRecord) == alignof(std::uint32_t));

std::uint32_t EncodeWatchdogTaskPayload(std::uint8_t task);
bool DecodeWatchdogTask(const WatchdogRetentionRecord& record,
                        std::uint8_t* task);
std::uint8_t WatchdogTaskForReset(const WatchdogRetentionRecord& record,
                                  std::uint8_t reset_reason);

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_WATCHDOG_RETENTION_H_
