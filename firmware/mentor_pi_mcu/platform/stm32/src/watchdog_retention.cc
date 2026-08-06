// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/watchdog_retention.h"

#include <cstdint>

namespace mentor_pi_mcu::platform::stm32 {

std::uint32_t EncodeWatchdogTaskPayload(std::uint8_t task) {
  return (static_cast<std::uint32_t>(kWatchdogRetentionVersion) << 24U) |
         static_cast<std::uint32_t>(task);
}

bool DecodeWatchdogTask(const WatchdogRetentionRecord& record,
                        std::uint8_t* task) {
  if (task == nullptr || record.magic != kWatchdogRetentionMagic ||
      record.payload_complement != ~record.payload) {
    return false;
  }
  const std::uint8_t version = static_cast<std::uint8_t>(record.payload >> 24U);
  constexpr std::uint32_t kReservedMask = 0x00ffff00U;
  const std::uint8_t candidate =
      static_cast<std::uint8_t>(record.payload & 0xffU);
  if (version != kWatchdogRetentionVersion ||
      (record.payload & kReservedMask) != 0U ||
      candidate >= kWatchdogTaskCount) {
    return false;
  }
  *task = candidate;
  return true;
}

std::uint8_t WatchdogTaskForReset(const WatchdogRetentionRecord& record,
                                  std::uint8_t reset_reason) {
  std::uint8_t task = kNoWatchdogTask;
  return reset_reason == kIndependentWatchdogResetReason &&
                 DecodeWatchdogTask(record, &task)
             ? task
             : kNoWatchdogTask;
}

}  // namespace mentor_pi_mcu::platform::stm32
