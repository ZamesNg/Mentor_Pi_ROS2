// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "mentor_pi_mcu/platform/stm32/watchdog_retention.h"

namespace {

using mentor_pi_mcu::platform::stm32::DecodeWatchdogTask;
using mentor_pi_mcu::platform::stm32::EncodeWatchdogTaskPayload;
using mentor_pi_mcu::platform::stm32::kIndependentWatchdogResetReason;
using mentor_pi_mcu::platform::stm32::kNoWatchdogTask;
using mentor_pi_mcu::platform::stm32::kWatchdogRetentionMagic;
using mentor_pi_mcu::platform::stm32::kWatchdogTaskCount;
using mentor_pi_mcu::platform::stm32::WatchdogRetentionRecord;
using mentor_pi_mcu::platform::stm32::WatchdogTaskForReset;

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                               \
      return false;                                                           \
    }                                                                         \
  } while (false)

WatchdogRetentionRecord MakeRecord(std::uint8_t task) {
  const std::uint32_t payload = EncodeWatchdogTaskPayload(task);
  return {kWatchdogRetentionMagic, payload, ~payload};
}

bool TestValidRecordsAndCauseGating() {
  for (std::uint8_t task = 0U; task < kWatchdogTaskCount; ++task) {
    const WatchdogRetentionRecord record = MakeRecord(task);
    std::uint8_t decoded = kNoWatchdogTask;
    CHECK(DecodeWatchdogTask(record, &decoded));
    CHECK(decoded == task);
    CHECK(WatchdogTaskForReset(record, kIndependentWatchdogResetReason) ==
          task);
    for (const std::uint8_t reset_reason :
         {std::uint8_t{0U}, std::uint8_t{1U}, std::uint8_t{2U},
          std::uint8_t{4U}, std::uint8_t{5U}, std::uint8_t{6U},
          std::numeric_limits<std::uint8_t>::max()}) {
      CHECK(WatchdogTaskForReset(record, reset_reason) == kNoWatchdogTask);
    }
  }
  return true;
}

bool TestTornAndMalformedRecordsAreRejected() {
  std::uint8_t decoded = kNoWatchdogTask;
  const WatchdogRetentionRecord valid = MakeRecord(2U);

  WatchdogRetentionRecord candidate = valid;
  candidate.magic = 0U;
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = {kWatchdogRetentionMagic, 0U, ~std::uint32_t{0U}};
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = {kWatchdogRetentionMagic, 0U, 0U};
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = {kWatchdogRetentionMagic, valid.payload, 0U};
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = valid;
  candidate.payload_complement ^= 1U;
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = valid;
  candidate.payload = EncodeWatchdogTaskPayload(3U);
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = valid;
  candidate.payload = (2U << 24U) | 2U;
  candidate.payload_complement = ~candidate.payload;
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = valid;
  candidate.payload |= 1U << 8U;
  candidate.payload_complement = ~candidate.payload;
  CHECK(!DecodeWatchdogTask(candidate, &decoded));

  candidate = MakeRecord(6U);
  CHECK(!DecodeWatchdogTask(candidate, &decoded));
  candidate = MakeRecord(kNoWatchdogTask);
  CHECK(!DecodeWatchdogTask(candidate, &decoded));
  CHECK(!DecodeWatchdogTask(valid, nullptr));
  return true;
}

}  // namespace

int main() {
  if (!TestValidRecordsAndCauseGating() ||
      !TestTornAndMalformedRecordsAreRejected()) {
    return 1;
  }
  std::puts("watchdog retention tests passed");
  return 0;
}
