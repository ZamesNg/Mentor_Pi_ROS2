// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <array>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

#include "mentor_pi_mcu/domain/fixed_containers.h"

namespace mentor_pi::mcu {
namespace {

constexpr std::uint32_t kIterationCount = 200000U;

struct CheckedValue {
  std::uint32_t sequence{0};
  std::uint32_t complement{0};
};

bool TestLatestMailbox() {
  LatestMailbox<CheckedValue> mailbox;
  static std::array<bool, kIterationCount + 1U> discarded{};
  static std::array<bool, kIterationCount + 1U> consumed{};
  std::atomic<bool> producer_done{false};
  bool valid = true;
  std::uint32_t last_sequence = 0;

  std::thread producer([&mailbox, &producer_done]() {
    for (std::uint32_t sequence = 1; sequence <= kIterationCount; ++sequence) {
      mailbox.Publish({sequence, ~sequence});
      if ((sequence % 3U) == 0U) {
        discarded[sequence] = mailbox.DiscardLatest();
      }
    }
    producer_done.store(true, std::memory_order_release);
  });
  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire) ||
           mailbox.has_unread()) {
      CheckedValue value{};
      if (mailbox.ConsumeLatest(&value)) {
        if (value.complement != ~value.sequence ||
            value.sequence <= last_sequence) {
          valid = false;
        }
        last_sequence = value.sequence;
        if (value.sequence <= kIterationCount) {
          consumed[value.sequence] = true;
        } else {
          valid = false;
        }
      }
    }
  });
  producer.join();
  consumer.join();
  for (std::uint32_t sequence = 1U; sequence <= kIterationCount; ++sequence) {
    if (discarded[sequence] && consumed[sequence]) {
      valid = false;
    }
  }
  return valid && last_sequence == kIterationCount;
}

bool TestDropOldestQueue() {
  DropOldestQueue<std::uint32_t, 16> queue;
  std::atomic<bool> producer_done{false};
  bool valid = true;
  std::uint32_t last_value = 0;

  std::thread producer([&]() {
    for (std::uint32_t value = 1; value <= kIterationCount; ++value) {
      queue.PushDropOldest(value);
    }
    producer_done.store(true, std::memory_order_release);
  });
  std::thread consumer([&]() {
    while (!producer_done.load(std::memory_order_acquire)) {
      std::uint32_t value = 0;
      if (queue.TryPop(&value)) {
        if (value <= last_value) {
          valid = false;
        }
        last_value = value;
      }
    }
    std::uint32_t value = 0;
    while (queue.TryPop(&value)) {
      if (value <= last_value) {
        valid = false;
      }
      last_value = value;
    }
  });
  producer.join();
  consumer.join();
  return valid && last_value == kIterationCount;
}

}  // namespace
}  // namespace mentor_pi::mcu

int main() {
  const bool mailbox_ok = mentor_pi::mcu::TestLatestMailbox();
  const bool queue_ok = mentor_pi::mcu::TestDropOldestQueue();
  if (!mailbox_ok || !queue_ok) {
    std::cerr << "mailbox=" << mailbox_ok << " queue=" << queue_ok << '\n';
    std::cerr << "Concurrent fixed-container test failed\n";
    return 1;
  }
  std::cout << "Concurrent fixed-container checks passed\n";
  return 0;
}
