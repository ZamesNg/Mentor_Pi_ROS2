// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_DOMAIN_CIRCULAR_RX_RING_H_
#define MENTOR_PI_MCU_DOMAIN_CIRCULAR_RX_RING_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace mentor_pi::mcu {

// Allocation-free bookkeeping and copy planning for a single-producer,
// single-consumer circular DMA buffer. The caller owns the byte storage and
// synchronization: producer updates and read commits must use the target's
// existing critical sections, while CopyRead deliberately performs no locking.
template <std::size_t kRingSizeBytes>
class CircularRxRing {
 public:
  static_assert(kRingSizeBytes >= 2U, "RX ring must have two halves");
  static_assert((kRingSizeBytes & (kRingSizeBytes - 1U)) == 0U,
                "RX ring must be a power of two");
  static_assert(kRingSizeBytes <= std::numeric_limits<std::uint32_t>::max(),
                "RX ring positions use uint32_t arithmetic");

  struct ProducerUpdate {
    std::uint32_t delta{0U};
    std::uint32_t occupied{0U};
    bool consistent{false};
    bool overrun{false};
  };

  struct ReadPlan {
    std::uint32_t consumer_position{0U};
    std::size_t copy_length{0U};
    std::size_t ring_offset{0U};
    std::size_t first_length{0U};
    bool overrun{false};
  };

  // Session reopen rebases only positions. Lifetime diagnostics intentionally
  // remain cumulative, matching the target transport's existing behavior.
  void ResetPositions(std::uint32_t position = 0U) {
    producer_position_ = position;
    consumer_position_ = position;
    previous_dma_position_ = position;
  }

  // sampled_position is the modulo-2^32 value reconstructed from a stable DMA
  // epoch/cursor snapshot. Unsigned subtraction preserves progress across wrap.
  ProducerUpdate UpdateProducer(std::uint32_t sampled_position,
                                bool sample_consistent) {
    if (!sample_consistent) {
      return {};
    }
    const std::uint32_t delta = sampled_position - previous_dma_position_;
    previous_dma_position_ = sampled_position;
    producer_position_ += delta;
    rx_wire_bytes_ = SaturatingAdd(rx_wire_bytes_, delta);

    const std::uint32_t occupied = producer_position_ - consumer_position_;
    // Avoid a redundant volatile write on the high-frequency producer path.
    // NOLINTNEXTLINE(readability-use-std-min-max)
    if (occupied > high_water_bytes_) {
      high_water_bytes_ = occupied;
    }
    return {delta, occupied, true, occupied > kRingSizeBytes};
  }

  ReadPlan PrepareRead(std::size_t capacity) const {
    ReadPlan plan{};
    plan.consumer_position = consumer_position_;
    const std::uint32_t available = producer_position_ - consumer_position_;
    if (available > kRingSizeBytes) {
      plan.overrun = true;
      return plan;
    }
    plan.copy_length = std::min(capacity, static_cast<std::size_t>(available));
    plan.ring_offset =
        static_cast<std::size_t>(consumer_position_) & (kRingSizeBytes - 1U);
    plan.first_length =
        std::min(plan.copy_length, kRingSizeBytes - plan.ring_offset);
    return plan;
  }

  static bool CopyRead(const std::uint8_t* ring, const ReadPlan& plan,
                       std::uint8_t* destination) {
    if (!HasValidLayout(plan)) {
      return false;
    }
    if (plan.copy_length == 0U) {
      return true;
    }
    if (ring == nullptr || destination == nullptr) {
      return false;
    }
    std::memcpy(destination, &ring[plan.ring_offset], plan.first_length);
    if (plan.copy_length > plan.first_length) {
      std::memcpy(destination + plan.first_length, ring,
                  plan.copy_length - plan.first_length);
    }
    return true;
  }

  // Returns false for an overrun, malformed, unavailable, or stale non-empty
  // plan. A zero-length commit is harmless and intentionally idempotent.
  bool CommitRead(const ReadPlan& plan) {
    const std::uint32_t available = producer_position_ - consumer_position_;
    if (!HasValidLayout(plan) || plan.consumer_position != consumer_position_ ||
        available > kRingSizeBytes || plan.copy_length > available) {
      return false;
    }
    consumer_position_ += static_cast<std::uint32_t>(plan.copy_length);
    return true;
  }

  std::uint32_t producer_position() const { return producer_position_; }
  std::uint32_t consumer_position() const { return consumer_position_; }
  std::uint32_t previous_dma_position() const { return previous_dma_position_; }
  std::uint32_t high_water_bytes() const { return high_water_bytes_; }
  std::uint64_t rx_wire_bytes() const { return rx_wire_bytes_; }

 private:
  static bool HasValidLayout(const ReadPlan& plan) {
    if (plan.overrun || plan.copy_length > kRingSizeBytes ||
        plan.ring_offset >= kRingSizeBytes ||
        plan.ring_offset != (static_cast<std::size_t>(plan.consumer_position) &
                             (kRingSizeBytes - 1U))) {
      return false;
    }
    const std::size_t expected_first_length =
        std::min(plan.copy_length, kRingSizeBytes - plan.ring_offset);
    return plan.first_length == expected_first_length;
  }

  static std::uint64_t SaturatingAdd(std::uint64_t value,
                                     std::uint32_t increment) {
    constexpr std::uint64_t kMaximum =
        std::numeric_limits<std::uint64_t>::max();
    return kMaximum - value < increment ? kMaximum : value + increment;
  }

  volatile std::uint32_t producer_position_{0U};
  volatile std::uint32_t consumer_position_{0U};
  volatile std::uint32_t previous_dma_position_{0U};
  volatile std::uint32_t high_water_bytes_{0U};
  volatile std::uint64_t rx_wire_bytes_{0U};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_CIRCULAR_RX_RING_H_
