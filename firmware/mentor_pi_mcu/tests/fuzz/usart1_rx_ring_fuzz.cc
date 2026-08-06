// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/circular_dma_position.h"
#include "mentor_pi_mcu/domain/circular_rx_ring.h"

namespace mentor_pi::mcu {
namespace {

using fuzz::FuzzInput;
using fuzz::Require;

constexpr std::size_t kRingSizeBytes = 8192U;
constexpr std::size_t kMaximumOperations = 64U;
constexpr std::size_t kMaximumByteWork = kRingSizeBytes * 2U;

using RxPosition = CircularDmaPosition<kRingSizeBytes>;
using RxRing = CircularRxRing<kRingSizeBytes>;

struct ReferenceState {
  std::uint32_t producer_position{0U};
  std::uint32_t consumer_position{0U};
  std::uint32_t previous_dma_position{0U};
  std::uint32_t high_water_bytes{0U};
  std::uint64_t rx_wire_bytes{0U};
};

std::uint64_t SaturatingAdd(std::uint64_t value, std::uint32_t increment) {
  constexpr std::uint64_t kMaximum = std::numeric_limits<std::uint64_t>::max();
  return kMaximum - value < increment ? kMaximum : value + increment;
}

void RequireState(const RxRing& ring, const ReferenceState& reference) {
  Require(ring.producer_position() == reference.producer_position);
  Require(ring.consumer_position() == reference.consumer_position);
  Require(ring.previous_dma_position() == reference.previous_dma_position);
  Require(ring.high_water_bytes() == reference.high_water_bytes);
  Require(ring.rx_wire_bytes() == reference.rx_wire_bytes);
}

std::uint32_t ReconstructConsistentPosition(std::uint32_t absolute_position) {
  const std::uint32_t boundary_count =
      absolute_position / RxPosition::kHalfSizeBytes;
  const std::uint32_t cursor =
      absolute_position % static_cast<std::uint32_t>(kRingSizeBytes);
  Require(RxPosition::IsConsistent(boundary_count, cursor));
  return RxPosition::Reconstruct(boundary_count, cursor);
}

void ProduceBytes(FuzzInput* input, std::size_t requested, RxRing* state,
                  ReferenceState* reference,
                  std::array<std::uint8_t, kRingSizeBytes>* ring,
                  std::size_t* byte_work) {
  const std::size_t budget = kMaximumByteWork - *byte_work;
  const std::size_t length = std::min(requested, budget);
  for (std::size_t index = 0U; index < length; ++index) {
    const std::size_t ring_index =
        (static_cast<std::size_t>(reference->producer_position) + index) &
        (kRingSizeBytes - 1U);
    (*ring)[ring_index] = input->ReadU8();
  }
  *byte_work += length;

  const std::uint32_t sampled_position = ReconstructConsistentPosition(
      reference->producer_position + static_cast<std::uint32_t>(length));
  const RxRing::ProducerUpdate update =
      state->UpdateProducer(sampled_position, true);
  const std::uint32_t delta =
      sampled_position - reference->previous_dma_position;
  reference->previous_dma_position = sampled_position;
  reference->producer_position += delta;
  reference->rx_wire_bytes = SaturatingAdd(reference->rx_wire_bytes, delta);
  const std::uint32_t occupied =
      reference->producer_position - reference->consumer_position;
  reference->high_water_bytes = std::max(reference->high_water_bytes, occupied);

  Require(update.consistent);
  Require(update.delta == delta);
  Require(update.occupied == occupied);
  Require(update.overrun == (occupied > kRingSizeBytes));
  RequireState(*state, *reference);
}

void Produce(FuzzInput* input, std::uint8_t selector, RxRing* state,
             ReferenceState* reference,
             std::array<std::uint8_t, kRingSizeBytes>* ring,
             std::size_t* byte_work) {
  std::size_t requested = input->ReadU16();
  if ((selector & 0x80U) != 0U) {
    requested = kRingSizeBytes + 1U;
  }
  ProduceBytes(input, requested, state, reference, ring, byte_work);
}

void RejectInconsistentSample(FuzzInput* input, RxRing* state,
                              const ReferenceState& reference) {
  const std::uint32_t arbitrary_position = input->ReadU32();
  const RxRing::ProducerUpdate update =
      state->UpdateProducer(arbitrary_position, false);
  Require(!update.consistent);
  Require(!update.overrun);
  Require(update.delta == 0U && update.occupied == 0U);
  RequireState(*state, reference);
}

void Read(FuzzInput* input, std::uint8_t selector, RxRing* state,
          ReferenceState* reference,
          const std::array<std::uint8_t, kRingSizeBytes>& ring,
          std::array<std::uint8_t, kRingSizeBytes>* output,
          std::size_t* byte_work) {
  std::size_t capacity = input->ReadU16();
  if ((selector & 0x80U) != 0U) {
    capacity = kRingSizeBytes;
  } else {
    capacity %= kRingSizeBytes + 1U;
  }
  capacity = std::min(capacity, kMaximumByteWork - *byte_work);
  const std::uint32_t available =
      reference->producer_position - reference->consumer_position;
  const bool overrun = available > kRingSizeBytes;
  const std::size_t expected_length =
      overrun ? 0U : std::min(capacity, static_cast<std::size_t>(available));
  const std::size_t expected_offset =
      static_cast<std::size_t>(reference->consumer_position) &
      (kRingSizeBytes - 1U);
  const std::size_t expected_first =
      std::min(expected_length, kRingSizeBytes - expected_offset);

  const RxRing::ReadPlan plan = state->PrepareRead(capacity);
  Require(plan.consumer_position == reference->consumer_position);
  Require(plan.overrun == overrun);
  Require(plan.copy_length == expected_length);
  if (!overrun) {
    Require(plan.ring_offset == expected_offset);
    Require(plan.first_length == expected_first);
  }
  const bool copied = RxRing::CopyRead(ring.data(), plan, output->data());
  Require(copied == !overrun);
  if (overrun) {
    Require(!state->CommitRead(plan));
    RequireState(*state, *reference);
    return;
  }

  for (std::size_t index = 0U; index < expected_length; ++index) {
    const std::size_t ring_index =
        (expected_offset + index) & (kRingSizeBytes - 1U);
    Require((*output)[index] == ring[ring_index]);
  }
  Require(state->CommitRead(plan));
  // A zero-length commit is intentionally idempotent because it does not
  // advance the consumer position. A non-empty plan becomes stale at once.
  Require(state->CommitRead(plan) == (expected_length == 0U));
  reference->consumer_position += static_cast<std::uint32_t>(expected_length);
  *byte_work += expected_length;
  RequireState(*state, *reference);
}

void ReadAcrossProducerProgress(
    FuzzInput* input, std::uint8_t selector, RxRing* state,
    ReferenceState* reference, std::array<std::uint8_t, kRingSizeBytes>* ring,
    std::array<std::uint8_t, kRingSizeBytes>* output, std::size_t* byte_work) {
  std::size_t capacity = input->ReadU16() % (kRingSizeBytes + 1U);
  capacity = std::min(capacity, kMaximumByteWork - *byte_work);
  const std::uint32_t available_before =
      reference->producer_position - reference->consumer_position;
  const RxRing::ReadPlan plan = state->PrepareRead(capacity);
  const bool overrun_before = available_before > kRingSizeBytes;
  Require(plan.overrun == overrun_before);
  Require(RxRing::CopyRead(ring->data(), plan, output->data()) ==
          !overrun_before);
  if (overrun_before) {
    Require(!state->CommitRead(plan));
    RequireState(*state, *reference);
    return;
  }

  for (std::size_t index = 0U; index < plan.copy_length; ++index) {
    const std::size_t ring_index =
        (plan.ring_offset + index) & (kRingSizeBytes - 1U);
    Require((*output)[index] == (*ring)[ring_index]);
  }
  *byte_work += plan.copy_length;

  const std::size_t headroom = kRingSizeBytes - available_before;
  const std::size_t requested =
      (selector & 0x80U) != 0U
          ? headroom + 1U
          : static_cast<std::size_t>(input->ReadU16()) % (headroom + 1U);
  ProduceBytes(input, requested, state, reference, ring, byte_work);

  const std::uint32_t available_after =
      reference->producer_position - reference->consumer_position;
  const bool commit_expected = available_after <= kRingSizeBytes;
  Require(state->CommitRead(plan) == commit_expected);
  if (commit_expected) {
    reference->consumer_position +=
        static_cast<std::uint32_t>(plan.copy_length);
  }
  RequireState(*state, *reference);
}

void Reset(FuzzInput* input, RxRing* state, ReferenceState* reference) {
  const std::uint32_t position = input->ReadU32();
  state->ResetPositions(position);
  reference->producer_position = position;
  reference->consumer_position = position;
  reference->previous_dma_position = position;
  RequireState(*state, *reference);
}

bool HasValidLayout(const RxRing::ReadPlan& plan) {
  if (plan.overrun || plan.copy_length > kRingSizeBytes ||
      plan.ring_offset >= kRingSizeBytes ||
      plan.ring_offset != (static_cast<std::size_t>(plan.consumer_position) &
                           (kRingSizeBytes - 1U))) {
    return false;
  }
  return plan.first_length ==
         std::min(plan.copy_length, kRingSizeBytes - plan.ring_offset);
}

void RejectMalformedPlan(FuzzInput* input, std::uint8_t selector, RxRing* state,
                         ReferenceState* reference,
                         const std::array<std::uint8_t, kRingSizeBytes>& ring,
                         std::array<std::uint8_t, kRingSizeBytes>* output) {
  RxRing::ReadPlan plan = state->PrepareRead(input->ReadU16());
  switch ((selector >> 3U) % 5U) {
    case 0U:
      plan.overrun = false;
      plan.copy_length = kRingSizeBytes + 1U;
      break;
    case 1U:
      plan.overrun = false;
      plan.ring_offset = kRingSizeBytes;
      break;
    case 2U:
      plan.overrun = false;
      plan.copy_length = 1U;
      plan.ring_offset = static_cast<std::size_t>(plan.consumer_position) &
                         (kRingSizeBytes - 1U);
      plan.first_length = 0U;
      break;
    case 3U:
      plan.overrun = false;
      plan.consumer_position += static_cast<std::uint32_t>(kRingSizeBytes);
      plan.ring_offset = static_cast<std::size_t>(plan.consumer_position) &
                         (kRingSizeBytes - 1U);
      plan.copy_length = 0U;
      plan.first_length = 0U;
      break;
    case 4U:
      plan.overrun = true;
      break;
    default:
      fuzz::FailInvariant();
  }

  const bool layout_valid = HasValidLayout(plan);
  Require(RxRing::CopyRead(ring.data(), plan, output->data()) == layout_valid);
  const std::uint32_t available =
      reference->producer_position - reference->consumer_position;
  const bool commit_valid =
      layout_valid && plan.consumer_position == reference->consumer_position &&
      available <= kRingSizeBytes && plan.copy_length <= available;
  Require(state->CommitRead(plan) == commit_valid);
  if (commit_valid) {
    reference->consumer_position +=
        static_cast<std::uint32_t>(plan.copy_length);
  }
  RequireState(*state, *reference);
}

void Exercise(const std::uint8_t* data, std::size_t size) {
  FuzzInput input(data, size);
  RxRing state;
  ReferenceState reference{};
  std::array<std::uint8_t, kRingSizeBytes> ring{};
  std::array<std::uint8_t, kRingSizeBytes> output{};
  std::size_t byte_work = 0U;

  for (std::size_t operation = 0U;
       operation < kMaximumOperations && input.remaining() != 0U; ++operation) {
    const std::uint8_t selector = input.ReadU8();
    switch (selector % 6U) {
      case 0U:
        Produce(&input, selector, &state, &reference, &ring, &byte_work);
        break;
      case 1U:
        Read(&input, selector, &state, &reference, ring, &output, &byte_work);
        break;
      case 2U:
        RejectInconsistentSample(&input, &state, reference);
        break;
      case 3U:
        Reset(&input, &state, &reference);
        break;
      case 4U:
        RejectMalformedPlan(&input, selector, &state, &reference, ring,
                            &output);
        break;
      case 5U:
        ReadAcrossProducerProgress(&input, selector, &state, &reference, &ring,
                                   &output, &byte_work);
        break;
      default:
        fuzz::FailInvariant();
    }
    if (byte_work == kMaximumByteWork) {
      break;
    }
  }
}

}  // namespace
}  // namespace mentor_pi::mcu

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  mentor_pi::mcu::Exercise(data, size);
  return 0;
}
