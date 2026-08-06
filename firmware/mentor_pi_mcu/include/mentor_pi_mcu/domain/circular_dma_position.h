// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_DOMAIN_CIRCULAR_DMA_POSITION_H_
#define MENTOR_PI_MCU_DOMAIN_CIRCULAR_DMA_POSITION_H_

#include <cstdint>

namespace mentor_pi::mcu {

// Reconstructs a modulo-2^32 producer position from a circular DMA cursor and
// a top-half counter that advances at every half- and full-buffer boundary.
// The boundary counter must have exactly one incrementing writer while DMA is
// active and may be reset only while that writer is disabled. This type owns no
// state, performs no allocation, and is safe to use from target snapshot code.
template <std::uint32_t kRingSizeBytes>
class CircularDmaPosition {
 public:
  static_assert(kRingSizeBytes >= 2U, "DMA ring must have two halves");
  static_assert((kRingSizeBytes & (kRingSizeBytes - 1U)) == 0U,
                "DMA ring must be a power of two");

  static constexpr std::uint32_t kHalfSizeBytes = kRingSizeBytes / 2U;

  // A boundary count parity of zero identifies the first half of the ring;
  // odd parity identifies the second half. A mismatch means that DMA crossed
  // a boundary while the snapshot was being taken and the caller must retry.
  static constexpr bool IsConsistent(std::uint32_t boundary_count,
                                     std::uint32_t cursor) {
    if (cursor >= kRingSizeBytes) {
      return false;
    }
    const std::uint32_t cursor_half = cursor / kHalfSizeBytes;
    return (boundary_count & 1U) == cursor_half;
  }

  // Call only for a consistent snapshot. Unsigned overflow is intentional: it
  // preserves producer-consumer differences across the 32-bit position wrap.
  static constexpr std::uint32_t Reconstruct(std::uint32_t boundary_count,
                                             std::uint32_t cursor) {
    return ((boundary_count / 2U) * kRingSizeBytes) + cursor;
  }
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_CIRCULAR_DMA_POSITION_H_
