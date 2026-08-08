// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_RECLAIMING_ARENA_H_
#define MENTOR_PI_MCU_APP_MICROROS_RECLAIMING_ARENA_H_

#include <cstddef>
#include <cstdint>

namespace mentor_pi_mcu::app::microros {

// A fixed-storage, first-fit allocator for the bounded micro-ROS construction
// phase. Metadata lives inside the supplied region; adjacent free blocks are
// coalesced, and Reset() restores the exact single-free-block baseline.
class ReclaimingArena {
 public:
  bool Initialize(void* storage, std::size_t capacity);
  void Reset();

  void* Allocate(std::size_t size);
  bool Deallocate(void* pointer);
  void* Reallocate(void* pointer, std::size_t size);
  void* ZeroAllocate(std::size_t count, std::size_t size);

  std::size_t bytes_used() const { return bytes_used_; }
  std::size_t peak_bytes_used() const { return peak_bytes_used_; }
  std::size_t capacity() const { return capacity_; }
  bool healthy() const { return healthy_; }

 private:
  struct alignas(std::max_align_t) BlockHeader {
    std::size_t payload_size;
    std::size_t previous_offset;
    std::uint32_t magic;
    std::uint32_t state;
  };
  static_assert(sizeof(BlockHeader) % alignof(std::max_align_t) == 0U,
                "arena headers must preserve payload alignment");

  static constexpr std::size_t kNoOffset = static_cast<std::size_t>(-1);
  static constexpr std::uint32_t kBlockMagic = 0x4D504152U;
  static constexpr std::uint32_t kBlockFree = 0x46524545U;
  static constexpr std::uint32_t kBlockAllocated = 0x55534544U;
  static constexpr std::size_t kAlignment = alignof(std::max_align_t);

  static constexpr std::size_t AlignUp(std::size_t value) {
    return (value + kAlignment - 1U) & ~(kAlignment - 1U);
  }

  BlockHeader* HeaderAt(std::size_t offset);
  const BlockHeader* HeaderAt(std::size_t offset) const;
  bool ValidateBlock(std::size_t offset) const;
  bool FindBlockForPointer(void* pointer, std::size_t* offset);
  std::size_t NextOffset(std::size_t offset, const BlockHeader& header) const;
  void SetFollowingPrevious(std::size_t offset);
  void SplitBlock(std::size_t offset, std::size_t payload_size);
  void MergeWithNext(std::size_t offset);
  void MarkCorrupt();

  std::uint8_t* storage_{nullptr};
  std::size_t capacity_{0U};
  std::size_t bytes_used_{0U};
  std::size_t peak_bytes_used_{0U};
  bool healthy_{false};
};

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_RECLAIMING_ARENA_H_
