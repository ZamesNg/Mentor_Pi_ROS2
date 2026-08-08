// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/reclaiming_arena.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>

namespace mentor_pi_mcu::app::microros {

bool ReclaimingArena::Initialize(void* storage, std::size_t capacity) {
  const auto address = reinterpret_cast<std::uintptr_t>(storage);
  if (storage == nullptr || address % kAlignment != 0U ||
      capacity < sizeof(BlockHeader) + kAlignment) {
    return false;
  }
  storage_ = static_cast<std::uint8_t*>(storage);
  capacity_ = capacity & ~(kAlignment - 1U);
  Reset();
  return healthy_;
}

void ReclaimingArena::Reset() {
  if (storage_ == nullptr || capacity_ < sizeof(BlockHeader) + kAlignment) {
    healthy_ = false;
    return;
  }
  std::memset(storage_, 0, capacity_);
  new (storage_) BlockHeader{capacity_ - sizeof(BlockHeader), kNoOffset,
                             kBlockMagic, kBlockFree};
  bytes_used_ = 0U;
  peak_bytes_used_ = 0U;
  healthy_ = true;
}

void* ReclaimingArena::Allocate(std::size_t size) {
  if (!healthy_ || size == 0U ||
      size > std::numeric_limits<std::size_t>::max() - (kAlignment - 1U)) {
    return nullptr;
  }
  const std::size_t aligned_size = AlignUp(size);
  std::size_t offset = 0U;
  while (offset < capacity_) {
    if (!ValidateBlock(offset)) {
      MarkCorrupt();
      return nullptr;
    }
    BlockHeader* const block = HeaderAt(offset);
    if (block->state == kBlockFree && block->payload_size >= aligned_size) {
      SplitBlock(offset, aligned_size);
      if (!healthy_) {
        return nullptr;
      }
      block->state = kBlockAllocated;
      bytes_used_ += block->payload_size;
      peak_bytes_used_ = std::max(peak_bytes_used_, bytes_used_);
      return storage_ + offset + sizeof(BlockHeader);
    }
    offset = NextOffset(offset, *block);
  }
  return nullptr;
}

bool ReclaimingArena::Deallocate(void* pointer) {
  if (pointer == nullptr) {
    return true;
  }
  std::size_t offset = 0U;
  if (!healthy_ || !FindBlockForPointer(pointer, &offset)) {
    MarkCorrupt();
    return false;
  }
  BlockHeader* block = HeaderAt(offset);
  if (block->state != kBlockAllocated || block->payload_size > bytes_used_) {
    MarkCorrupt();
    return false;
  }
  bytes_used_ -= block->payload_size;
  block->state = kBlockFree;

  MergeWithNext(offset);
  if (!healthy_) {
    return false;
  }
  block = HeaderAt(offset);
  if (block->previous_offset != kNoOffset) {
    const std::size_t previous_offset = block->previous_offset;
    if (!ValidateBlock(previous_offset)) {
      MarkCorrupt();
      return false;
    }
    BlockHeader* const previous = HeaderAt(previous_offset);
    if (previous->state == kBlockFree) {
      MergeWithNext(previous_offset);
    }
  }
  return healthy_;
}

void* ReclaimingArena::Reallocate(void* pointer, std::size_t size) {
  if (pointer == nullptr) {
    return Allocate(size);
  }
  if (size == 0U) {
    static_cast<void>(Deallocate(pointer));
    return nullptr;
  }
  if (size > std::numeric_limits<std::size_t>::max() - (kAlignment - 1U)) {
    return nullptr;
  }

  std::size_t offset = 0U;
  if (!healthy_ || !FindBlockForPointer(pointer, &offset)) {
    MarkCorrupt();
    return nullptr;
  }
  BlockHeader* block = HeaderAt(offset);
  if (block->state != kBlockAllocated) {
    MarkCorrupt();
    return nullptr;
  }
  const std::size_t old_size = block->payload_size;
  const std::size_t aligned_size = AlignUp(size);
  if (aligned_size <= old_size) {
    SplitBlock(offset, aligned_size);
    if (healthy_) {
      block = HeaderAt(offset);
      bytes_used_ -= old_size - block->payload_size;
    }
    return healthy_ ? pointer : nullptr;
  }

  const std::size_t next_offset = NextOffset(offset, *block);
  if (next_offset < capacity_ && ValidateBlock(next_offset) &&
      HeaderAt(next_offset)->state == kBlockFree &&
      old_size + sizeof(BlockHeader) + HeaderAt(next_offset)->payload_size >=
          aligned_size) {
    MergeWithNext(offset);
    if (!healthy_) {
      return nullptr;
    }
    SplitBlock(offset, aligned_size);
    if (!healthy_) {
      return nullptr;
    }
    block = HeaderAt(offset);
    bytes_used_ += block->payload_size - old_size;
    peak_bytes_used_ = std::max(peak_bytes_used_, bytes_used_);
    return pointer;
  }

  void* const replacement = Allocate(size);
  if (replacement == nullptr) {
    return nullptr;
  }
  std::memcpy(replacement, pointer, std::min(old_size, size));
  if (!Deallocate(pointer)) {
    return nullptr;
  }
  return replacement;
}

void* ReclaimingArena::ZeroAllocate(std::size_t count, std::size_t size) {
  if (count == 0U || size == 0U ||
      count > std::numeric_limits<std::size_t>::max() / size) {
    return nullptr;
  }
  const std::size_t total = count * size;
  void* const allocation = Allocate(total);
  if (allocation != nullptr) {
    std::memset(allocation, 0, total);
  }
  return allocation;
}

ReclaimingArena::BlockHeader* ReclaimingArena::HeaderAt(std::size_t offset) {
  return static_cast<BlockHeader*>(
      __builtin_assume_aligned(storage_ + offset, kAlignment));
}

const ReclaimingArena::BlockHeader* ReclaimingArena::HeaderAt(
    std::size_t offset) const {
  return static_cast<const BlockHeader*>(
      __builtin_assume_aligned(storage_ + offset, kAlignment));
}

bool ReclaimingArena::ValidateBlock(std::size_t offset) const {
  if (!healthy_ || offset % kAlignment != 0U ||
      offset > capacity_ - sizeof(BlockHeader)) {
    return false;
  }
  const BlockHeader* const block = HeaderAt(offset);
  if (block->magic != kBlockMagic ||
      (block->state != kBlockFree && block->state != kBlockAllocated) ||
      block->payload_size % kAlignment != 0U ||
      block->payload_size > capacity_ - offset - sizeof(BlockHeader)) {
    return false;
  }
  const std::size_t next_offset = NextOffset(offset, *block);
  return next_offset == capacity_ ||
         next_offset <= capacity_ - sizeof(BlockHeader);
}

bool ReclaimingArena::FindBlockForPointer(void* pointer, std::size_t* offset) {
  if (pointer == nullptr || offset == nullptr || storage_ == nullptr) {
    return false;
  }
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  const auto begin = reinterpret_cast<std::uintptr_t>(storage_);
  if (address < begin + sizeof(BlockHeader) || address >= begin + capacity_) {
    return false;
  }
  const std::size_t candidate =
      static_cast<std::size_t>(address - begin) - sizeof(BlockHeader);
  if (!ValidateBlock(candidate) ||
      storage_ + candidate + sizeof(BlockHeader) != pointer) {
    return false;
  }
  *offset = candidate;
  return true;
}

std::size_t ReclaimingArena::NextOffset(std::size_t offset,
                                        const BlockHeader& header) const {
  return offset + sizeof(BlockHeader) + header.payload_size;
}

void ReclaimingArena::SetFollowingPrevious(std::size_t offset) {
  if (!ValidateBlock(offset)) {
    MarkCorrupt();
    return;
  }
  const std::size_t next_offset = NextOffset(offset, *HeaderAt(offset));
  if (next_offset == capacity_) {
    return;
  }
  if (!ValidateBlock(next_offset)) {
    MarkCorrupt();
    return;
  }
  HeaderAt(next_offset)->previous_offset = offset;
}

void ReclaimingArena::SplitBlock(std::size_t offset, std::size_t payload_size) {
  if (!ValidateBlock(offset)) {
    MarkCorrupt();
    return;
  }
  BlockHeader* const block = HeaderAt(offset);
  if (payload_size > block->payload_size) {
    MarkCorrupt();
    return;
  }
  const std::size_t remaining = block->payload_size - payload_size;
  if (remaining < sizeof(BlockHeader) + kAlignment) {
    return;
  }
  const std::size_t new_offset = offset + sizeof(BlockHeader) + payload_size;
  new (storage_ + new_offset) BlockHeader{remaining - sizeof(BlockHeader),
                                          offset, kBlockMagic, kBlockFree};
  block->payload_size = payload_size;
  SetFollowingPrevious(new_offset);
}

void ReclaimingArena::MergeWithNext(std::size_t offset) {
  if (!ValidateBlock(offset)) {
    MarkCorrupt();
    return;
  }
  BlockHeader* const block = HeaderAt(offset);
  const std::size_t next_offset = NextOffset(offset, *block);
  if (next_offset == capacity_) {
    return;
  }
  if (!ValidateBlock(next_offset)) {
    MarkCorrupt();
    return;
  }
  const BlockHeader* const next = HeaderAt(next_offset);
  if (next->state != kBlockFree) {
    return;
  }
  block->payload_size += sizeof(BlockHeader) + next->payload_size;
  SetFollowingPrevious(offset);
}

void ReclaimingArena::MarkCorrupt() { healthy_ = false; }

}  // namespace mentor_pi_mcu::app::microros
