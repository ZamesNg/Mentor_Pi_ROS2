// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/arena_allocator.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "rcutils/allocator.h"
}

#include "mentor_pi_mcu/platform/stm32/memory_regions.h"

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::size_t AlignUp(std::size_t value, std::size_t alignment) {
  return (value + alignment - 1U) & ~(alignment - 1U);
}

}  // namespace

bool ArenaAllocator::Initialize() {
  const auto region = mentor_pi_mcu::platform::stm32::MicroRosArena();
  if (region.data == nullptr || region.size == 0U) {
    return false;
  }
  storage_ = region.data;
  capacity_ = region.size;
  offset_ = 0U;
  mode_ = Mode::kIdle;

  allocator_ = rcutils_get_zero_initialized_allocator();
  allocator_.allocate = &ArenaAllocator::Allocate;
  allocator_.deallocate = &ArenaAllocator::Deallocate;
  allocator_.reallocate = &ArenaAllocator::Reallocate;
  allocator_.zero_allocate = &ArenaAllocator::ZeroAllocate;
  allocator_.state = this;
  return true;
}

bool ArenaAllocator::InstallAsRcutilsDefault() {
  return rcutils_set_default_allocator(&allocator_);
}

void ArenaAllocator::PrepareForCreate() {
  std::memset(storage_, 0, capacity_);
  offset_ = 0U;
  mode_ = Mode::kCreating;
}

void ArenaAllocator::SealActive() { mode_ = Mode::kActiveSealed; }

void ArenaAllocator::BeginDestroy() { mode_ = Mode::kDestroying; }

void ArenaAllocator::ResetAfterDestroy() {
  std::memset(storage_, 0, capacity_);
  offset_ = 0U;
  mode_ = Mode::kIdle;
}

void* ArenaAllocator::Allocate(std::size_t size, void* state) {
  if (state == nullptr) {
    return nullptr;
  }
  return static_cast<ArenaAllocator*>(state)->AllocateImpl(size);
}

void ArenaAllocator::Deallocate(void* pointer, void* state) {
  if (state == nullptr) {
    return;
  }
  static_cast<ArenaAllocator*>(state)->DeallocateImpl(pointer);
}

void* ArenaAllocator::Reallocate(void* pointer, std::size_t size, void* state) {
  if (state == nullptr) {
    return nullptr;
  }
  return static_cast<ArenaAllocator*>(state)->ReallocateImpl(pointer, size);
}

void* ArenaAllocator::ZeroAllocate(std::size_t count, std::size_t size,
                                   void* state) {
  if (state == nullptr || count == 0U || size == 0U ||
      count > std::numeric_limits<std::size_t>::max() / size) {
    return nullptr;
  }
  const std::size_t total = count * size;
  void* const allocation =
      static_cast<ArenaAllocator*>(state)->AllocateImpl(total);
  if (allocation != nullptr) {
    std::memset(allocation, 0, total);
  }
  return allocation;
}

void* ArenaAllocator::AllocateImpl(std::size_t size) {
  if (mode_ != Mode::kCreating || size == 0U) {
    if (mode_ == Mode::kActiveSealed || mode_ == Mode::kDestroying) {
      RecordForbiddenCall();
    }
    return nullptr;
  }

  constexpr std::size_t kAlignment = alignof(std::max_align_t);
  const std::size_t data_offset =
      AlignUp(offset_ + sizeof(BlockHeader), kAlignment);
  if (data_offset > capacity_ || size > capacity_ - data_offset) {
    return nullptr;
  }
  const BlockHeader header{size};
  std::memcpy(storage_ + data_offset - sizeof(BlockHeader), &header,
              sizeof(header));
  offset_ = data_offset + size;
  return storage_ + data_offset;
}

void ArenaAllocator::DeallocateImpl(void* pointer) {
  if (pointer == nullptr) {
    return;
  }
  if (mode_ == Mode::kActiveSealed) {
    RecordForbiddenCall();
    return;
  }
  if (mode_ != Mode::kCreating && mode_ != Mode::kDestroying) {
    return;
  }
  // Individual blocks are intentionally not reclaimed. A complete arena reset
  // after fini restores the exact baseline and prevents fragmentation.
}

void* ArenaAllocator::ReallocateImpl(void* pointer, std::size_t size) {
  if (mode_ != Mode::kCreating) {
    if (mode_ == Mode::kActiveSealed || mode_ == Mode::kDestroying) {
      RecordForbiddenCall();
    }
    return nullptr;
  }
  if (pointer == nullptr) {
    return AllocateImpl(size);
  }
  if (size == 0U) {
    return nullptr;
  }

  auto* const bytes = static_cast<std::uint8_t*>(pointer);
  if (bytes < storage_ + sizeof(BlockHeader) || bytes >= storage_ + capacity_) {
    return nullptr;
  }
  BlockHeader header{};
  std::memcpy(&header, bytes - sizeof(BlockHeader), sizeof(header));
  void* const replacement = AllocateImpl(size);
  if (replacement != nullptr) {
    std::memcpy(replacement, pointer, std::min(header.size, size));
  }
  return replacement;
}

void ArenaAllocator::RecordForbiddenCall() {
  if (post_seal_attempts_ != std::numeric_limits<std::uint32_t>::max()) {
    ++post_seal_attempts_;
  }
  invariant_violated_ = true;
}

}  // namespace mentor_pi_mcu::app::microros
