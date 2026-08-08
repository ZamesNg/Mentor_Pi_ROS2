// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/arena_allocator.h"

#include <cstddef>
#include <cstdint>
#include <limits>

extern "C" {
#include "rcutils/allocator.h"
}

#include "mentor_pi_mcu/platform/stm32/memory_regions.h"

namespace mentor_pi_mcu::app::microros {
namespace {

// rcutils_set_default_allocator() retains the callbacks but clears state in
// the pinned version. Firmware owns exactly one arena, so default-allocator
// calls resolve that instance here.
ArenaAllocator* g_allocator_instance = nullptr;

ArenaAllocator* ResolveAllocator(void* state) {
  return state != nullptr ? static_cast<ArenaAllocator*>(state)
                          : g_allocator_instance;
}

}  // namespace

bool ArenaAllocator::Initialize() {
  const auto region = mentor_pi_mcu::platform::stm32::MicroRosArena();
  if (!arena_.Initialize(region.data, region.size)) {
    return false;
  }
  allocator_ = rcutils_get_zero_initialized_allocator();
  allocator_.allocate = &ArenaAllocator::Allocate;
  allocator_.deallocate = &ArenaAllocator::Deallocate;
  allocator_.reallocate = &ArenaAllocator::Reallocate;
  allocator_.zero_allocate = &ArenaAllocator::ZeroAllocate;
  allocator_.state = this;
  g_allocator_instance = this;
  mode_ = Mode::kIdle;
  post_seal_attempts_ = 0U;
  invariant_violated_ = false;
  return true;
}

bool ArenaAllocator::InstallAsRcutilsDefault() {
  return rcutils_set_default_allocator(&allocator_);
}

void ArenaAllocator::PrepareForCreate() {
  arena_.Reset();
  mode_ = Mode::kCreating;
}

void ArenaAllocator::SealActive() { mode_ = Mode::kActiveSealed; }

void ArenaAllocator::BeginDestroy() { mode_ = Mode::kDestroying; }

void ArenaAllocator::ResetAfterDestroy() {
  arena_.Reset();
  mode_ = Mode::kIdle;
}

void* ArenaAllocator::Allocate(std::size_t size, void* state) {
  ArenaAllocator* const allocator = ResolveAllocator(state);
  return allocator != nullptr ? allocator->AllocateImpl(size) : nullptr;
}

void ArenaAllocator::Deallocate(void* pointer, void* state) {
  ArenaAllocator* const allocator = ResolveAllocator(state);
  if (allocator != nullptr) {
    allocator->DeallocateImpl(pointer);
  }
}

void* ArenaAllocator::Reallocate(void* pointer, std::size_t size, void* state) {
  ArenaAllocator* const allocator = ResolveAllocator(state);
  return allocator != nullptr ? allocator->ReallocateImpl(pointer, size)
                              : nullptr;
}

void* ArenaAllocator::ZeroAllocate(std::size_t count, std::size_t size,
                                   void* state) {
  ArenaAllocator* const allocator = ResolveAllocator(state);
  if (allocator == nullptr) {
    return nullptr;
  }
  if (allocator->mode_ != Mode::kCreating) {
    if (allocator->mode_ == Mode::kActiveSealed ||
        allocator->mode_ == Mode::kDestroying) {
      allocator->RecordForbiddenCall();
    }
    return nullptr;
  }
  if (count == 0U || size == 0U ||
      count > std::numeric_limits<std::size_t>::max() / size) {
    return nullptr;
  }
  void* const allocation = allocator->arena_.ZeroAllocate(count, size);
  if (!allocator->arena_.healthy()) {
    allocator->invariant_violated_ = true;
  }
  return allocation;
}

void* ArenaAllocator::AllocateImpl(std::size_t size) {
  if (mode_ != Mode::kCreating) {
    if (mode_ == Mode::kActiveSealed || mode_ == Mode::kDestroying) {
      RecordForbiddenCall();
    }
    return nullptr;
  }
  void* const allocation = arena_.Allocate(size);
  if (!arena_.healthy()) {
    invariant_violated_ = true;
  }
  return allocation;
}

void ArenaAllocator::DeallocateImpl(void* pointer) {
  if (mode_ == Mode::kActiveSealed) {
    RecordForbiddenCall();
    return;
  }
  if (pointer == nullptr) {
    return;
  }
  if (mode_ != Mode::kCreating && mode_ != Mode::kDestroying) {
    return;
  }
  if (!arena_.Deallocate(pointer)) {
    invariant_violated_ = true;
  }
}

void* ArenaAllocator::ReallocateImpl(void* pointer, std::size_t size) {
  if (mode_ != Mode::kCreating) {
    if (mode_ == Mode::kActiveSealed || mode_ == Mode::kDestroying) {
      RecordForbiddenCall();
    }
    return nullptr;
  }
  void* const allocation = arena_.Reallocate(pointer, size);
  if (!arena_.healthy()) {
    invariant_violated_ = true;
  }
  return allocation;
}

void ArenaAllocator::RecordForbiddenCall() {
  if (post_seal_attempts_ != std::numeric_limits<std::uint32_t>::max()) {
    ++post_seal_attempts_;
  }
  invariant_violated_ = true;
}

}  // namespace mentor_pi_mcu::app::microros
