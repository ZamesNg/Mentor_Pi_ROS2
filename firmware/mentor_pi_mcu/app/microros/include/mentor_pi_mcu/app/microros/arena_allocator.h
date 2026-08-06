// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_ARENA_ALLOCATOR_H_
#define MENTOR_PI_MCU_APP_MICROROS_ARENA_ALLOCATOR_H_

#include <cstddef>
#include <cstdint>

extern "C" {
#include "rcl/allocator.h"
}

namespace mentor_pi_mcu::app::microros {

class ArenaAllocator {
 public:
  enum class Mode : std::uint8_t {
    kIdle = 0,
    kCreating = 1,
    kActiveSealed = 2,
    kDestroying = 3,
  };

  bool Initialize();
  bool InstallAsRcutilsDefault();
  void PrepareForCreate();
  void SealActive();
  void BeginDestroy();
  void ResetAfterDestroy();

  rcl_allocator_t allocator() const { return allocator_; }
  Mode mode() const { return mode_; }
  std::size_t bytes_used() const { return offset_; }
  std::size_t capacity() const { return capacity_; }
  std::uint32_t post_seal_attempts() const { return post_seal_attempts_; }
  bool invariant_violated() const { return invariant_violated_; }

 private:
  struct BlockHeader {
    std::size_t size;
  };

  static void* Allocate(std::size_t size, void* state);
  static void Deallocate(void* pointer, void* state);
  static void* Reallocate(void* pointer, std::size_t size, void* state);
  static void* ZeroAllocate(std::size_t count, std::size_t size, void* state);

  void* AllocateImpl(std::size_t size);
  void DeallocateImpl(void* pointer);
  void* ReallocateImpl(void* pointer, std::size_t size);
  void RecordForbiddenCall();

  std::uint8_t* storage_{nullptr};
  std::size_t capacity_{0U};
  std::size_t offset_{0U};
  rcl_allocator_t allocator_{};
  Mode mode_{Mode::kIdle};
  std::uint32_t post_seal_attempts_{0U};
  bool invariant_violated_{false};
};

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_ARENA_ALLOCATOR_H_
