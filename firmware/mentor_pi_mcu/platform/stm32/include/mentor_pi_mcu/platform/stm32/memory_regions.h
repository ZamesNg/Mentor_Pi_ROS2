// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_MEMORY_REGIONS_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_MEMORY_REGIONS_H_

#include <cstddef>
#include <cstdint>

#if !defined(__GNUC__)
#error "RRCLite memory-section attributes require the pinned Arm GNU compiler"
#endif

// DMA2 cannot access CCM at 0x10000000. Every DMA source or destination must
// carry MENTOR_PI_DMA_BUFFER and therefore resolve into .dma_buffer in SRAM.
#define MENTOR_PI_DMA_BUFFER \
  __attribute__((section(".dma_buffer"), aligned(32)))

// CPU-only state may be placed in CCM. Never apply this to a DMA object.
#define MENTOR_PI_CCM_BUFFER __attribute__((section(".ccmram"), aligned(8)))

namespace mentor_pi_mcu::platform::stm32 {

inline constexpr std::size_t kMicroRosArenaSizeBytes = std::size_t{48U} * 1024U;

struct MutableMemoryRegion {
  std::uint8_t* data;
  std::size_t size;
};

// Returns the single resettable arena reserved for micro-ROS entity creation.
// The caller owns allocation sealing and whole-arena reset semantics.
MutableMemoryRegion MicroRosArena();

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_MEMORY_REGIONS_H_
