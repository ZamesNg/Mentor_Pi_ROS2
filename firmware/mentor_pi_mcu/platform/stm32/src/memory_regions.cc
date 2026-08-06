// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/memory_regions.h"

#include <cstddef>
#include <cstdint>

namespace mentor_pi_mcu::platform::stm32 {
namespace {

MENTOR_PI_CCM_BUFFER std::uint8_t g_micro_ros_arena[kMicroRosArenaSizeBytes];

}  // namespace

MutableMemoryRegion MicroRosArena() {
  return MutableMemoryRegion{g_micro_ros_arena, sizeof(g_micro_ros_arena)};
}

}  // namespace mentor_pi_mcu::platform::stm32
