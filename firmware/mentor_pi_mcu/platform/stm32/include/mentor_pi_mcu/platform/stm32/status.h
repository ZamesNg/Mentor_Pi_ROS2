// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_STATUS_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_STATUS_H_

#include <cstdint>

namespace mentor_pi_mcu::platform::stm32 {

enum class Status : std::uint8_t {
  kOk = 0,
  kInvalidArgument,
  kNotInitialized,
  kBusy,
  kTimeout,
  kIoError,
  kOverflow,
};

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_STATUS_H_
