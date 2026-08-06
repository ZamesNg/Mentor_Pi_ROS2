// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_TRANSPORT_ADAPTER_H_
#define MENTOR_PI_MCU_APP_MICROROS_TRANSPORT_ADAPTER_H_

#include <cstdint>

#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"

namespace mentor_pi_mcu::app::microros {

bool ConfigureCustomTransport();
void CloseCustomTransport();
mentor_pi_mcu::platform::stm32::TransportSnapshot ReadTransportSnapshot();
TeardownReason ClassifyTransportFault(std::uint8_t error_flags);

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_TRANSPORT_ADAPTER_H_
