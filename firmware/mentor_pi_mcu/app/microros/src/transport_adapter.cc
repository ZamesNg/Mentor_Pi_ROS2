// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/transport_adapter.h"

#include <cstdint>

extern "C" {
#include "rmw_microros/rmw_microros.h"
}

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::uint8_t ErrorBit(
    mentor_pi_mcu::platform::stm32::Usart1Error error) {
  return static_cast<std::uint8_t>(error);
}

using mentor_pi_mcu::platform::stm32::Usart1Error;
static_assert(ErrorBit(Usart1Error::kFraming) == kTransportFramingError);
static_assert(ErrorBit(Usart1Error::kNoise) == kTransportNoiseError);
static_assert(ErrorBit(Usart1Error::kOverrun) == kTransportOverrunError);
static_assert(ErrorBit(Usart1Error::kParity) == kTransportParityError);
static_assert(ErrorBit(Usart1Error::kTxDma) == kTransportTxDmaError);
static_assert(ErrorBit(Usart1Error::kTxTimeout) == kTransportTxTimeoutError);
static_assert(ErrorBit(Usart1Error::kRxRingOverrun) ==
              kTransportRxRingOverrunError);
static_assert(ErrorBit(Usart1Error::kDma) == kTransportDmaError);

}  // namespace

bool ConfigureCustomTransport() {
  return rmw_uros_set_custom_transport(
             MICROROS_TRANSPORTS_FRAMING_MODE, nullptr, &MentorPiTransportOpen,
             &MentorPiTransportClose, &MentorPiTransportWrite,
             &MentorPiTransportRead) == RMW_RET_OK;
}

void CloseCustomTransport() {
  if (mentor_pi_mcu::platform::stm32::GetTransportSnapshot().open) {
    mentor_pi_mcu::platform::stm32::CloseUsart1Transport();
  }
}

mentor_pi_mcu::platform::stm32::TransportSnapshot ReadTransportSnapshot() {
  return mentor_pi_mcu::platform::stm32::GetTransportSnapshot();
}

TeardownReason ClassifyTransportFault(std::uint8_t error_flags) {
  return ClassifyTransportErrorFlags(error_flags);
}

}  // namespace mentor_pi_mcu::app::microros
