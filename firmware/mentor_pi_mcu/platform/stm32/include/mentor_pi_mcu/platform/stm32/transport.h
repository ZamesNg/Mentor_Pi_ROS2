// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_TRANSPORT_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_TRANSPORT_H_

#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/platform/stm32/status.h"

struct uxrCustomTransport;

namespace mentor_pi_mcu::platform::stm32 {

inline constexpr std::size_t kUsart1RxRingSizeBytes = std::size_t{8U} * 1024U;
inline constexpr std::size_t kUsart1TxBounceSizeBytes = 1024U;
inline constexpr std::size_t kXrceTransportMtuBytes = 512U;
inline constexpr std::uint32_t kUsart1Baud = 1000000U;

enum class Usart1Error : std::uint8_t {
  kNone = 0,
  kFraming = 1U << 0,
  kNoise = 1U << 1,
  kOverrun = 1U << 2,
  kParity = 1U << 3,
  kTxDma = 1U << 4,
  kTxTimeout = 1U << 5,
  kRxRingOverrun = 1U << 6,
  kDma = 1U << 7,
};

struct TransportSnapshot {
  std::uint32_t producer_position;
  std::uint32_t consumer_position;
  std::uint32_t rx_high_water_bytes;
  std::uint32_t maximum_wait_us;
  std::uint64_t rx_wire_bytes;
  std::uint64_t tx_wire_bytes;
  std::uint8_t error_flags;
  bool open;
};

Status OpenUsart1Transport();
void CloseUsart1Transport();

// Read blocks for at most min(timeout_ms, 10 ms) and never drops an overrun.
std::size_t ReadUsart1(std::uint8_t* destination, std::size_t capacity,
                       std::uint32_t timeout_ms, Status* status);

// Write copies into the DMA bounce buffer and permits no queued transfer.
std::size_t WriteUsart1(const std::uint8_t* source, std::size_t length,
                        Status* status);

TransportSnapshot GetTransportSnapshot();
bool TransportHasFatalError();

// The RX DMA top half runs above the FreeRTOS syscall ceiling. It only clears
// hardware flags, advances a single-writer boundary counter, latches a sticky
// error bit, and pends USART1 for deferred FreeRTOS-safe processing.
void HandleUsart1RxDmaTopHalfFromIsr();

// FreeRTOS-callable ISR delegates. These only update bounded state and notify
// MicroRosTask.
void HandleUsart1Irq();
void HandleUsart1RxDmaProgressFromIsr();
void HandleUsart1TxCompleteFromIsr();
void HandleUsart1DmaErrorFromIsr();

}  // namespace mentor_pi_mcu::platform::stm32

// Exact micro-ROS custom-transport callback signatures. The implementation is
// independent of the transport object's argument and uses the dedicated USART1.
extern "C" bool MentorPiTransportOpen(uxrCustomTransport* transport);
extern "C" bool MentorPiTransportClose(uxrCustomTransport* transport);
extern "C" std::size_t MentorPiTransportWrite(uxrCustomTransport* transport,
                                              const std::uint8_t* buffer,
                                              std::size_t length,
                                              std::uint8_t* error);
extern "C" std::size_t MentorPiTransportRead(uxrCustomTransport* transport,
                                             std::uint8_t* buffer,
                                             std::size_t length, int timeout_ms,
                                             std::uint8_t* error);

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_TRANSPORT_H_
