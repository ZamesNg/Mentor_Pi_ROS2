// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/platform/stm32/transport.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "uxr/client/transport.h"
}

#include "mentor_pi_mcu/domain/circular_dma_position.h"
#include "mentor_pi_mcu/domain/circular_rx_ring.h"
#include "mentor_pi_mcu/platform/stm32/hal_handles.h"
#include "mentor_pi_mcu/platform/stm32/memory_regions.h"
#include "mentor_pi_mcu/platform/stm32/peripherals.h"
#include "mentor_pi_mcu/platform/stm32/platform.h"
#include "mentor_pi_mcu/platform/stm32/task_entries.h"

namespace mentor_pi_mcu::platform::stm32 {
namespace {

static_assert((kUsart1RxDmaRingSizeBytes & (kUsart1RxDmaRingSizeBytes - 1U)) ==
                  0U,
              "USART1 RX DMA ring must be a power of two");
static_assert(kUsart1TxBounceSizeBytes >= kXrceTransportMtuBytes,
              "TX bounce must hold a complete XRCE transport write");

MENTOR_PI_DMA_BUFFER std::uint8_t g_rx_dma_ring[kUsart1RxDmaRingSizeBytes];
MENTOR_PI_DMA_BUFFER std::uint8_t g_tx_bounce[kUsart1TxBounceSizeBytes];

StaticSemaphore_t g_tx_semaphore_storage{};
SemaphoreHandle_t g_tx_semaphore = nullptr;

// HAL's half/full callbacks are the sole incrementing writer while circular RX
// is active. Open resets the epoch only while the RX IRQ is disabled.
volatile std::uint32_t g_rx_dma_boundary_count = 0U;
volatile std::uint32_t g_maximum_wait_us = 0U;
volatile std::uint64_t g_tx_wire_bytes = 0U;
volatile std::size_t g_active_tx_length = 0U;
volatile std::uint8_t g_error_flags = 0U;
volatile bool g_open = false;
volatile bool g_tx_in_progress = false;

std::uint8_t ErrorBit(Usart1Error error) {
  return static_cast<std::uint8_t>(error);
}

using Usart1RxDmaPosition =
    mentor_pi::mcu::CircularDmaPosition<static_cast<std::uint32_t>(
        kUsart1RxDmaRingSizeBytes)>;
using Usart1RxRing = mentor_pi::mcu::CircularRxRing<kUsart1RxDmaRingSizeBytes>;

Usart1RxRing g_rx_state{};

struct DmaProducerSample {
  std::uint32_t position;
  bool consistent;
};

std::uint64_t SaturatingAdd64(std::uint64_t value, std::uint32_t increment) {
  constexpr std::uint64_t kMaximum = std::numeric_limits<std::uint64_t>::max();
  return kMaximum - value < increment ? kMaximum : value + increment;
}

std::uint32_t StableDmaCursor() {
  std::uint32_t previous = __HAL_DMA_GET_COUNTER(&g_dma_usart1_rx);
  for (std::size_t attempt = 0U; attempt < 4U; ++attempt) {
    const std::uint32_t current = __HAL_DMA_GET_COUNTER(&g_dma_usart1_rx);
    if (current == previous) {
      if (current > kUsart1RxDmaRingSizeBytes) {
        return static_cast<std::uint32_t>(kUsart1RxDmaRingSizeBytes + 1U);
      }
      return static_cast<std::uint32_t>(kUsart1RxDmaRingSizeBytes) - current;
    }
    previous = current;
  }
  if (previous > kUsart1RxDmaRingSizeBytes) {
    return static_cast<std::uint32_t>(kUsart1RxDmaRingSizeBytes + 1U);
  }
  return static_cast<std::uint32_t>(kUsart1RxDmaRingSizeBytes) - previous;
}

DmaProducerSample StableDmaProducerPosition() {
  for (std::size_t attempt = 0U; attempt < 8U; ++attempt) {
    const std::uint32_t boundary_before = g_rx_dma_boundary_count;
    const std::uint32_t cursor = StableDmaCursor();
    const std::uint32_t boundary_after = g_rx_dma_boundary_count;
    if (boundary_before == boundary_after &&
        Usart1RxDmaPosition::IsConsistent(boundary_after, cursor)) {
      return {Usart1RxDmaPosition::Reconstruct(boundary_after, cursor), true};
    }
  }
  return {g_rx_state.previous_dma_position(), false};
}

bool UpdateProducerFromTask() {
  DmaProducerSample sample = StableDmaProducerPosition();
  if (!sample.consistent) {
    // A boundary can occur between the epoch and NDTR reads. Leaving task
    // context once lets the normal-priority HAL callback repair the epoch.
    taskYIELD();
    sample = StableDmaProducerPosition();
  }
  if (!sample.consistent) {
    taskENTER_CRITICAL();
    g_error_flags |= ErrorBit(Usart1Error::kDma);
    taskEXIT_CRITICAL();
    EmergencyStopMotors();
    return false;
  }

  const Usart1RxRing::ProducerUpdate update =
      g_rx_state.UpdateProducer(sample.position, true);
  if (update.overrun) {
    taskENTER_CRITICAL();
    g_error_flags |= ErrorBit(Usart1Error::kRxRingOverrun);
    taskEXIT_CRITICAL();
    EmergencyStopMotors();
    return false;
  }
  return true;
}

void LatchTaskError(Usart1Error error) {
  taskENTER_CRITICAL();
  g_error_flags |= ErrorBit(error);
  taskEXIT_CRITICAL();
  EmergencyStopMotors();
}

void UpdateMaximumWait(std::uint32_t start_cycles) {
  const std::uint32_t elapsed_cycles = CycleCounter() - start_cycles;
  const std::uint32_t elapsed_us = elapsed_cycles / 168U;
  taskENTER_CRITICAL();
  if (elapsed_us > g_maximum_wait_us) {
    g_maximum_wait_us = elapsed_us;
  }
  taskEXIT_CRITICAL();
}

Status ReinitializeUsartIfNeeded() {
  if (g_usart1.Instance != USART1) {
    return Status::kNotInitialized;
  }
  if (g_usart1.gState != HAL_UART_STATE_RESET) {
    return Status::kOk;
  }
  return HAL_UART_Init(&g_usart1) == HAL_OK ? Status::kOk : Status::kIoError;
}

void DrainTxCompletionSemaphore() {
  while (xSemaphoreTake(g_tx_semaphore, 0U) == pdTRUE) {
  }
}

}  // namespace

Status OpenUsart1Transport() {
  if (g_open) {
    return Status::kBusy;
  }
  const Status initialize_status = ReinitializeUsartIfNeeded();
  if (initialize_status != Status::kOk) {
    return initialize_status;
  }
  if (g_tx_semaphore == nullptr) {
    g_tx_semaphore = xSemaphoreCreateBinaryStatic(&g_tx_semaphore_storage);
    if (g_tx_semaphore == nullptr) {
      return Status::kIoError;
    }
  }
  DrainTxCompletionSemaphore();

  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  static_cast<void>(HAL_UART_DMAStop(&g_usart1));

  // Clear stale USART error/data state before arming circular reception.
  const std::uint32_t stale_status = g_usart1.Instance->SR;
  volatile const std::uint32_t stale_data = g_usart1.Instance->DR;
  static_cast<void>(stale_status);
  static_cast<void>(stale_data);

  taskENTER_CRITICAL();
  g_rx_state.ResetPositions();
  g_rx_dma_boundary_count = 0U;
  g_active_tx_length = 0U;
  g_error_flags = 0U;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL();

  std::memset(g_rx_dma_ring, 0, sizeof(g_rx_dma_ring));
  if (HAL_UART_Receive_DMA(
          &g_usart1, g_rx_dma_ring,
          static_cast<std::uint16_t>(kUsart1RxDmaRingSizeBytes)) != HAL_OK) {
    LatchTaskError(Usart1Error::kDma);
    return Status::kIoError;
  }

  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  taskENTER_CRITICAL();
  g_open = true;
  taskEXIT_CRITICAL();
  return Status::kOk;
}

void CloseUsart1Transport() {
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  EmergencyStopMotors();

  taskENTER_CRITICAL();
  g_open = false;
  g_active_tx_length = 0U;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL();

  if (g_usart1.Instance != nullptr) {
    static_cast<void>(HAL_UART_DMAStop(&g_usart1));
  }
  if (g_tx_semaphore != nullptr) {
    DrainTxCompletionSemaphore();
  }
}

std::size_t ReadUsart1(std::uint8_t* destination, std::size_t capacity,
                       std::uint32_t timeout_ms, Status* status) {
  if (status == nullptr) {
    return 0U;
  }
  *status = Status::kOk;
  if (destination == nullptr && capacity != 0U) {
    *status = Status::kInvalidArgument;
    return 0U;
  }
  if (!g_open) {
    *status = Status::kNotInitialized;
    return 0U;
  }
  if (capacity == 0U) {
    return 0U;
  }

  const std::uint32_t start_cycles = CycleCounter();
  const std::uint32_t bounded_timeout_ms =
      std::min<std::uint32_t>(timeout_ms, 10U);
  std::uint32_t waited_ms = 0U;

  while (true) {
    if (g_error_flags != 0U || !UpdateProducerFromTask()) {
      *status = Status::kIoError;
      return 0U;
    }
    const Usart1RxRing::ReadPlan plan = g_rx_state.PrepareRead(capacity);
    if (plan.overrun) {
      LatchTaskError(Usart1Error::kRxRingOverrun);
      *status = Status::kOverflow;
      return 0U;
    }
    if (plan.copy_length != 0U) {
      if (!Usart1RxRing::CopyRead(g_rx_dma_ring, plan, destination)) {
        LatchTaskError(Usart1Error::kDma);
        *status = Status::kIoError;
        return 0U;
      }

      // Account for any progress during the copy before committing. The ring
      // helper rejects a stale plan or a full-lap overwrite fail-closed.
      if (!UpdateProducerFromTask()) {
        *status = Status::kIoError;
        return 0U;
      }
      if (!g_rx_state.CommitRead(plan)) {
        LatchTaskError(Usart1Error::kRxRingOverrun);
        *status = Status::kOverflow;
        return 0U;
      }
      UpdateMaximumWait(start_cycles);
      return plan.copy_length;
    }

    if (waited_ms >= bounded_timeout_ms) {
      UpdateMaximumWait(start_cycles);
      return 0U;
    }
    static_cast<void>(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U)));
    ++waited_ms;
  }
}

std::size_t WriteUsart1(const std::uint8_t* source, std::size_t length,
                        Status* status) {
  if (status == nullptr) {
    return 0U;
  }
  *status = Status::kOk;
  if (source == nullptr || length == 0U || length > kUsart1TxBounceSizeBytes) {
    *status = Status::kInvalidArgument;
    if (length > kUsart1TxBounceSizeBytes) {
      LatchTaskError(Usart1Error::kTxDma);
    }
    return 0U;
  }
  if (!g_open) {
    *status = Status::kNotInitialized;
    return 0U;
  }

  taskENTER_CRITICAL();
  const bool already_active = g_tx_in_progress;
  if (!already_active) {
    g_tx_in_progress = true;
    g_active_tx_length = length;
  }
  taskEXIT_CRITICAL();
  if (already_active) {
    *status = Status::kBusy;
    return 0U;
  }

  DrainTxCompletionSemaphore();
  std::memcpy(g_tx_bounce, source, length);
  if (HAL_UART_Transmit_DMA(&g_usart1, g_tx_bounce,
                            static_cast<std::uint16_t>(length)) != HAL_OK) {
    taskENTER_CRITICAL();
    g_active_tx_length = 0U;
    g_tx_in_progress = false;
    taskEXIT_CRITICAL();
    LatchTaskError(Usart1Error::kTxDma);
    *status = Status::kIoError;
    return 0U;
  }

  const std::uint32_t start_cycles = CycleCounter();
  const BaseType_t completed = xSemaphoreTake(
      g_tx_semaphore, pdMS_TO_TICKS(Usart1WriteDeadlineMs(length)));
  UpdateMaximumWait(start_cycles);
  if (completed != pdTRUE) {
    static_cast<void>(HAL_UART_AbortTransmit(&g_usart1));
    taskENTER_CRITICAL();
    g_active_tx_length = 0U;
    g_tx_in_progress = false;
    taskEXIT_CRITICAL();
    LatchTaskError(Usart1Error::kTxTimeout);
    *status = Status::kTimeout;
    return 0U;
  }
  if (g_error_flags != 0U) {
    *status = Status::kIoError;
    return 0U;
  }
  return length;
}

TransportSnapshot GetTransportSnapshot() {
  taskENTER_CRITICAL();
  const TransportSnapshot snapshot{g_rx_state.producer_position(),
                                   g_rx_state.consumer_position(),
                                   g_rx_state.high_water_bytes(),
                                   g_maximum_wait_us,
                                   g_rx_state.rx_wire_bytes(),
                                   g_tx_wire_bytes,
                                   g_error_flags,
                                   g_open};
  taskEXIT_CRITICAL();
  return snapshot;
}

bool TransportHasFatalError() { return g_error_flags != 0U; }

void* Usart1TransportArgument() { return &g_usart1; }

void HandleUsart1RxDmaBoundaryFromIsr() {
  UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  ++g_rx_dma_boundary_count;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  NotifyTaskFromIsr(TaskId::kMicroRos);
}

void HandleUsart1TxCompleteFromIsr() {
  BaseType_t higher_priority_task_woken = pdFALSE;
  UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  g_tx_wire_bytes = SaturatingAdd64(
      g_tx_wire_bytes, static_cast<std::uint32_t>(g_active_tx_length));
  g_active_tx_length = 0U;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  if (g_tx_semaphore != nullptr) {
    xSemaphoreGiveFromISR(g_tx_semaphore, &higher_priority_task_woken);
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HandleUsart1ErrorFromIsr(std::uint32_t hal_error_code) {
  std::uint8_t error_flags = 0U;
  if ((hal_error_code & HAL_UART_ERROR_FE) != 0U) {
    error_flags |= ErrorBit(Usart1Error::kFraming);
  }
  if ((hal_error_code & HAL_UART_ERROR_NE) != 0U) {
    error_flags |= ErrorBit(Usart1Error::kNoise);
  }
  if ((hal_error_code & HAL_UART_ERROR_ORE) != 0U) {
    error_flags |= ErrorBit(Usart1Error::kOverrun);
  }
  if ((hal_error_code & HAL_UART_ERROR_PE) != 0U) {
    error_flags |= ErrorBit(Usart1Error::kParity);
  }
  if ((hal_error_code & HAL_UART_ERROR_DMA) != 0U || error_flags == 0U) {
    error_flags |= ErrorBit(Usart1Error::kDma);
  }

  BaseType_t higher_priority_task_woken = pdFALSE;
  UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  g_error_flags |= error_flags;
  g_active_tx_length = 0U;
  const bool tx_was_active = g_tx_in_progress;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  EmergencyStopMotors();
  if (tx_was_active && g_tx_semaphore != nullptr) {
    xSemaphoreGiveFromISR(g_tx_semaphore, &higher_priority_task_woken);
  }
  NotifyTaskFromIsr(TaskId::kMicroRos);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

}  // namespace mentor_pi_mcu::platform::stm32

extern "C" bool MentorPiTransportOpen(uxrCustomTransport* transport) {
  if (transport == nullptr ||
      transport->args != &mentor_pi_mcu::platform::stm32::g_usart1) {
    return false;
  }
  return mentor_pi_mcu::platform::stm32::OpenUsart1Transport() ==
         mentor_pi_mcu::platform::stm32::Status::kOk;
}

extern "C" bool MentorPiTransportClose(uxrCustomTransport* transport) {
  if (transport == nullptr ||
      transport->args != &mentor_pi_mcu::platform::stm32::g_usart1) {
    return false;
  }
  mentor_pi_mcu::platform::stm32::CloseUsart1Transport();
  return true;
}

extern "C" std::size_t MentorPiTransportWrite(uxrCustomTransport* transport,
                                              const std::uint8_t* buffer,
                                              std::size_t length,
                                              std::uint8_t* error) {
  if (transport == nullptr ||
      transport->args != &mentor_pi_mcu::platform::stm32::g_usart1) {
    if (error != nullptr) {
      *error = 1U;
    }
    return 0U;
  }
  mentor_pi_mcu::platform::stm32::Status status{};
  const std::size_t written =
      mentor_pi_mcu::platform::stm32::WriteUsart1(buffer, length, &status);
  if (error != nullptr) {
    *error = status == mentor_pi_mcu::platform::stm32::Status::kOk ? 0U : 1U;
  }
  return written;
}

extern "C" std::size_t MentorPiTransportRead(uxrCustomTransport* transport,
                                             std::uint8_t* buffer,
                                             std::size_t length, int timeout_ms,
                                             std::uint8_t* error) {
  if (transport == nullptr ||
      transport->args != &mentor_pi_mcu::platform::stm32::g_usart1) {
    if (error != nullptr) {
      *error = 1U;
    }
    return 0U;
  }
  const std::uint32_t bounded_timeout_ms =
      timeout_ms > 0 ? static_cast<std::uint32_t>(timeout_ms) : 0U;
  mentor_pi_mcu::platform::stm32::Status status{};
  const std::size_t read = mentor_pi_mcu::platform::stm32::ReadUsart1(
      buffer, length, bounded_timeout_ms, &status);
  if (error != nullptr) {
    *error = status == mentor_pi_mcu::platform::stm32::Status::kOk ? 0U : 1U;
  }
  return read;
}
