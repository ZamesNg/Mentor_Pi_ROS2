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

static_assert((kUsart1RxRingSizeBytes & (kUsart1RxRingSizeBytes - 1U)) == 0U,
              "USART1 RX ring must be a power of two");
static_assert(kUsart1TxBounceSizeBytes >= kXrceTransportMtuBytes,
              "TX bounce must hold a complete XRCE transport write");

MENTOR_PI_DMA_BUFFER std::uint8_t g_rx_ring[kUsart1RxRingSizeBytes];
MENTOR_PI_DMA_BUFFER std::uint8_t g_tx_bounce[kUsart1TxBounceSizeBytes];

StaticSemaphore_t g_tx_semaphore_storage{};
SemaphoreHandle_t g_tx_semaphore = nullptr;

// DMA2 Stream 2 is the sole incrementing writer while a session is open. Open
// resets this state only with that IRQ disabled. Its priority is deliberately
// above the FreeRTOS syscall ceiling, so it never touches RTOS state or shared
// RMW flags.
volatile std::uint32_t g_rx_dma_boundary_count = 0U;
volatile bool g_rx_dma_top_half_error = false;
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
        kUsart1RxRingSizeBytes)>;
using Usart1RxRing = mentor_pi::mcu::CircularRxRing<kUsart1RxRingSizeBytes>;

Usart1RxRing g_rx_state{};

struct DmaProducerSample {
  std::uint32_t position;
  bool consistent;
};

std::uint64_t SaturatingAdd64(std::uint64_t value, std::uint32_t increment) {
  constexpr std::uint64_t kMaximum = std::numeric_limits<std::uint64_t>::max();
  if (kMaximum - value < increment) {
    return kMaximum;
  }
  return value + increment;
}

std::uint32_t StableDmaCursor() {
  std::uint32_t previous = __HAL_DMA_GET_COUNTER(&g_dma_usart1_rx);
  for (std::size_t attempt = 0; attempt < 4U; ++attempt) {
    const std::uint32_t current = __HAL_DMA_GET_COUNTER(&g_dma_usart1_rx);
    if (current == previous) {
      return static_cast<std::uint32_t>(kUsart1RxRingSizeBytes) - current;
    }
    previous = current;
  }
  return static_cast<std::uint32_t>(kUsart1RxRingSizeBytes) - previous;
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

void UpdateProducerLocked() {
  const DmaProducerSample sample = StableDmaProducerPosition();
  const Usart1RxRing::ProducerUpdate update =
      g_rx_state.UpdateProducer(sample.position, sample.consistent);
  if (!update.consistent) {
    g_error_flags |= ErrorBit(Usart1Error::kDma);
    return;
  }
  if (update.overrun) {
    g_error_flags |= ErrorBit(Usart1Error::kRxRingOverrun);
  }
}

void UpdateProducerFromTask() {
  taskENTER_CRITICAL();
  UpdateProducerLocked();
  taskEXIT_CRITICAL();
  if (g_error_flags != 0U || g_rx_dma_top_half_error) {
    EmergencyStopMotors();
  }
}

void LatchTaskError(Usart1Error error) {
  taskENTER_CRITICAL();
  g_error_flags |= ErrorBit(error);
  taskEXIT_CRITICAL();
  EmergencyStopMotors();
}

std::uint32_t WriteDeadlineMs(std::size_t length) {
  // ceil(10 bits * bytes * 1000 / 1,000,000 baud) + 2 ms.
  const std::size_t serial_time_ms = (length + 99U) / 100U;
  return static_cast<std::uint32_t>(serial_time_ms + 2U);
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
  while (xSemaphoreTake(g_tx_semaphore, 0U) == pdTRUE) {
  }

  // This IRQ is above BASEPRI and therefore is not masked by a FreeRTOS task
  // critical section. Disable it explicitly before resetting its single-writer
  // epoch state.
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  taskENTER_CRITICAL();
  g_rx_state.ResetPositions();
  g_rx_dma_boundary_count = 0U;
  g_rx_dma_top_half_error = false;
  g_active_tx_length = 0U;
  g_error_flags = 0U;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL();

  if (HAL_UART_Receive_DMA(&g_usart1, g_rx_ring,
                           static_cast<std::uint16_t>(sizeof(g_rx_ring))) !=
      HAL_OK) {
    return Status::kIoError;
  }
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  __HAL_UART_ENABLE_IT(&g_usart1, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&g_usart1, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(&g_usart1, UART_IT_PE);
  g_open = true;
  return Status::kOk;
}

void CloseUsart1Transport() {
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
  EmergencyStopMotors();
  taskENTER_CRITICAL();
  g_open = false;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL();

  if (g_usart1.Instance == nullptr) {
    return;
  }
  __HAL_UART_DISABLE_IT(&g_usart1, UART_IT_IDLE);
  __HAL_UART_DISABLE_IT(&g_usart1, UART_IT_ERR);
  __HAL_UART_DISABLE_IT(&g_usart1, UART_IT_PE);
  static_cast<void>(HAL_UART_DMAStop(&g_usart1));
  static_cast<void>(HAL_UART_DeInit(&g_usart1));
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
  UpdateProducerFromTask();
  std::uint32_t available =
      g_rx_state.producer_position() - g_rx_state.consumer_position();
  if (available == 0U && timeout_ms != 0U) {
    const std::uint32_t bounded_timeout_ms =
        std::min<std::uint32_t>(timeout_ms, 10U);
    static_cast<void>(
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(bounded_timeout_ms)));
    UpdateProducerFromTask();
    available = g_rx_state.producer_position() - g_rx_state.consumer_position();
  }
  UpdateMaximumWait(start_cycles);

  const Usart1RxRing::ReadPlan plan = g_rx_state.PrepareRead(capacity);
  if (plan.overrun) {
    *status = Status::kOverflow;
    return 0U;
  }
  if (!Usart1RxRing::CopyRead(g_rx_ring, plan, destination)) {
    LatchTaskError(Usart1Error::kDma);
    *status = Status::kIoError;
    return 0U;
  }

  taskENTER_CRITICAL();
  const bool committed = g_rx_state.CommitRead(plan);
  taskEXIT_CRITICAL();
  if (!committed) {
    LatchTaskError(Usart1Error::kDma);
    *status = Status::kIoError;
    return 0U;
  }
  return plan.copy_length;
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

  while (xSemaphoreTake(g_tx_semaphore, 0U) == pdTRUE) {
  }
  std::memcpy(g_tx_bounce, source, length);
  if (HAL_UART_Transmit_DMA(&g_usart1, g_tx_bounce,
                            static_cast<std::uint16_t>(length)) != HAL_OK) {
    taskENTER_CRITICAL();
    g_tx_in_progress = false;
    taskEXIT_CRITICAL();
    LatchTaskError(Usart1Error::kDma);
    *status = Status::kIoError;
    return 0U;
  }

  const std::uint32_t start_cycles = CycleCounter();
  const BaseType_t completed =
      xSemaphoreTake(g_tx_semaphore, pdMS_TO_TICKS(WriteDeadlineMs(length)));
  UpdateMaximumWait(start_cycles);
  if (completed != pdTRUE) {
    LatchTaskError(Usart1Error::kTxTimeout);
    *status = Status::kTimeout;
    return 0U;
  }
  if ((g_error_flags & ErrorBit(Usart1Error::kDma)) != 0U) {
    *status = Status::kIoError;
    return 0U;
  }
  return length;
}

TransportSnapshot GetTransportSnapshot() {
  taskENTER_CRITICAL();
  const std::uint8_t error_flags =
      g_rx_dma_top_half_error ? static_cast<std::uint8_t>(
                                    g_error_flags | ErrorBit(Usart1Error::kDma))
                              : g_error_flags;
  const TransportSnapshot snapshot{g_rx_state.producer_position(),
                                   g_rx_state.consumer_position(),
                                   g_rx_state.high_water_bytes(),
                                   g_maximum_wait_us,
                                   g_rx_state.rx_wire_bytes(),
                                   g_tx_wire_bytes,
                                   error_flags,
                                   g_open};
  taskEXIT_CRITICAL();
  return snapshot;
}

bool TransportHasFatalError() {
  return g_error_flags != 0U || g_rx_dma_top_half_error;
}

void HandleUsart1RxDmaTopHalfFromIsr() {
  const std::uint32_t half_flag = __HAL_DMA_GET_HT_FLAG_INDEX(&g_dma_usart1_rx);
  const std::uint32_t complete_flag =
      __HAL_DMA_GET_TC_FLAG_INDEX(&g_dma_usart1_rx);
  const std::uint32_t transfer_error_flag =
      __HAL_DMA_GET_TE_FLAG_INDEX(&g_dma_usart1_rx);
  const std::uint32_t direct_error_flag =
      __HAL_DMA_GET_DME_FLAG_INDEX(&g_dma_usart1_rx);
  const std::uint32_t fifo_error_flag =
      __HAL_DMA_GET_FE_FLAG_INDEX(&g_dma_usart1_rx);

  std::uint32_t boundary_increment = 0U;
  if (__HAL_DMA_GET_FLAG(&g_dma_usart1_rx, half_flag) != RESET &&
      __HAL_DMA_GET_IT_SOURCE(&g_dma_usart1_rx, DMA_IT_HT) != RESET) {
    __HAL_DMA_CLEAR_FLAG(&g_dma_usart1_rx, half_flag);
    ++boundary_increment;
  }
  if (__HAL_DMA_GET_FLAG(&g_dma_usart1_rx, complete_flag) != RESET &&
      __HAL_DMA_GET_IT_SOURCE(&g_dma_usart1_rx, DMA_IT_TC) != RESET) {
    __HAL_DMA_CLEAR_FLAG(&g_dma_usart1_rx, complete_flag);
    ++boundary_increment;
  }
  g_rx_dma_boundary_count += boundary_increment;

  const bool transfer_error =
      __HAL_DMA_GET_FLAG(&g_dma_usart1_rx, transfer_error_flag) != RESET;
  const bool direct_error =
      __HAL_DMA_GET_FLAG(&g_dma_usart1_rx, direct_error_flag) != RESET;
  const bool fifo_error =
      __HAL_DMA_GET_FLAG(&g_dma_usart1_rx, fifo_error_flag) != RESET;
  if (transfer_error || direct_error || fifo_error) {
    __HAL_DMA_CLEAR_FLAG(&g_dma_usart1_rx, transfer_error_flag |
                                               direct_error_flag |
                                               fifo_error_flag);
    __HAL_DMA_DISABLE_IT(&g_dma_usart1_rx,
                         DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&g_dma_usart1_rx, DMA_IT_FE);
    g_rx_dma_top_half_error = true;
  }

  // USART1 runs below the syscall ceiling and performs the task notification,
  // producer snapshot, and motor stop after any in-progress task critical
  // section completes. No FreeRTOS API is legal in this top half.
  __DMB();
  HAL_NVIC_SetPendingIRQ(USART1_IRQn);
}

void HandleUsart1RxDmaProgressFromIsr() {
  UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  UpdateProducerLocked();
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  if (g_error_flags != 0U || g_rx_dma_top_half_error) {
    EmergencyStopMotors();
  }
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

void HandleUsart1DmaErrorFromIsr() {
  BaseType_t higher_priority_task_woken = pdFALSE;
  UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  g_error_flags |= ErrorBit(Usart1Error::kDma);
  g_active_tx_length = 0U;
  g_tx_in_progress = false;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  EmergencyStopMotors();
  if (g_tx_semaphore != nullptr) {
    xSemaphoreGiveFromISR(g_tx_semaphore, &higher_priority_task_woken);
  }
  NotifyTaskFromIsr(TaskId::kMicroRos);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HandleUsart1Irq() {
  HandleUsart1RxDmaProgressFromIsr();
  if (g_rx_dma_top_half_error) {
    HandleUsart1DmaErrorFromIsr();
    return;
  }
  const std::uint32_t status_register = g_usart1.Instance->SR;
  const std::uint32_t error_bits =
      status_register &
      (USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_PE);
  if (error_bits != 0U) {
    volatile const std::uint32_t discarded = g_usart1.Instance->DR;
    static_cast<void>(discarded);
    UBaseType_t const interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    if ((error_bits & USART_SR_FE) != 0U) {
      g_error_flags |= ErrorBit(Usart1Error::kFraming);
    }
    if ((error_bits & USART_SR_NE) != 0U) {
      g_error_flags |= ErrorBit(Usart1Error::kNoise);
    }
    if ((error_bits & USART_SR_ORE) != 0U) {
      g_error_flags |= ErrorBit(Usart1Error::kOverrun);
    }
    if ((error_bits & USART_SR_PE) != 0U) {
      g_error_flags |= ErrorBit(Usart1Error::kParity);
    }
    taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
    EmergencyStopMotors();
    if (g_tx_in_progress) {
      HandleUsart1DmaErrorFromIsr();
    } else {
      NotifyTaskFromIsr(TaskId::kMicroRos);
    }
    return;
  }

  if ((status_register & USART_SR_IDLE) != 0U &&
      (g_usart1.Instance->CR1 & USART_CR1_IDLEIE) != 0U) {
    volatile const std::uint32_t discarded = g_usart1.Instance->DR;
    static_cast<void>(discarded);
    HandleUsart1RxDmaProgressFromIsr();
  }

  // HAL owns the normal TX-DMA transition from DMA complete to UART TC. It is
  // called only after the error path above, so it cannot abort circular RX in
  // response to FE/NE/ORE/PE from interrupt context.
  if ((status_register & USART_SR_TC) != 0U &&
      (g_usart1.Instance->CR1 & USART_CR1_TCIE) != 0U) {
    HAL_UART_IRQHandler(&g_usart1);
  }
}

}  // namespace mentor_pi_mcu::platform::stm32

extern "C" bool MentorPiTransportOpen(uxrCustomTransport* transport) {
  static_cast<void>(transport);
  return mentor_pi_mcu::platform::stm32::OpenUsart1Transport() ==
         mentor_pi_mcu::platform::stm32::Status::kOk;
}

extern "C" bool MentorPiTransportClose(uxrCustomTransport* transport) {
  static_cast<void>(transport);
  mentor_pi_mcu::platform::stm32::CloseUsart1Transport();
  return true;
}

extern "C" std::size_t MentorPiTransportWrite(uxrCustomTransport* transport,
                                              const std::uint8_t* buffer,
                                              std::size_t length,
                                              std::uint8_t* error) {
  static_cast<void>(transport);
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
  static_cast<void>(transport);
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
