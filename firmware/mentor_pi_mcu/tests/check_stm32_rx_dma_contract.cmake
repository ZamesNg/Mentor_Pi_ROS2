# Copyright 2026 Mentor Pi contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

foreach(required_variable
        TRANSPORT_SOURCE INTERRUPTS_SOURCE MSP_SOURCE ADAPTER_SOURCE)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

file(READ "${TRANSPORT_SOURCE}" transport_source)
file(READ "${INTERRUPTS_SOURCE}" interrupts_source)
file(READ "${MSP_SOURCE}" msp_source)
file(READ "${ADAPTER_SOURCE}" adapter_source)

function(require_fragment source fragment description)
  string(FIND "${source}" "${fragment}" position)
  if(position EQUAL -1)
    message(FATAL_ERROR "Missing ${description}: ${fragment}")
  endif()
endfunction()

function(forbid_fragment source fragment description)
  string(FIND "${source}" "${fragment}" position)
  if(NOT position EQUAL -1)
    message(FATAL_ERROR "Forbidden ${description}: ${fragment}")
  endif()
endfunction()

foreach(fragment
        "HAL_UART_Receive_DMA("
        "CircularDmaPosition<"
        "CircularRxRing<"
        "boundary_before == boundary_after"
        "Usart1RxDmaPosition::IsConsistent"
        "Usart1RxDmaPosition::Reconstruct"
        "HandleUsart1RxDmaBoundaryFromIsr"
        "HAL_UART_Transmit_DMA("
        "xSemaphoreTake("
        "Usart1WriteDeadlineMs(length)"
        "HAL_UART_AbortTransmit("
        "EmergencyStopMotors()")
  require_fragment("${transport_source}" "${fragment}"
                   "bounded HAL DMA transport contract")
endforeach()

foreach(fragment
        "HandleUsart1RxDmaTopHalfFromIsr"
        "__HAL_DMA_DISABLE_IT"
        "HAL_UART_Transmit(")
  forbid_fragment("${transport_source}" "${fragment}"
                  "custom or blocking transport path")
endforeach()

foreach(fragment
        "HAL_DMA_IRQHandler(&stm32_platform::g_dma_usart1_rx)"
        "HAL_DMA_IRQHandler(&stm32_platform::g_dma_usart1_tx)"
        "HAL_UART_IRQHandler(&stm32_platform::g_usart1)"
        "HAL_UART_RxHalfCpltCallback"
        "HandleUsart1RxDmaBoundaryFromIsr()"
        "HandleUsart1TxCompleteFromIsr()"
        "HandleUsart1ErrorFromIsr(uart->ErrorCode)")
  require_fragment("${interrupts_source}" "${fragment}"
                   "standard HAL interrupt/callback path")
endforeach()

foreach(fragment
        "constexpr std::uint32_t kTransportIrqPriority = 6U"
        "kTransportIrqPriority >="
        "HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, kTransportIrqPriority"
        "HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, kTransportIrqPriority"
        "HAL_NVIC_SetPriority(USART1_IRQn, kTransportIrqPriority")
  require_fragment("${msp_source}" "${fragment}"
                   "FreeRTOS-safe transport IRQ priority")
endforeach()

require_fragment("${adapter_source}"
                 "stm32::Usart1TransportArgument()"
                 "dedicated USART1 custom-transport argument")

message(STATUS "STM32 standard HAL RX/TX DMA source contract passed")
