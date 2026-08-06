# Copyright 2026 Mentor Pi contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

foreach(required_variable TRANSPORT_SOURCE INTERRUPTS_SOURCE MSP_SOURCE)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

file(READ "${TRANSPORT_SOURCE}" transport_source)
file(READ "${INTERRUPTS_SOURCE}" interrupts_source)
file(READ "${MSP_SOURCE}" msp_source)

function(extract_function source start_marker end_marker output)
  string(FIND "${source}" "${start_marker}" start)
  string(FIND "${source}" "${end_marker}" end)
  if(start EQUAL -1 OR end EQUAL -1 OR end LESS_EQUAL start)
    message(FATAL_ERROR
            "Cannot isolate ${start_marker} before ${end_marker}")
  endif()
  math(EXPR length "${end} - ${start}")
  string(SUBSTRING "${source}" ${start} ${length} body)
  set(${output} "${body}" PARENT_SCOPE)
endfunction()

extract_function("${interrupts_source}" "void DMA2_Stream2_IRQHandler()"
                 "void DMA2_Stream3_IRQHandler()" rx_dma_irq_body)
string(FIND "${rx_dma_irq_body}"
       "HandleUsart1RxDmaTopHalfFromIsr()" top_half_call)
if(top_half_call EQUAL -1)
  message(FATAL_ERROR "DMA2 Stream 2 does not delegate to the RX top half")
endif()
string(FIND "${rx_dma_irq_body}" "HAL_DMA_IRQHandler" hal_dma_call)
if(NOT hal_dma_call EQUAL -1)
  message(FATAL_ERROR "RX DMA IRQ must not enter the HAL DMA state machine")
endif()

extract_function("${transport_source}"
                 "void HandleUsart1RxDmaTopHalfFromIsr()"
                 "void HandleUsart1RxDmaProgressFromIsr()" top_half_body)
foreach(forbidden_call
        "taskENTER_"
        "taskEXIT_"
        "xSemaphore"
        "NotifyTaskFromIsr"
        "portYIELD_FROM_ISR"
        "EmergencyStopMotors"
        "HAL_DMA_IRQHandler"
        "HAL_UART_IRQHandler")
  string(FIND "${top_half_body}" "${forbidden_call}" forbidden_position)
  if(NOT forbidden_position EQUAL -1)
    message(FATAL_ERROR
            "RX DMA top half contains forbidden call ${forbidden_call}")
  endif()
endforeach()
foreach(required_fragment
        "g_rx_dma_boundary_count += boundary_increment"
        "g_rx_dma_top_half_error = true"
        "HAL_NVIC_SetPendingIRQ(USART1_IRQn)")
  string(FIND "${top_half_body}" "${required_fragment}" fragment_position)
  if(fragment_position EQUAL -1)
    message(FATAL_ERROR
            "RX DMA top half is missing ${required_fragment}")
  endif()
endforeach()

string(FIND "${msp_source}"
       "constexpr std::uint32_t kRxDmaAccountingIrqPriority = 4U"
       priority_definition)
string(FIND "${msp_source}"
       "kRxDmaAccountingIrqPriority <"
       priority_assertion)
string(FIND "${msp_source}"
       "HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, kRxDmaAccountingIrqPriority"
       priority_assignment)
if(priority_definition EQUAL -1 OR priority_assertion EQUAL -1 OR
   priority_assignment EQUAL -1)
  message(FATAL_ERROR
          "RX DMA IRQ is not fixed above the FreeRTOS syscall ceiling")
endif()

foreach(required_fragment
        "CircularDmaPosition<"
        "boundary_before == boundary_after"
        "Usart1RxDmaPosition::IsConsistent"
        "Usart1RxDmaPosition::Reconstruct")
  string(FIND "${transport_source}" "${required_fragment}" fragment_position)
  if(fragment_position EQUAL -1)
    message(FATAL_ERROR
            "RX DMA producer reconstruction is missing ${required_fragment}")
  endif()
endforeach()

foreach(sticky_state g_rx_dma_boundary_count g_rx_dma_top_half_error)
  string(REGEX MATCHALL "${sticky_state}[\t ]*=[\t ]*(0U|false)"
         reset_assignments "${transport_source}")
  list(LENGTH reset_assignments reset_count)
  if(NOT reset_count EQUAL 2)
    message(FATAL_ERROR
            "${sticky_state} must be initialized once and reset only by open; "
            "found ${reset_count} reset assignments")
  endif()
endforeach()

extract_function("${transport_source}" "Status OpenUsart1Transport()"
                 "void CloseUsart1Transport()" transport_open_body)
string(FIND "${transport_open_body}"
       "HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn)" disable_position)
string(FIND "${transport_open_body}"
       "g_rx_dma_boundary_count = 0U" epoch_reset_position)
string(FIND "${transport_open_body}"
       "g_rx_dma_top_half_error = false" error_reset_position)
if(disable_position EQUAL -1 OR epoch_reset_position EQUAL -1 OR
   error_reset_position EQUAL -1 OR
   disable_position GREATER epoch_reset_position OR
   disable_position GREATER error_reset_position)
  message(FATAL_ERROR
          "Open must disable the above-BASEPRI RX IRQ before epoch reset")
endif()

message(STATUS "STM32 RX DMA top-half source contract passed")
