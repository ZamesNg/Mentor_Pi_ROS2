# Copyright 2026 Mentor Pi contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

foreach(required_variable TARGET_SOURCE PERIPHERAL_SOURCE)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

file(READ "${TARGET_SOURCE}" target_source)
file(READ "${PERIPHERAL_SOURCE}" peripheral_source)

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

extract_function("${target_source}" "void WaitForTask("
                 "void EnterCritical(" wait_for_task_body)
foreach(required_fragment
    "task == controller::ControllerTask::kSafetySupervisor"
    "xTaskGetTickCount()"
    "vTaskDelayUntil(&target->safety_last_wake_tick, ticks)")
  string(FIND "${wait_for_task_body}" "${required_fragment}" position)
  if(position EQUAL -1)
    message(FATAL_ERROR
            "SafetySupervisor periodic wait lacks ${required_fragment}")
  endif()
endforeach()

function(require_timer_start_rollback function_body_variable running_flag
         low_marker label)
  set(body "${${function_body_variable}}")
  string(FIND "${body}" "HAL_TIM_Base_Start_IT" start_call)
  string(FIND "${body}" "if (status != HAL_OK)" failure_check)
  string(FIND "${body}" "${running_flag} = false" rollback REVERSE)
  string(FIND "${body}" "${low_marker}" low_write REVERSE)
  if(start_call EQUAL -1 OR failure_check EQUAL -1 OR rollback EQUAL -1 OR
     low_write EQUAL -1 OR failure_check LESS_EQUAL start_call OR
     rollback LESS_EQUAL failure_check OR low_write LESS_EQUAL failure_check)
    message(FATAL_ERROR
            "${label} does not roll back running/output state after start failure")
  endif()
endfunction()

extract_function("${peripheral_source}" "Status StartPwmServoFrameGenerator()"
                 "void StopPwmServoFrameGenerator()" pwm_start_body)
require_timer_start_rollback(pwm_start_body "g_servo_running"
                             "DriveServoPinsLow()" "PWM servo timer start")

extract_function("${peripheral_source}" "Status SetBuzzerFrequency("
                 "Status StartRgbTransfer(" buzzer_start_body)
require_timer_start_rollback(buzzer_start_body "g_buzzer_running"
                             "HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)"
                             "Buzzer timer start")

message(STATUS "STM32 peripheral safety source contract passed")
