# Copyright 2026 Mentor Pi contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

foreach(required_variable TARGET_SOURCE TRANSPORT_SOURCE MOTOR_HEADER
                          MICROROS_META BUILD_WRAPPER TARGET_CMAKE_SOURCE
                          PROFILE_PROBE_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

file(READ "${TARGET_SOURCE}" target_source)
file(READ "${TRANSPORT_SOURCE}" transport_source)
file(READ "${MOTOR_HEADER}" motor_header)
file(READ "${MICROROS_META}" microros_meta)

string(REGEX MATCHALL "RMW_UXRCE_MAX_SERVICES=7" service_pool_matches
       "${microros_meta}")
list(LENGTH service_pool_matches service_pool_count)
if(NOT service_pool_count EQUAL 1)
  message(FATAL_ERROR
          "micro-ROS static service capacity must exactly cover 7 services")
endif()

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

extract_function("${target_source}" "Result ArmMotor(" "void DisarmMotor("
                 arm_motor_body)
extract_function("${target_source}" "Result ApplyMotorDuty("
                 "Result InitializePwmServos(" apply_motor_duty_body)

function(require_pre_and_post_gate body_variable write_marker label
         post_gate_output)
  set(body "${${body_variable}}")
  string(FIND "${body}" "platform::TransportHasFatalError()" pre_gate)
  string(FIND "${body}" "${write_marker}" write REVERSE)
  if(pre_gate EQUAL -1 OR write EQUAL -1 OR pre_gate GREATER_EQUAL write)
    message(FATAL_ERROR "${label} lacks a transport pre-check before its write")
  endif()

  string(SUBSTRING "${body}" 0 ${write} pre_write_body)
  string(FIND "${pre_write_body}" "platform::EmergencyStopMotors()" pre_stop)
  string(FIND "${pre_write_body}" "ResultCode::kBusy" pre_busy)
  if(pre_stop EQUAL -1 OR pre_busy EQUAL -1 OR pre_stop LESS_EQUAL pre_gate OR
     pre_busy LESS_EQUAL pre_stop)
    message(FATAL_ERROR
            "${label} pre-check must emergency-stop before returning BUSY")
  endif()

  string(LENGTH "${body}" body_length)
  math(EXPR post_write_length "${body_length} - ${write}")
  string(SUBSTRING "${body}" ${write} ${post_write_length} post_write_body)
  string(FIND "${post_write_body}" "platform::TransportHasFatalError()"
         post_gate_relative)
  if(post_gate_relative EQUAL -1)
    message(FATAL_ERROR "${label} lacks a transport post-check after its write")
  endif()
  math(EXPR post_gate "${write} + ${post_gate_relative}")

  math(EXPR post_gate_length "${body_length} - ${post_gate}")
  string(SUBSTRING "${body}" ${post_gate} ${post_gate_length} post_gate_body)
  string(FIND "${post_gate_body}" "platform::EmergencyStopMotors()" post_stop)
  string(FIND "${post_gate_body}" "ResultCode::kBusy" post_busy)
  if(post_stop EQUAL -1 OR post_busy EQUAL -1 OR post_stop EQUAL 0 OR
     post_busy LESS_EQUAL post_stop)
    message(FATAL_ERROR
            "${label} post-check must emergency-stop before returning BUSY")
  endif()
  set(${post_gate_output} ${post_gate} PARENT_SCOPE)
endfunction()

require_pre_and_post_gate(arm_motor_body "platform::ArmMotorOutput(" "ArmMotor"
                          arm_post_gate)
require_pre_and_post_gate(apply_motor_duty_body
                          "platform::SetMotorDutyPermille("
                          "ApplyMotorDuty" duty_post_gate)

# The named marker is intentionally between the loop and the post-check. Along
# with using the last duty-write occurrence above, this prevents a check placed
# inside the per-channel loop from satisfying the source contract.
string(FIND "${apply_motor_duty_body}"
       "after the complete four-channel update" complete_update_marker)
string(FIND "${apply_motor_duty_body}" "platform::SetMotorDutyPermille("
       last_duty_write REVERSE)
if(complete_update_marker EQUAL -1 OR
   complete_update_marker LESS_EQUAL last_duty_write OR
   complete_update_marker GREATER_EQUAL duty_post_gate)
  message(FATAL_ERROR
          "ApplyMotorDuty post-check is not after the complete update marker")
endif()

# Runtime code may initialize the latch once and clear it only when the normal
# physical transport open path starts a fresh session. Any third assignment to
# zero would create an unaudited unlock path.
string(REGEX MATCHALL "g_error_flags[	 ]*=[	 ]*0U" latch_zero_assignments
       "${transport_source}")
list(LENGTH latch_zero_assignments latch_zero_count)
if(NOT latch_zero_count EQUAL 2)
  message(FATAL_ERROR
          "Expected only declaration and OpenUsart1Transport latch clears; "
          "found ${latch_zero_count}")
endif()

extract_function("${transport_source}" "Status OpenUsart1Transport("
                 "void CloseUsart1Transport(" transport_open_body)
string(FIND "${transport_open_body}" "g_error_flags = 0U" open_clear)
if(open_clear EQUAL -1)
  message(FATAL_ERROR "OpenUsart1Transport no longer clears the fault latch")
endif()

# The target has exactly one motor authority profile: ADRC CLOSED_LOOP with
# bounded authority (6 RPS, ±1000 permille). Legacy mode selection has
# collapsed into this single default profile.
string(FIND "${target_source}"
       "mentor_pi::mcu::MotorControlConfiguration BuildMotorConfiguration()"
       motor_profile_start)
string(FIND "${target_source}"
       "extern \"C\" void vApplicationGetIdleTaskMemory" motor_profile_end)
if(motor_profile_start EQUAL -1 OR motor_profile_end EQUAL -1 OR
   motor_profile_end LESS_EQUAL motor_profile_start)
  message(FATAL_ERROR "Cannot isolate BuildMotorConfiguration")
endif()
math(EXPR motor_profile_length
     "${motor_profile_end} - ${motor_profile_start}")
string(SUBSTRING "${target_source}" ${motor_profile_start}
       ${motor_profile_length} motor_profile_body)
foreach(required_marker "DefaultAdrcMotorControlConfiguration()")
  string(FIND "${motor_profile_body}" "${required_marker}" marker_position)
  if(marker_position EQUAL -1)
    message(FATAL_ERROR
            "BuildMotorConfiguration lacks '${required_marker}'")
  endif()
endforeach()
foreach(forbidden_assignment output_limit_permille)
  string(FIND "${motor_profile_body}" "${forbidden_assignment}"
         assignment_position)
  if(NOT assignment_position EQUAL -1)
    message(FATAL_ERROR
            "Target duplicates motor profile field ${forbidden_assignment}")
  endif()
endforeach()

foreach(required_profile_constant
    "kMotorImplementationMaximumRps = 6.0F"
    "kMotorOutputLimitPermille = 1000"
    "constexpr MotorControlConfiguration DefaultAdrcMotorControlConfiguration()")
  string(FIND "${motor_header}" "${required_profile_constant}"
         profile_constant_position)
  if(profile_constant_position EQUAL -1)
    message(FATAL_ERROR
            "Motor profile contract lacks '${required_profile_constant}'")
  endif()
endforeach()

foreach(forbidden_legacy_profile
    "MotorControlMode"
    "LockedMotorControlConfiguration"
    "CommissioningMotorControlConfiguration"
    "kMotorCommissioning"
    "kMotorDirectionCheck")
  string(FIND "${motor_header}" "${forbidden_legacy_profile}"
         legacy_profile_position)
  if(NOT legacy_profile_position EQUAL -1)
    message(FATAL_ERROR
            "Motor profile contract retains '${forbidden_legacy_profile}'")
  endif()
endforeach()

function(run_profile_probe expected_mode expected_control_mode
         expected_speed expected_output)
  execute_process(
    COMMAND "${BUILD_WRAPPER}" --print-motor-profile
    RESULT_VARIABLE probe_result
    OUTPUT_VARIABLE probe_output
    ERROR_VARIABLE probe_error
  )
  if(NOT probe_result EQUAL 0)
    message(FATAL_ERROR
            "Motor profile probe failed unexpectedly: ${probe_error}")
  endif()
  foreach(expected_line
      "mode=${expected_mode}"
      "control_mode=${expected_control_mode}"
      "maximum_accepted_rps=${expected_speed}"
      "output_limit_permille=${expected_output}"
      "release_qualified=0")
    string(FIND "${probe_output}" "${expected_line}" line_position)
    if(line_position EQUAL -1)
      message(FATAL_ERROR
              "Motor profile probe lacks '${expected_line}'")
    endif()
  endforeach()
endfunction()

run_profile_probe(ADRC CLOSED_LOOP 6.0 1000)
file(REMOVE_RECURSE "${PROFILE_PROBE_ROOT}")

message(STATUS "STM32 motor transport/profile gate contract passed")
