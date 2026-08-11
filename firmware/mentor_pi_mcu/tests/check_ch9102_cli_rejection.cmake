if(NOT DEFINED BOOT_CONTROL OR NOT EXISTS "${BOOT_CONTROL}")
  message(FATAL_ERROR "BOOT_CONTROL executable is missing")
endif()

execute_process(
  COMMAND "${BOOT_CONTROL}" --device /dev/null --mode application
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(result EQUAL 0)
  message(FATAL_ERROR "CH9102F boot control accepted /dev/null")
endif()
set(combined "${output}${error}")
if(NOT combined MATCHES "expected 1a86:55d4")
  message(FATAL_ERROR
    "CH9102F rejection did not report the required identity: ${combined}")
endif()
