foreach(required_variable PLATFORM_SOURCE CONTROLLER_SOURCE LINKER_SCRIPT
                          TARGET_CMAKE TARGET_SOURCE)
  if(NOT DEFINED ${required_variable} OR
     NOT EXISTS "${${required_variable}}")
    message(FATAL_ERROR
      "Missing watchdog-retention contract input: ${required_variable}")
  endif()
endforeach()

file(READ "${PLATFORM_SOURCE}" platform_source)
file(READ "${CONTROLLER_SOURCE}" controller_source)
file(READ "${LINKER_SCRIPT}" linker_source)
file(READ "${TARGET_CMAKE}" target_cmake_source)
file(READ "${TARGET_SOURCE}" target_source)

foreach(required_platform_text
    "section(\".noinit.watchdog_retention\")"
    "g_watchdog_retention_record;"
    "WatchdogTaskForReset(retained, g_reset_reason)"
    "ClearWatchdogRetentionRecord();"
    "void PersistWatchdogTask(std::uint8_t task)"
    "g_watchdog_task_persisted"
    "g_watchdog_retention_record.payload_complement = ~payload")
  string(FIND "${platform_source}" "${required_platform_text}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR
      "STM32 watchdog retention is missing: ${required_platform_text}")
  endif()
endforeach()

string(FIND "${platform_source}"
  "void PersistWatchdogTask(std::uint8_t task)" persist_start)
string(SUBSTRING "${platform_source}" ${persist_start} -1 persist_source)
string(FIND "${persist_source}"
  "g_watchdog_retention_record.magic = 0U;" invalid_magic)
string(FIND "${persist_source}"
  "g_watchdog_retention_record.payload = payload;" payload_write)
string(FIND "${persist_source}"
  "g_watchdog_retention_record.payload_complement = ~payload;"
  complement_write)
string(FIND "${persist_source}" "__DMB();" publish_barrier)
string(FIND "${persist_source}"
  "g_watchdog_retention_record.magic = kWatchdogRetentionMagic;" valid_magic)
if(invalid_magic EQUAL -1 OR payload_write EQUAL -1 OR
   complement_write EQUAL -1 OR publish_barrier EQUAL -1 OR
   valid_magic EQUAL -1 OR NOT invalid_magic LESS payload_write OR
   NOT payload_write LESS complement_write OR
   NOT complement_write LESS publish_barrier OR
   NOT publish_barrier LESS valid_magic)
  message(FATAL_ERROR
    "Retained watchdog write must invalidate magic, write payload/complement, "
    "barrier, then publish magic")
endif()

string(FIND "${controller_source}"
  "hooks_.persist_watchdog_task(hooks_.context" persist_hook)
string(FIND "${controller_source}"
  "watchdog_task_persist_requested_ = true;" persist_once)
string(FIND "${controller_source}"
  "last_watchdog_task_.store(stale_task" publishes_live_stall)
if(persist_hook EQUAL -1 OR persist_once EQUAL -1 OR
   NOT publishes_live_stall EQUAL -1)
  message(FATAL_ERROR
    "Controller must persist a live offender once without publishing it as "
    "prior-boot evidence")
endif()

string(FIND "${linker_source}"
  "KEEP(*(.noinit.watchdog_retention))" retained_keep)
string(FIND "${target_cmake_source}"
  "platform/stm32/src/watchdog_retention.cc" codec_linked)
if(retained_keep EQUAL -1 OR codec_linked EQUAL -1)
  message(FATAL_ERROR
    "Watchdog record must be retained by the linker and codec linked on target")
endif()

string(FIND "${target_source}"
  "controller::kControllerTaskCount == platform::kWatchdogTaskCount"
  task_count_assert)
if(task_count_assert EQUAL -1)
  message(FATAL_ERROR
    "Target glue must statically tie controller and retained task counts")
endif()
