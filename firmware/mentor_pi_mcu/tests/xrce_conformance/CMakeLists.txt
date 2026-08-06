cmake_minimum_required(VERSION 3.16)
project(mentor_pi_xrce_conformance VERSION 2.4.2 LANGUAGES C CXX)

include(FetchContent)
include(CTest)

option(MENTOR_PI_XRCE_CONFORMANCE_SANITIZERS
       "Build the conformance target with AddressSanitizer and UBSan" ON)
set(MENTOR_PI_XRCE_CLIENT_ARCHIVE
    "https://github.com/eProsima/Micro-XRCE-DDS-Client/archive/83f129a80770a09aac9e823896ecbf6a0eddf0fc.tar.gz"
    CACHE STRING "Pinned Micro-XRCE-DDS Client source archive URL or local path")

set(MENTOR_PI_XRCE_CLIENT_COMMIT
    "83f129a80770a09aac9e823896ecbf6a0eddf0fc")
set(MENTOR_PI_XRCE_CLIENT_ARCHIVE_SHA256
    "87ee5057b33385e81a713670b479d24feb17811c2d75c6cf8d1bc63470813933")

get_filename_component(MENTOR_PI_MCU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.."
                       ABSOLUTE)
file(READ "${MENTOR_PI_MCU_ROOT}/config/microros_sources.lock"
     MENTOR_PI_MICROROS_SOURCE_LOCK)
set(MENTOR_PI_EXPECTED_LOCK_ENTRY
    "https://github.com/eProsima/Micro-XRCE-DDS-Client ${MENTOR_PI_XRCE_CLIENT_COMMIT}")
string(FIND "${MENTOR_PI_MICROROS_SOURCE_LOCK}"
       "${MENTOR_PI_EXPECTED_LOCK_ENTRY}" MENTOR_PI_LOCK_ENTRY_POSITION)
if(MENTOR_PI_LOCK_ENTRY_POSITION EQUAL -1)
  message(FATAL_ERROR
          "XRCE conformance source does not match microros_sources.lock")
endif()

FetchContent_Declare(
  mentor_pi_pinned_xrce_client
  URL "${MENTOR_PI_XRCE_CLIENT_ARCHIVE}"
  URL_HASH "SHA256=${MENTOR_PI_XRCE_CLIENT_ARCHIVE_SHA256}"
  DOWNLOAD_EXTRACT_TIMESTAMP OFF
  SOURCE_SUBDIR "rrclite-populate-only"
)
FetchContent_MakeAvailable(mentor_pi_pinned_xrce_client)

set(UCLIENT_PROFILE_DISCOVERY OFF)
set(UCLIENT_PROFILE_UDP OFF)
set(UCLIENT_PROFILE_TCP OFF)
set(UCLIENT_PROFILE_CAN OFF)
set(UCLIENT_PROFILE_SERIAL OFF)
set(UCLIENT_PROFILE_CUSTOM_TRANSPORT ON)
set(UCLIENT_PROFILE_MULTITHREAD OFF)
set(UCLIENT_PROFILE_SHARED_MEMORY OFF)
set(UCLIENT_PROFILE_STREAM_FRAMING ON)
set(UCLIENT_PLATFORM_POSIX OFF)
set(UCLIENT_PLATFORM_POSIX_NOPOLL OFF)
set(UCLIENT_PLATFORM_WINDOWS OFF)
set(UCLIENT_PLATFORM_FREERTOS_PLUS_TCP OFF)
set(UCLIENT_PLATFORM_RTEMS_BSD_NET OFF)
set(UCLIENT_PLATFORM_ZEPHYR OFF)
set(UCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS 1)
set(UCLIENT_MAX_OUTPUT_RELIABLE_STREAMS 1)
set(UCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS 1)
set(UCLIENT_MAX_INPUT_RELIABLE_STREAMS 1)
set(UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS 10)
set(UCLIENT_MIN_SESSION_CONNECTION_INTERVAL 1000)
set(UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL 100)
set(UCLIENT_CUSTOM_TRANSPORT_MTU 512)
set(UCLIENT_TWEAK_XRCE_WRITE_LIMIT ON)
set(UCLIENT_HARD_LIVELINESS_CHECK OFF)

set(MENTOR_PI_XRCE_GENERATED_INCLUDE
    "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
file(MAKE_DIRECTORY "${MENTOR_PI_XRCE_GENERATED_INCLUDE}/uxr/client")
configure_file(
  "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/include/uxr/client/config.h.in"
  "${MENTOR_PI_XRCE_GENERATED_INCLUDE}/uxr/client/config.h"
)

add_library(mentor_pi_pinned_xrce_framing STATIC
  "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/src/c/profile/transport/custom/custom_transport.c"
  "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/src/c/profile/transport/stream_framing/stream_framing_protocol.c"
  "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/src/c/util/time.c"
)
target_include_directories(mentor_pi_pinned_xrce_framing
  PUBLIC
    "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/include"
    "${MENTOR_PI_XRCE_GENERATED_INCLUDE}"
  PRIVATE
    "${mentor_pi_pinned_xrce_client_SOURCE_DIR}/src/c"
)
set_target_properties(mentor_pi_pinned_xrce_framing PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED YES
)

add_executable(mentor_pi_xrce_framing_conformance
  xrce_framing_conformance.cc
)
target_link_libraries(mentor_pi_xrce_framing_conformance
  PRIVATE mentor_pi_pinned_xrce_framing)
target_compile_features(mentor_pi_xrce_framing_conformance PRIVATE cxx_std_17)
target_compile_options(mentor_pi_xrce_framing_conformance PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wshadow
  -Wdouble-promotion
  -Wformat=2
  -Werror
  -fno-exceptions
  -fno-rtti
)

if(MENTOR_PI_XRCE_CONFORMANCE_SANITIZERS AND
   CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" AND
   CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mentor_pi_pinned_xrce_framing PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
  target_compile_options(mentor_pi_xrce_framing_conformance PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
  target_link_options(mentor_pi_xrce_framing_conformance PRIVATE
    -fsanitize=address,undefined
  )
  if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    # The pinned client's public custom-transport adapter casts its typed
    # callback to the framing layer's equivalent void-pointer signature. Keep
    # every other ASan/UBSan check enabled while suppressing only Clang's
    # function-type instrumentation on these unmodified upstream C sources.
    target_compile_options(mentor_pi_pinned_xrce_framing PRIVATE
      -fno-sanitize=function
    )
  endif()
endif()

add_test(NAME mentor_pi_xrce_framing_conformance
         COMMAND mentor_pi_xrce_framing_conformance)
