# SPDX-License-Identifier: GPL-2.0-or-later

cmake_minimum_required(VERSION 3.16)
project(mentor_pi_tracking)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mentor_pi_interfaces REQUIRED)
find_package(mentor_pi_tracking_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(ALTRO_CHECKOUT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/altro-cpp")
set(ALTRO_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/patches/altro-disable-docs.patch")
if(NOT EXISTS "${ALTRO_CHECKOUT}/CMakeLists.txt" OR
  NOT EXISTS "${ALTRO_CHECKOUT}/LICENSE" OR NOT EXISTS "${ALTRO_PATCH}")
  message(FATAL_ERROR
    "Pinned ALTO sources are missing; run tools/bootstrap_host_dependencies.sh")
endif()
set(ALTRO_PATCHED_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/altro-patched-source")
file(REMOVE_RECURSE "${ALTRO_PATCHED_SOURCE}")
file(MAKE_DIRECTORY "${ALTRO_PATCHED_SOURCE}")
file(COPY "${ALTRO_CHECKOUT}/" DESTINATION "${ALTRO_PATCHED_SOURCE}"
  PATTERN ".git" EXCLUDE)
execute_process(
  COMMAND patch --dry-run --silent -p1 -i "${ALTRO_PATCH}"
  WORKING_DIRECTORY "${ALTRO_PATCHED_SOURCE}"
  RESULT_VARIABLE ALTRO_PATCH_CHECK
)
if(NOT ALTRO_PATCH_CHECK EQUAL 0)
  message(FATAL_ERROR "Reviewed ALTO compatibility patch no longer applies")
endif()
execute_process(
  COMMAND patch --silent -p1 -i "${ALTRO_PATCH}"
  WORKING_DIRECTORY "${ALTRO_PATCHED_SOURCE}"
  COMMAND_ERROR_IS_FATAL ANY
)
set(ALTRO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ALTRO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ALTRO_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(ALTRO_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(ALTRO_RUN_CLANG_TIDY OFF CACHE BOOL "" FORCE)
set(ALTRO_SET_POSITION_INDEPENDENT ON CACHE BOOL "" FORCE)
add_subdirectory("${ALTRO_PATCHED_SOURCE}" altro-build EXCLUDE_FROM_ALL)

set(MENTOR_PI_WARNING_FLAGS -Wall -Wextra -Wpedantic)

add_library(mentor_pi_tracking_core STATIC
  src/mpc_solver.cpp
  src/polynomial_trajectory.cpp
)
target_compile_features(mentor_pi_tracking_core PUBLIC cxx_std_17)
target_compile_options(mentor_pi_tracking_core PRIVATE ${MENTOR_PI_WARNING_FLAGS})
target_include_directories(mentor_pi_tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mentor_pi_tracking_core PRIVATE
  "${ALTRO_PATCHED_SOURCE}")
ament_target_dependencies(mentor_pi_tracking_core
  mentor_pi_tracking_interfaces
)
target_link_libraries(mentor_pi_tracking_core altro)

add_library(mentor_pi_tracker_node STATIC src/tracker_node.cpp)
target_compile_features(mentor_pi_tracker_node PUBLIC cxx_std_17)
target_compile_options(mentor_pi_tracker_node PRIVATE ${MENTOR_PI_WARNING_FLAGS})
target_include_directories(mentor_pi_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mentor_pi_tracker_node
  diagnostic_msgs geometry_msgs mentor_pi_interfaces
  mentor_pi_tracking_interfaces nav_msgs rclcpp std_msgs std_srvs
)
target_link_libraries(mentor_pi_tracker_node mentor_pi_tracking_core)

add_executable(mecanum_mpc_tracker src/mecanum_tracker_main.cpp)
add_executable(ackermann_mpc_tracker src/ackermann_tracker_main.cpp)
foreach(target mecanum_mpc_tracker ackermann_mpc_tracker)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE ${MENTOR_PI_WARNING_FLAGS})
  ament_target_dependencies(${target} rclcpp)
  target_link_libraries(${target} mentor_pi_tracker_node)
endforeach()

add_executable(polynomial_trajectory_publisher
  examples/polynomial_trajectory_publisher.cpp)
target_compile_features(polynomial_trajectory_publisher PRIVATE cxx_std_17)
target_compile_options(polynomial_trajectory_publisher PRIVATE
  ${MENTOR_PI_WARNING_FLAGS})
ament_target_dependencies(polynomial_trajectory_publisher
  mentor_pi_tracking_interfaces rclcpp)

install(TARGETS
  mentor_pi_tracking_core mentor_pi_tracker_node
  mecanum_mpc_tracker ackermann_mpc_tracker polynomial_trajectory_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${ALTRO_CHECKOUT}/LICENSE"
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME ALTO-GPL-2.0.txt)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(polynomial_trajectory_test
    test/polynomial_trajectory_test.cpp)
  if(TARGET polynomial_trajectory_test)
    target_compile_features(polynomial_trajectory_test PRIVATE cxx_std_17)
    target_compile_options(polynomial_trajectory_test PRIVATE
      ${MENTOR_PI_WARNING_FLAGS})
    ament_target_dependencies(polynomial_trajectory_test
      mentor_pi_tracking_interfaces)
    target_link_libraries(polynomial_trajectory_test mentor_pi_tracking_core)
  endif()
  ament_add_gtest(mpc_solver_test test/mpc_solver_test.cpp)
  if(TARGET mpc_solver_test)
    target_compile_features(mpc_solver_test PRIVATE cxx_std_17)
    target_compile_options(mpc_solver_test PRIVATE ${MENTOR_PI_WARNING_FLAGS})
    ament_target_dependencies(mpc_solver_test mentor_pi_tracking_interfaces)
    target_link_libraries(mpc_solver_test mentor_pi_tracking_core)
  endif()
  ament_add_gtest(tracker_node_contract_test test/tracker_node_contract_test.cpp)
  if(TARGET tracker_node_contract_test)
    target_compile_features(tracker_node_contract_test PRIVATE cxx_std_17)
    target_compile_options(tracker_node_contract_test PRIVATE
      ${MENTOR_PI_WARNING_FLAGS})
    ament_target_dependencies(tracker_node_contract_test
      geometry_msgs mentor_pi_interfaces mentor_pi_tracking_interfaces nav_msgs
      rclcpp std_msgs std_srvs)
    target_link_libraries(tracker_node_contract_test mentor_pi_tracker_node)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
