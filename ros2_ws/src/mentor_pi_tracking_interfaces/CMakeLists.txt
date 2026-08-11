# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(mentor_pi_tracking_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolynomialSegment.msg"
  "msg/PolynomialTrajectory.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
