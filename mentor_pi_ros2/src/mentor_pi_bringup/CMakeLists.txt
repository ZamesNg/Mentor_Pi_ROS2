cmake_minimum_required(VERSION 3.16)
project(mentor_pi_bringup LANGUAGES CXX)

option(MENTOR_PI_BUILD_ROS2 "Build the ROS 2 bringup adapters" ON)

add_library(mentor_pi_supervisor_core
  src/configuration.cc
  src/supervisor_core.cc
)
target_compile_features(mentor_pi_supervisor_core PUBLIC cxx_std_17)
target_include_directories(mentor_pi_supervisor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../mentor_pi_interfaces/include>
)

set(MENTOR_PI_WARNING_FLAGS
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wshadow
  -Wdouble-promotion
  -Wformat=2
  -Werror
)
target_compile_options(mentor_pi_supervisor_core
  PRIVATE ${MENTOR_PI_WARNING_FLAGS}
)

add_library(mentor_pi_qualification_core
  src/qualification_monitor_core.cc
)
target_compile_features(mentor_pi_qualification_core PUBLIC cxx_std_17)
target_include_directories(mentor_pi_qualification_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(mentor_pi_qualification_core
  PRIVATE ${MENTOR_PI_WARNING_FLAGS}
)

add_library(mentor_pi_qualification_campaign_core
  src/qualification_campaign_core.cc
  src/qualification_campaign_evidence.cc
  src/qualification_campaign_evidence_verifier.cc
)
target_compile_features(mentor_pi_qualification_campaign_core PUBLIC cxx_std_17)
target_include_directories(mentor_pi_qualification_campaign_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_options(mentor_pi_qualification_campaign_core
  PRIVATE ${MENTOR_PI_WARNING_FLAGS}
)

add_library(mentor_pi_motor_commissioning_core
  src/motor_commissioning_core.cc
)
target_compile_features(mentor_pi_motor_commissioning_core PUBLIC cxx_std_17)
target_include_directories(mentor_pi_motor_commissioning_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(mentor_pi_motor_commissioning_core
  PRIVATE ${MENTOR_PI_WARNING_FLAGS}
)

include(CTest)
if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  find_program(MENTOR_PI_BASH_EXECUTABLE bash)
  if(NOT MENTOR_PI_BASH_EXECUTABLE)
    message(FATAL_ERROR "bash is required for deployment-asset tests")
  endif()

  add_executable(configuration_test test/configuration_test.cc)
  target_link_libraries(configuration_test PRIVATE mentor_pi_supervisor_core)
  target_compile_options(configuration_test PRIVATE ${MENTOR_PI_WARNING_FLAGS})
  add_test(NAME configuration_test COMMAND configuration_test)

  add_executable(supervisor_core_test test/supervisor_core_test.cc)
  target_link_libraries(supervisor_core_test PRIVATE mentor_pi_supervisor_core)
  target_compile_options(supervisor_core_test PRIVATE ${MENTOR_PI_WARNING_FLAGS})
  add_test(NAME supervisor_core_test COMMAND supervisor_core_test)

  add_executable(qualification_monitor_core_test
    test/qualification_monitor_core_test.cc
  )
  target_link_libraries(qualification_monitor_core_test
    PRIVATE mentor_pi_qualification_core
  )
  target_compile_options(qualification_monitor_core_test
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  add_test(
    NAME qualification_monitor_core_test
    COMMAND qualification_monitor_core_test
  )

  add_executable(qualification_campaign_core_test
    test/qualification_campaign_core_test.cc
  )
  target_link_libraries(qualification_campaign_core_test
    PRIVATE mentor_pi_qualification_campaign_core
  )
  target_compile_options(qualification_campaign_core_test
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  add_test(
    NAME qualification_campaign_core_test
    COMMAND qualification_campaign_core_test
  )

  add_executable(qualification_campaign_evidence_verifier_test
    test/qualification_campaign_evidence_verifier_test.cc
  )
  target_include_directories(qualification_campaign_evidence_verifier_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(qualification_campaign_evidence_verifier_test
    PRIVATE
      mentor_pi_qualification_campaign_core
      Threads::Threads
  )
  target_compile_options(qualification_campaign_evidence_verifier_test
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  add_test(
    NAME qualification_campaign_evidence_verifier_test
    COMMAND qualification_campaign_evidence_verifier_test
  )

  add_executable(motor_commissioning_core_test
    test/motor_commissioning_core_test.cc
  )
  target_link_libraries(motor_commissioning_core_test
    PRIVATE mentor_pi_motor_commissioning_core
  )
  target_compile_options(motor_commissioning_core_test
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  add_test(
    NAME motor_commissioning_core_test
    COMMAND motor_commissioning_core_test
  )

  add_test(
    NAME systemd_deployment_test
    COMMAND "${MENTOR_PI_BASH_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/systemd_deployment_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME capture_board_diagnostics_test
    COMMAND "${MENTOR_PI_BASH_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/capture_board_diagnostics_test.sh"
      "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

if(MENTOR_PI_BUILD_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(mentor_pi_interfaces REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    ament_cpplint()
    ament_lint_cmake()
    ament_xmllint()
  endif()

  add_library(configuration_supervisor_node
    src/configuration_supervisor_node.cc
  )
  target_compile_features(configuration_supervisor_node PUBLIC cxx_std_17)
  target_include_directories(configuration_supervisor_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(configuration_supervisor_node
    mentor_pi_supervisor_core
  )
  target_compile_options(configuration_supervisor_node
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(configuration_supervisor_node
    mentor_pi_interfaces
    rcl_interfaces
    rclcpp
    std_msgs
  )

  add_executable(configuration_supervisor
    src/configuration_supervisor_main.cc
  )
  target_compile_features(configuration_supervisor PUBLIC cxx_std_17)
  target_link_libraries(configuration_supervisor
    configuration_supervisor_node
  )
  target_compile_options(configuration_supervisor
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(configuration_supervisor rclcpp)

  if(BUILD_TESTING)
    add_executable(configuration_supervisor_ros_test
      test/configuration_supervisor_ros_test.cc
    )
    target_compile_features(configuration_supervisor_ros_test PUBLIC cxx_std_17)
    target_link_libraries(configuration_supervisor_ros_test
      configuration_supervisor_node
    )
    target_compile_options(configuration_supervisor_ros_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(configuration_supervisor_ros_test
      mentor_pi_interfaces
      rcl_interfaces
      rclcpp
      std_msgs
    )
    add_test(
      NAME configuration_supervisor_ros_test
      COMMAND configuration_supervisor_ros_test
    )
    set_tests_properties(configuration_supervisor_ros_test PROPERTIES TIMEOUT 15)

    add_executable(configuration_supervisor_middleware_fault_test
      test/configuration_supervisor_middleware_fault_test.cc
    )
    target_compile_features(configuration_supervisor_middleware_fault_test
      PUBLIC cxx_std_17
    )
    target_link_libraries(configuration_supervisor_middleware_fault_test
      configuration_supervisor_node
    )
    target_compile_options(configuration_supervisor_middleware_fault_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(configuration_supervisor_middleware_fault_test
      mentor_pi_interfaces
      rcl_interfaces
      rclcpp
      std_msgs
    )
    add_test(
      NAME configuration_supervisor_middleware_fault_test
      COMMAND configuration_supervisor_middleware_fault_test
    )
    set_tests_properties(
      configuration_supervisor_middleware_fault_test
      PROPERTIES TIMEOUT 15
    )

    add_executable(fake_micro_ros_agent test/fake_micro_ros_agent.cc)
    target_compile_features(fake_micro_ros_agent PUBLIC cxx_std_17)
    target_compile_options(fake_micro_ros_agent
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )

    add_executable(configuration_supervisor_launch_test
      test/configuration_supervisor_launch_test.cc
    )
    target_compile_features(
      configuration_supervisor_launch_test PUBLIC cxx_std_17
    )
    target_compile_options(configuration_supervisor_launch_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(configuration_supervisor_launch_test
      mentor_pi_interfaces
      rclcpp
      std_msgs
    )
    add_test(
      NAME configuration_supervisor_launch_test
      COMMAND configuration_supervisor_launch_test
        $<TARGET_FILE:fake_micro_ros_agent>
        ${CMAKE_CURRENT_SOURCE_DIR}/test/configuration_supervisor_launch.yaml
    )
    set_tests_properties(configuration_supervisor_launch_test PROPERTIES
      TIMEOUT 20
    )
  endif()

  add_library(qualification_monitor_node
    src/qualification_monitor_node.cc
  )
  target_compile_features(qualification_monitor_node PUBLIC cxx_std_17)
  target_include_directories(qualification_monitor_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(qualification_monitor_node
    mentor_pi_qualification_core
  )
  target_compile_options(qualification_monitor_node
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(qualification_monitor_node
    mentor_pi_interfaces
    rcl_interfaces
    rclcpp
  )

  add_executable(qualification_monitor
    src/qualification_monitor_main.cc
  )
  target_compile_features(qualification_monitor PUBLIC cxx_std_17)
  target_link_libraries(qualification_monitor
    qualification_monitor_node
  )
  target_compile_options(qualification_monitor
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(qualification_monitor rclcpp)

  if(BUILD_TESTING)
    add_executable(qualification_monitor_node_test
      test/qualification_monitor_node_test.cc
    )
    target_compile_features(qualification_monitor_node_test
      PUBLIC cxx_std_17
    )
    target_link_libraries(qualification_monitor_node_test
      qualification_monitor_node
    )
    target_compile_options(qualification_monitor_node_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(qualification_monitor_node_test
      mentor_pi_interfaces
      rclcpp
    )
    add_test(
      NAME qualification_monitor_node_test
      COMMAND qualification_monitor_node_test
    )
    set_tests_properties(qualification_monitor_node_test PROPERTIES
      TIMEOUT 10
    )
  endif()

  add_library(qualification_campaign_node
    src/qualification_campaign_node.cc
  )
  target_compile_features(qualification_campaign_node PUBLIC cxx_std_17)
  target_include_directories(qualification_campaign_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(qualification_campaign_node
    mentor_pi_supervisor_core
    mentor_pi_qualification_campaign_core
    mentor_pi_qualification_core
  )
  target_compile_options(qualification_campaign_node
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(qualification_campaign_node
    mentor_pi_interfaces
    rcl_interfaces
    rclcpp
  )

  add_executable(qualification_campaign
    src/qualification_campaign_main.cc
  )
  target_compile_features(qualification_campaign PUBLIC cxx_std_17)
  target_link_libraries(qualification_campaign
    qualification_campaign_node
  )
  target_compile_options(qualification_campaign
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(qualification_campaign rclcpp)

  if(BUILD_TESTING)
    add_executable(qualification_campaign_node_test
      test/qualification_campaign_node_test.cc
    )
    target_compile_features(qualification_campaign_node_test PUBLIC cxx_std_17)
    target_link_libraries(qualification_campaign_node_test
      qualification_campaign_node
    )
    target_compile_options(qualification_campaign_node_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(qualification_campaign_node_test
      mentor_pi_interfaces
      rclcpp
    )
    add_test(
      NAME qualification_campaign_node_test
      COMMAND qualification_campaign_node_test
    )
    set_tests_properties(qualification_campaign_node_test PROPERTIES
      TIMEOUT 30
    )
  endif()

  add_library(motor_commissioning_node
    src/motor_commissioning_node.cc
  )
  target_compile_features(motor_commissioning_node PUBLIC cxx_std_17)
  target_include_directories(motor_commissioning_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(motor_commissioning_node
    mentor_pi_motor_commissioning_core
  )
  target_compile_options(motor_commissioning_node
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(motor_commissioning_node
    mentor_pi_interfaces
    rcl_interfaces
    rclcpp
    std_msgs
  )

  add_executable(motor_commissioning
    src/motor_commissioning_main.cc
  )
  target_compile_features(motor_commissioning PUBLIC cxx_std_17)
  target_link_libraries(motor_commissioning
    motor_commissioning_node
  )
  target_compile_options(motor_commissioning
    PRIVATE ${MENTOR_PI_WARNING_FLAGS}
  )
  ament_target_dependencies(motor_commissioning rclcpp)

  if(BUILD_TESTING)
    add_executable(motor_commissioning_node_test
      test/motor_commissioning_node_test.cc
    )
    target_compile_features(motor_commissioning_node_test PUBLIC cxx_std_17)
    target_link_libraries(motor_commissioning_node_test
      motor_commissioning_node
    )
    target_compile_options(motor_commissioning_node_test
      PRIVATE ${MENTOR_PI_WARNING_FLAGS}
    )
    ament_target_dependencies(motor_commissioning_node_test
      mentor_pi_interfaces
      rclcpp
      std_msgs
    )
    add_test(
      NAME motor_commissioning_node_test
      COMMAND motor_commissioning_node_test
    )
    set_tests_properties(motor_commissioning_node_test PROPERTIES TIMEOUT 25)
  endif()

  install(
    TARGETS
      mentor_pi_supervisor_core
      mentor_pi_qualification_core
      mentor_pi_qualification_campaign_core
      mentor_pi_motor_commissioning_core
      configuration_supervisor_node
      configuration_supervisor
      qualification_monitor_node
      qualification_monitor
      qualification_campaign_node
      qualification_campaign
      motor_commissioning_node
      motor_commissioning
    EXPORT mentor_pi_bringup_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config launch systemd udev DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS
    scripts/capture_board_diagnostics
    systemd/install_production_assets
    systemd/promote_host_release
    systemd/require_controller_target_inactive
    systemd/run_configuration_supervisor
    DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES README.md DESTINATION share/${PROJECT_NAME})

  ament_export_include_directories(include)
  ament_export_targets(mentor_pi_bringup_targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    mentor_pi_interfaces
    rcl_interfaces
    rclcpp
    std_msgs
  )
  ament_package()
endif()
