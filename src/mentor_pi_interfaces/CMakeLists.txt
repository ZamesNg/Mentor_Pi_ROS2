cmake_minimum_required(VERSION 3.8)
project(mentor_pi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(message_files
  "msg/BatteryState.msg"
  "msg/BusServoCommand.msg"
  "msg/BusServoState.msg"
  "msg/ButtonEvent.msg"
  "msg/BuzzerCommand.msg"
  "msg/ControllerDiagnostics.msg"
  "msg/Heartbeat.msg"
  "msg/ImuState.msg"
  "msg/LedCommand.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/OledCommand.msg"
  "msg/PwmServoCommand.msg"
  "msg/PwmServoState.msg"
  "msg/Result.msg"
  "msg/RgbCommand.msg"
)

set(service_files
  "srv/ConfigureBusServo.srv"
  "srv/GetBusServoState.srv"
  "srv/SetBatteryThreshold.srv"
  "srv/SetMotorModel.srv"
  "srv/SetPwmServoOffsets.srv"
  "srv/StopBusServos.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${service_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../ros_package_schema/"
  DESTINATION share/ros_package_schema
)
ament_export_include_directories(include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)
  find_package(rosidl_typesupport_cpp REQUIRED)
  find_package(rosidl_typesupport_introspection_cpp REQUIRED)

  ament_cpplint()
  ament_lint_cmake()
  ament_xmllint()

  ament_add_gtest(generated_type_support_test
    test/test_generated_type_support.cpp
    TIMEOUT 60
  )
  if(TARGET generated_type_support_test)
    ament_target_dependencies(generated_type_support_test
      rclcpp
      rosidl_runtime_cpp
      rosidl_typesupport_cpp
      rosidl_typesupport_introspection_cpp
    )
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(generated_type_support_test
      "${cpp_typesupport_target}")
    target_compile_features(generated_type_support_test PRIVATE cxx_std_17)
  endif()

  ament_add_pytest_test(interface_contract_test
    test/test_interface_contract.py
    TIMEOUT 30
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
