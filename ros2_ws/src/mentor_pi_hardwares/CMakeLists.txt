cmake_minimum_required(VERSION 3.16)
project(mentor_pi_hardwares)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mentor_pi_interfaces REQUIRED)

add_library(mentor_pi_hardwares SHARED
  src/hardware_common.cpp
  src/mecanum_hardware.cpp
  src/ackermann_hardware.cpp
)

target_compile_features(mentor_pi_hardwares PUBLIC cxx_std_17)
target_include_directories(mentor_pi_hardwares
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set(MENTOR_PI_WARNING_FLAGS
  -Wall
  -Wextra
  -Wpedantic
)

target_compile_options(mentor_pi_hardwares
  PRIVATE
    ${MENTOR_PI_WARNING_FLAGS}
)

ament_target_dependencies(mentor_pi_hardwares
  controller_manager
  hardware_interface
  mentor_pi_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_plugins.xml)

install(
  TARGETS mentor_pi_hardwares
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Remove launch entry points retired by the unified vehicle.launch.py. CMake's
# install step does not otherwise delete files left by an older release.
install(CODE "
  file(REMOVE
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/mecanum.launch.py\"
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/ackermann.launch.py\"
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/exp_vehicle_launch.py\"
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/vehicle_launch.py\"
  )
")

install(
  FILES ros2_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mentor_pi_hardwares)
ament_export_dependencies(
  controller_manager
  hardware_interface
  mentor_pi_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  ament_add_gtest(hardware_common_test test/hardware_common_test.cpp)
  if(TARGET hardware_common_test)
    target_link_libraries(hardware_common_test mentor_pi_hardwares)
    ament_target_dependencies(hardware_common_test mentor_pi_interfaces)
    target_compile_features(hardware_common_test PRIVATE cxx_std_17)
    target_compile_options(hardware_common_test PRIVATE ${MENTOR_PI_WARNING_FLAGS})
  endif()

  ament_add_gtest(hardware_plugin_test test/hardware_plugin_test.cpp)
  if(TARGET hardware_plugin_test)
    ament_target_dependencies(hardware_plugin_test
      hardware_interface
      mentor_pi_interfaces
      pluginlib
      rclcpp
      rclcpp_lifecycle
      std_msgs
    )
    target_compile_features(hardware_plugin_test PRIVATE cxx_std_17)
    target_compile_options(hardware_plugin_test PRIVATE ${MENTOR_PI_WARNING_FLAGS})
  endif()

  ament_add_pytest_test(hardware_launch_test
    test/test_launch_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
