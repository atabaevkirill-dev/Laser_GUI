cmake_minimum_required(VERSION 3.16)
project(hexabot_hardware CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Bus protocol and transports (no ROS).
add_library(sts_bus STATIC
  src/sts_protocol.cpp
  src/serial_transport.cpp
  src/simulated_bus.cpp
  src/sts_bus.cpp)
target_include_directories(sts_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
set_target_properties(sts_bus PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED src/sts3215_system.cpp)
target_link_libraries(${PROJECT_NAME} sts_bus)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface hexabot_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} sts_bus
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(PROGRAMS scripts/sts_tool.py DESTINATION lib/${PROJECT_NAME} RENAME sts_tool)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sts_protocol test/test_protocol.cpp)
  target_link_libraries(test_sts_protocol sts_bus)
  ament_add_gtest(test_sts_bus test/test_bus.cpp)
  target_link_libraries(test_sts_bus sts_bus)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
