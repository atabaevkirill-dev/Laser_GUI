cmake_minimum_required(VERSION 3.16)
project(hexabot_locomotion CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hexabot_interfaces REQUIRED)

# ROS-free core: geometry, IK, gait, odometry.
add_library(hexabot_locomotion_core
  src/geometry.cpp
  src/gait.cpp
  src/odometry.cpp
  src/interpolator.cpp
)
target_include_directories(hexabot_locomotion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node hexabot_locomotion_core)
ament_target_dependencies(locomotion_node
  rclcpp geometry_msgs sensor_msgs std_msgs nav_msgs lifecycle_msgs tf2_ros
  controller_manager_msgs hexabot_interfaces)

add_executable(gait_trace tools/gait_trace.cpp)
target_link_libraries(gait_trace hexabot_locomotion_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS hexabot_locomotion_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS locomotion_node gait_trace DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics hexabot_locomotion_core)
  ament_add_gtest(test_gait test/test_gait.cpp)
  target_link_libraries(test_gait hexabot_locomotion_core)
  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry hexabot_locomotion_core)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
