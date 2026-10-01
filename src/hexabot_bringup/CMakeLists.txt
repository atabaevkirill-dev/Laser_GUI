cmake_minimum_required(VERSION 3.16)
project(hexabot_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Full stack on a simulated servo bus: controllers + locomotion stand up and walk.
  add_launch_test(test/test_stack_simbus.py TIMEOUT 120)
endif()

ament_package()
