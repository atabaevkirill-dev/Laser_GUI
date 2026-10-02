cmake_minimum_required(VERSION 3.16)
project(hexabot_navigation NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_configs test/test_configs.py
    ENV CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config)
endif()

ament_package()
