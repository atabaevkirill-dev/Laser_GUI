cmake_minimum_required(VERSION 3.16)
project(hexabot_gazebo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py scripts/walk_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_worlds test/test_worlds.py
    ENV WORLD_DIR=${CMAKE_CURRENT_SOURCE_DIR}/worlds COURSE=${CMAKE_CURRENT_SOURCE_DIR}/../../sim/course.json)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Stands up and walks in Gazebo physics, checked against ground truth.
  add_launch_test(test/test_walk_physics.py TIMEOUT 300)
endif()

ament_package()
