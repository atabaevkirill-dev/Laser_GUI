cmake_minimum_required(VERSION 3.16)
project(hexabot_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    ENV DESCRIPTION_DIR=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
