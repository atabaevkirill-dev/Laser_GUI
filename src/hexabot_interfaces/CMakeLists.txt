cmake_minimum_required(VERSION 3.16)
project(hexabot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyPose.msg"
  "msg/LocomotionState.msg"
  "msg/LrfMeasurement.msg"
  "msg/LrfStatus.msg"
  "msg/IlluminatorState.msg"
  "msg/ScoutEvent.msg"
  "msg/TargetReport.msg"
  "msg/TerrainInfo.msg"
  "msg/VisionState.msg"
  "srv/SetGait.srv"
  "srv/SetMode.srv"
  "srv/LrfMeasure.srv"
  "srv/LrfConfigure.srv"
  "srv/SetIlluminator.srv"
  "srv/SetVisionMode.srv"
  "srv/SelectTarget.srv"
  "action/AimAndRange.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
