cmake_minimum_required(VERSION 3.8)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/FilteredObjectMsg.msg
  msg/CanCommsDataMsg.msg
  msg/BinaryClassMsg.msg
  msg/DriveCtrlInputMsg.msg
  msg/MobileyeObjectDataMsg.msg
  msg/ObjectDeletionMsg.msg
  msg/RadarObjectDataMsg.msg
  msg/RawSensorObjectDataMsg.msg
  msg/RawLaneDataMsg.msg
  msg/SudoDriverInputMsg.msg
  msg/TrackedOutputMsg.msg
  msg/TargetOutputMsg.msg
  msg/AssociatedMeMsg.msg
  msg/AssociatedRadarMsg.msg
  msg/AssociatedObjectMsg.msg
  msg/AccOutputMsg.msg
  msg/AccInputMsg.msg
  msg/SensorDiagnosticDataMsg.msg
  msg/V2xDataMsg.msg
  msg/IntxnInfoMsg.msg
  msg/UnfilteredIntxnDataMsg.msg
  srv/SensorDiagnosticFlagCh2.srv
  srv/SensorDiagnosticFlagCh3.srv
  srv/SensorDiagnosticFlagCh4.srv
  srv/EnvStateSrv.srv
  DEPENDENCIES std_msgs
)

install(
  FILES ros1_mapping.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
