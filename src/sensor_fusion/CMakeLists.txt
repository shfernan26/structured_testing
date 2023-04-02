cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

SET(SENSOR_FUSION_VERBOSITY ROSCONSOLE_SEVERITY_DEBUG)
SET(SENSOR_FUSION_TEST_VERBOSITY ROSCONSOLE_SEVERITY_WARN)

set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(./include/)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(
    data_association
    src/data_association_functions.cpp
    src/data_association_node.cpp
)
target_include_directories(data_association
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(data_association ${colcon_LIBRARIES})
ament_target_dependencies(data_association rclcpp rclpy common std_msgs)

add_executable(
    environment_state
    src/env_state_functions.cpp
    src/env_state_node.cpp
    src/object_state.cpp
)
target_include_directories(environment_state
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(environment_state ${colcon_LIBRARIES})
ament_target_dependencies(environment_state rclcpp rclpy common std_msgs)


install(TARGETS
  data_association
  environment_state
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_data_association test/test_data_association.cpp src/data_association_functions.cpp)
  target_link_libraries(test_data_association ${colcon_LIBRARIES})
  ament_target_dependencies(test_data_association rclcpp common std_msgs)

  ament_add_gtest(test_env_state test/test_env_state.cpp src/env_state_functions.cpp src/object_state.cpp)
  target_link_libraries(test_env_state ${colcon_LIBRARIES})
  ament_target_dependencies(test_env_state rclcpp common std_msgs)


endif()

ament_package()
