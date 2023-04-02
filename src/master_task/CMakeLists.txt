cmake_minimum_required(VERSION 3.8)
project(master_task)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(./include/)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(master_task src/master_task_main.cpp src/master_task.cpp)
target_include_directories(master_task
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(master_task ${colcon_LIBRARIES})
ament_target_dependencies(master_task rclcpp common std_msgs)

install(TARGETS
  master_task
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_master_task test/test_master_task.cpp src/master_task.cpp)
  target_link_libraries(test_master_task ${colcon_LIBRARIES})
  ament_target_dependencies(test_master_task rclcpp common std_msgs)


endif()

ament_package()