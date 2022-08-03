cmake_minimum_required(VERSION 3.0.2)
project(structured_testing)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS src/start_sim.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
