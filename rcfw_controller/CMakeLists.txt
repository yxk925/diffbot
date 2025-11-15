cmake_minimum_required(VERSION 2.8.3)
project(rcfw_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/rcfw_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
