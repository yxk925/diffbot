cmake_minimum_required(VERSION 3.0.2)
project(person_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES person_follower
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/person_follower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
