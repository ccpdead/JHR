cmake_minimum_required(VERSION 2.8.3)
project(jhr_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jhr_navigation
#  CATKIN_DEPENDS geometry_msgs move_base_msgs roscpp rospy actionlib
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_test src/move_test.cpp)
target_link_libraries(move_test ${catkin_LIBRARIES})
