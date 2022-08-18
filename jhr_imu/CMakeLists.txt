cmake_minimum_required(VERSION 3.0.2)
project(jhr_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  WT61CTTL.msg
)

add_service_files(
  FILES
  setAccZero.srv
  setYawZero.srv
)

generate_messages(
  DEPENDENCIES
  jhr_imu
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jhr_imu
   CATKIN_DEPENDS roscpp rospy sensor_msgs tf message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
