cmake_minimum_required(VERSION 3.0.2)
project(wildalign_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  keyframe.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)
