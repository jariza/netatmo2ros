cmake_minimum_required(VERSION 2.8.3)
project(netatmo2ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Weatherdata.msg
  WeatherdataArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

