cmake_minimum_required(VERSION 3.0.2)
project(hmi_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_action_files(
  FILES
    Query.action
    Cast.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs
)
