cmake_minimum_required(VERSION 3.5)
project(hmi_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
    Query.action
    Cast.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime
)
