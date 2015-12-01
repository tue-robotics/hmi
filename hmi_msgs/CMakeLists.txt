cmake_minimum_required(VERSION 2.8.3)
project(hmi_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
)

add_message_files(
  FILES
    Choice.msg
    Result.msg
)

add_action_files(
  FILES
    HumanInteraction.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hmi_msgs
#  CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)

# add_dependencies(hmi_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hmi_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
