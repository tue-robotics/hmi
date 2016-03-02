cmake_minimum_required(VERSION 2.8.3)
project(hmi_server)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  hmi_msgs
  rospy
)

catkin_package(
 CATKIN_DEPENDS actionlib hmi_msgs rospy
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hmi_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
