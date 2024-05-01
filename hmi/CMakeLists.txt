cmake_minimum_required(VERSION 3.0.2)
project(hmi)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test(-W2)

  catkin_add_nosetests(test)
endif()
