cmake_minimum_required(VERSION 3.0.2)
project(hmi)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/dragonfly_restart_mock
  scripts/multi_client
  scripts/qr_code_decoder
  scripts/string_topic_answerer
  src/${PROJECT_NAME}/abstract_server.py
  tools/console_answerer
  tools/continue_gui
  tools/conversation_playback
  tools/noop_server
  tools/random_answerer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore UNKNOWN_PACKAGE")

  catkin_add_nosetests(test)
endif()
