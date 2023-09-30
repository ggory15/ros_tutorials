cmake_minimum_required(VERSION 2.8.3)
project(add_two_ints_server_py)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)


catkin_install_python(PROGRAMS
  scripts/add_two_ints_server
  scripts/add_two_ints_server.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
install(FILES
  launch/add_two_ints_server.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)

