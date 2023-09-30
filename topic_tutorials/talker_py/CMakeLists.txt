cmake_minimum_required(VERSION 2.8.3)
project(talker_py)


catkin_package(CATKIN_DEPENDS std_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/talker
  scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
install(FILES
  launch/talker.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)

