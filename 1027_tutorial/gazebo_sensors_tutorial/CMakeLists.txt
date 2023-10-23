cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sensors_tutorial)

add_definitions(-std=c++11 -Wall -Wextra -Wdeprecated)#-Wpedantic -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  aruco_ros
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  message_generation
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   getpiecepose.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    aruco_ros
    std_msgs
    geometry_msgs
    message_runtime
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(aruco_frames_detection src/aruco_frames_detection.cpp)
target_link_libraries(aruco_frames_detection ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(perception src/perception.cpp)
add_dependencies(perception ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(perception ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
