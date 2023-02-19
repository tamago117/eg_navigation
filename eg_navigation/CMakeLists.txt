cmake_minimum_required(VERSION 3.1.3)
project(eg_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
${CATKIN_DEPENDS}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eg_navigation
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(eg_navigation
  src/tf_position.cpp
  src/robot_status.cpp
)

#add_library(tf_position
#  src/tf_position.cpp
#)

add_executable(twist_maneger   src/twist_maneger.cpp)
target_link_libraries(twist_maneger
  ${catkin_LIBRARIES}
)

add_executable(state_maneger   src/state_maneger.cpp)
target_link_libraries(state_maneger
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)