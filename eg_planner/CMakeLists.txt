cmake_minimum_required(VERSION 3.0.2)
project(eg_planner)

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

    eg_navigation
)
find_package(catkin REQUIRED COMPONENTS
${CATKIN_DEPENDS}
)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES eg_planner
   CATKIN_DEPENDS ${CATKIN_DEPENDS}
   DEPENDS system_lib
)
add_library(eg_planner
  src/cubic_spline1d.cpp
  src/cubic_spline2d.cpp
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



add_executable(astar_planning_node   src/astar_planning_node.cpp)
target_link_libraries(astar_planning_node
  ${catkin_LIBRARIES}
)

add_executable(lane_planner   src/lane_planner.cpp)
target_link_libraries(lane_planner
  ${catkin_LIBRARIES}
)

add_executable(dwa_path_plan   src/dwa_path_plan.cpp)
target_link_libraries(dwa_path_plan
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)