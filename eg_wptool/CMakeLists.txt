cmake_minimum_required(VERSION 3.0.2)
project(eg_wptool)

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
    pcl_ros

    eg_navigation
    eg_planner
)

find_package(catkin REQUIRED COMPONENTS
${CATKIN_DEPENDS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eg_wptool
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



add_executable(wpRecord   src/wpRecord.cpp)
target_link_libraries(wpRecord
  ${catkin_LIBRARIES}
)

add_executable(csvOutput   src/csv_out.cpp)
target_link_libraries(csvOutput
  ${catkin_LIBRARIES}
)

add_executable(wpLoad   src/wpLoad.cpp)
target_link_libraries(wpLoad
  ${catkin_LIBRARIES}
)

add_executable(wpVisualizer   src/wpVisualizer.cpp)
target_link_libraries(wpVisualizer
  ${catkin_LIBRARIES}
)

add_executable(wpControl   src/wpControl.cpp)
target_link_libraries(wpControl
  ${catkin_LIBRARIES}
)

add_executable(wpNearSearch   src/wpNearSearch.cpp)
target_link_libraries(wpNearSearch
  ${catkin_LIBRARIES}
)

add_executable(wpInitialPose   src/wpInitialPose.cpp)
target_link_libraries(wpInitialPose
  ${catkin_LIBRARIES}
)

