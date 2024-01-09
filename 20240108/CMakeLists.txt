cmake_minimum_required(VERSION 3.0.2)
project(my_second_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_second_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(std_msgs_talker_node src/std_msgs_talker.cpp)
add_dependencies(std_msgs_talker_node ${std_msgs_talker_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_msgs_talker_node ${catkin_LIBRARIES})

add_executable(std_msgs_listener_node src/std_msgs_listener.cpp)
add_dependencies(std_msgs_listener_node ${std_msgs_listener_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_msgs_listener_node ${catkin_LIBRARIES})
