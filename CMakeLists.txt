cmake_minimum_required(VERSION 3.0.2)
project(imitate_mocap_repub)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imitate_mocap_repub
  CATKIN_DEPENDS roscpp tf2_msgs
)

add_executable(imitate_mocap_repub_node src/MotionRepub.cpp)
target_link_libraries(imitate_mocap_repub_node ${catkin_LIBRARIES})
add_dependencies(imitate_mocap_repub_node ${catkin_EXPORTED_TARGETS})
