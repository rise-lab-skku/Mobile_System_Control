cmake_minimum_required(VERSION 3.0.2)
project(MPC_control_ex)

set(OsqpEigen_DIR ${CMAKE_SOURCE_DIR}/External/osqp-eigen/lib/cmake/OsqpEigen)
set(osqp_DIR ${CMAKE_SOURCE_DIR}/External/osqp/lib/cmake/osqp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  OsqpEigen
  tf
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(MPC_control_ex_node
  src/MPC_controller.cpp
  src/main.cpp
)
target_link_libraries(MPC_control_ex_node 
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)