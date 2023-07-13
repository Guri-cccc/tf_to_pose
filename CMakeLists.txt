cmake_minimum_required(VERSION 3.0.2)
project(tf_to_pose)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf
  fast_tf
  pose_tf
)

catkin_package(
  LIBRARIES fast_tf_lib pose_tf_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_to_pose_node src/tf_to_pose.cpp)
add_dependencies(tf_to_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_to_pose_node
  ${catkin_LIBRARIES}
  fast_tf_lib
  pose_tf_lib
  yaml-cpp
)