cmake_minimum_required(VERSION 3.0.2)
project(wmm_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  turtlesim
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)