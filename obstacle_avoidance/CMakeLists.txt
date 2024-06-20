cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  object_detection
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs object_detection
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
