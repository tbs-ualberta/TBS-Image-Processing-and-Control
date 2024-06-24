cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

cmake_minimum_required(VERSION 3.10.2)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  kinect_pub
)

add_message_files(
  FILES
  MaskArray.msg
  MaskData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  kinect_pub
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs kinect_pub
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/img_processor_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)