cmake_minimum_required(VERSION 3.1)
project(kinect_pub)

# Find pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFREENECT2 REQUIRED freenect2)

# Include directories for libfreenect2
include_directories(${LIBFREENECT2_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR})
include_directories(/usr/local/include/libfreenect2)

# Link directories for libfreenect2
link_directories(${LIBFREENECT2_LIBRARY_DIRS})
link_directories(/usr/local/lib)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_generation
  std_msgs
)

add_service_files(
  FILES
  GetCameraInfo.srv
)

add_message_files(
  FILES
  RegistrationData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Find PCL
find_package(PCL REQUIRED)

# Add PCL include directories
include_directories(${PCL_INCLUDE_DIRS})

# Link PCL libraries
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBFREENECT2_INCLUDE_DIRS}
)

add_executable(kinect_pub_node src/kinect_publisher_node.cpp) # names the executable (for rosrun use kinect_pub_node)
target_link_libraries(kinect_pub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/local/lib/libfreenect2.so  # Directly specify the full path to the library
)