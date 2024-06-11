
cmake_minimum_required(VERSION 3.0.2)
project(wmm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  kortex_driver
  Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wmm_control
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs kortex_driver
)



## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MainController.cpp
  src/ControlHelper.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wmm_control_node src/MainController.cpp src/ControlHelper.cpp)
target_link_libraries(wmm_control_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES})

install(TARGETS wmm_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)