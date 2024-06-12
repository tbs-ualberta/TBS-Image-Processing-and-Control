
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
  src/ControlHelper.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wmm_control_node src/MainController.cpp)
target_link_libraries(wmm_control_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES})

add_executable(test_base_node src/Test_base.cpp)
add_dependencies(test_base_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_base_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS wmm_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)