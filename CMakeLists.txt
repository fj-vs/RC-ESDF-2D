cmake_minimum_required(VERSION 3.10)
project(rc_esdf_global_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
)

find_path(EIGEN3_INCLUDE_DIR Eigen/Core PATHS /usr/include/eigen3 /usr/local/include/eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen headers not found (expected Eigen/Core).")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES esdf_global_planner
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(esdf_global_planner
  src/global_esdf_planner.cpp
  src/nmpc_controller.cpp
)

target_link_libraries(esdf_global_planner
  ${catkin_LIBRARIES}
)

add_executable(esdf_bspline_planner_node
  main.cpp
)

target_link_libraries(esdf_bspline_planner_node
  esdf_global_planner
  ${catkin_LIBRARIES}
)

install(TARGETS esdf_global_planner esdf_bspline_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
