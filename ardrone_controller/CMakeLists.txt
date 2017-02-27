cmake_minimum_required(VERSION 2.8.3)
project(ardrone_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug


set(glc_includes ${CMAKE_SOURCE_DIR}/include/glc_lib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  tf
)


catkin_package(INCLUDE_DIRS include/glc_lib)

include_directories(${glc_includes})
include_directories(include/glc_lib)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller_node src/controller_node.cpp)
add_executable(imu_calibration_node src/imu_calibration_node.cpp)
add_executable(motion_planner_node src/glc_planner.cpp)

target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(imu_calibration_node ${catkin_LIBRARIES})
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})

