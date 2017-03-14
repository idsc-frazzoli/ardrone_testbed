cmake_minimum_required(VERSION 2.8.3)
project(ardrone_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  geometry_msgs
  tf
  message_generation
  sensor_fusion_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
 )

catkin_package(
  INCLUDE_DIRS include include/glc_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_fusion_comm message_runtime
)

# include_directories(${catkin_INCLUDE_DIRS})
add_executable(controller_node src/controller_node.cpp)
add_executable(imu_calibration_node src/imu_calibration_node.cpp)
#add_executable(scale_estimator src/scale_estimator.cpp)

target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(imu_calibration_node ${catkin_LIBRARIES})
#target_link_libraries(scale_estimator ${catkin_LIBRARIES})

