cmake_minimum_required(VERSION 2.8.3)
project(ardrone_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  tf
  message_generation
  sensor_fusion_comm
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ardrone_controller
  CATKIN_DEPENDS roscpp std_msgs sensor_fusion_comm message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_message_files(FILES DoubleArrayStamped.msg)

add_executable(controller_node src/controller_node.cpp)
add_executable(imu_calibration_node src/imu_calibration_node.cpp)

target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(imu_calibration_node ${catkin_LIBRARIES})
