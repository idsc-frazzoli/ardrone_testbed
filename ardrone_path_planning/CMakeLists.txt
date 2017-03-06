cmake_minimum_required(VERSION 2.8.3)
project(ardrone_path_planning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug


# 1) Find the packages needed
find_package(catkin REQUIRED COMPONENTS 
             roslib 
             roscpp 
             std_msgs 
             geometry_msgs 
             tf
             )

# 2) Add the include directories -- must come after find_package
include_directories(
    ${catkin_INCLUDE_DIRS} 
    include 
    include/glc_lib
    )

# 3) Tell catkin the include dirs between packages
catkin_package(
  INCLUDE_DIRS include include/glc_lib
  CATKIN_DEPENDS roscpp std_msgs
  )
  
# 4) Add the package executables
add_executable(motion_planner_node src/glc_planner.cpp)

# 5) Link the code
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})
