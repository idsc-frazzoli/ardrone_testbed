cmake_minimum_required(VERSION 2.8.3)
project(glc_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug


set(glc_includes ${CMAKE_SOURCE_DIR}/include)
include_directories(${glc_includes})

add_executable(glc_lib main.cpp)

install(TARGETS glc_lib RUNTIME DESTINATION bin)

add_subdirectory(include)
add_subdirectory(config)