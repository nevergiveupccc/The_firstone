cmake_minimum_required(VERSION 3.0)

project(SOLIDERGUN)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall" )

set(CMAKE_BUILD_TYPE Debug)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(main_cmake main.cpp src/Gun.cpp src/Solider.cpp)