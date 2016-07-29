cmake_minimum_required(VERSION 2.8.3)
project(rocoma_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    roco
    rocoma
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roco
    rocoma
  DEPENDS
)