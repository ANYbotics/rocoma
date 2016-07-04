cmake_minimum_required(VERSION 2.8.3)
project(rocoma)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(catkin REQUIRED
  COMPONENTS
    roco
    message_logger
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS
    include 
  LIBRARIES
  CATKIN_DEPENDS
    roco
    message_logger
  DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

add_library(coma
  src/ControllerManager.cpp
)
add_dependencies(coma
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(coma
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)