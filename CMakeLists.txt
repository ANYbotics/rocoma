cmake_minimum_required(VERSION 2.8.3)
project(rocoma)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(catkin REQUIRED
  COMPONENTS
    roco
    message_logger
    any_worker
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roco
    message_logger
    any_worker
  DEPENDS boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

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
