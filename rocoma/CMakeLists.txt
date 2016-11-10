cmake_minimum_required(VERSION 2.8.3)
project(rocoma)

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(catkin REQUIRED
  COMPONENTS
    roco
    any_worker
    message_logger
    signal_logger
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
    any_worker
    message_logger
    signal_logger
  DEPENDS boost
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ControllerManager.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
