cmake_minimum_required(VERSION 2.8.3)
project(rocoma_ros)

find_package(catkin REQUIRED
  roscpp
  roco
  rocoma
  rocoma_msgs
  rocoma_plugin
  any_msgs
  pluginlib
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
  roscpp
  roco
  rocoma
  rocoma_msgs
  rocoma_plugin
  any_msgs
  pluginlib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ControllerManagerRos.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)