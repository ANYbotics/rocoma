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
  roslib
  std_srvs
  std_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES
CATKIN_DEPENDS
  roscpp
  roco
  rocoma
  rocoma_msgs
  rocoma_plugin
  any_msgs
  pluginlib
  roslib
  std_srvs
  std_msgs
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

