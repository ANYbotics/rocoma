#-------------------------------------------------------------------------------
#
# Author(s): Christian Gehring, Stelian Coros
# Date     : 1-Dec-2014
#
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8.3)
project(locomotion_controller)

# Activate C++11.
set(CMAKE_CXX_FLAGS "-std=c++11")

# Exclude dynamics in robot model.
if (NOT RM_EXCLUDE_DYNAMICS) 
add_definitions(-DRM_EXCLUDE_DYNAMICS) 
endif()

# Play audio when activated.
if (PLAY_AUDIO) 
add_definitions(-DPLAY_AUDIO) 
endif()

# Indicate that ROS is enabled
set(USE_ROS ON CACHE BOOL "use ros") 
add_definitions(-DUSE_ROS) 

# This is a list of all catkin packages that depend on roco and which can be enabled or disabled with roco_assembly.
set(CATKIN_ROCO_PKGS)
set(CATKIN_QUADRUPED_PKGS)

# This will fill ${CATKIN_ROCO_PKGS} with the enabled roco packages.
find_package(catkin REQUIRED COMPONENTS roco_assembly quadruped_assembly)

include(${roco_assembly_SOURCE_PREFIX}/cmake/roco_ros.cmake)

find_package(catkin REQUIRED
  COMPONENTS
    any_msgs
    locomotion_controller_msgs
    nodelet
    parameter_handler_ros
    quadruped_assembly
    quadruped_model
    quadruped_msgs
    robot_utils
    roco
    roco_assembly
    roco_freeze
    roscpp
    roscpp_nodewrap
    series_elastic_actuator_ros
    signal_logger
    signal_logger_ros
    signal_logger_std
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

if (USE_ANYMAL_DESCRIPTION)
    add_definitions(-DUSE_ANYMAL_DESCRIPTION)
endif(USE_ANYMAL_DESCRIPTION)

if (USE_STARLETH_DESCRIPTION)
    add_definitions(-DUSE_STARLETH_DESCRIPTION)
endif(USE_STARLETH_DESCRIPTION)

catkin_package(
  INCLUDE_DIRS
    include 
  LIBRARIES
    locomotion_controller
    model
  CATKIN_DEPENDS
    any_msgs
    locomotion_controller_msgs
    nodelet
    parameter_handler_ros
    quadruped_assembly
    quadruped_model
    quadruped_msgs
    robot_utils
    roco
    roco_assembly
    roco_freeze
    roscpp
    roscpp_nodewrap
    series_elastic_actuator_ros
    signal_logger
    signal_logger_ros
    signal_logger_std
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

# This library includes the robot model and the terrain model.
add_library(model
  src/Model.cpp
)
target_link_libraries(model
   ${catkin_LIBRARIES}
)

# This library provides the locomotion controller.
add_library(locomotion_controller
  src/ControllerManager.cpp
  src/add_locomotion_controllers.cpp
  src/LocomotionController.cpp
)
add_dependencies(locomotion_controller
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(locomotion_controller
  model
  ${catkin_LIBRARIES}
)

add_executable(locomotion_controller_node
  src/locomotion_controller_node.cpp
)
add_dependencies(locomotion_controller_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(locomotion_controller_node
  model
  locomotion_controller
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS locomotion_controller locomotion_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_locomotion_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
