cmake_minimum_required(VERSION 2.8.3)
project(locomotion_controller)

set(CMAKE_CXX_FLAGS "-std=c++11")


# Exclude dynamics in robot model
if (NOT RM_EXCLUDE_DYNAMICS) 
add_definitions( -DRM_EXCLUDE_DYNAMICS) 
endif()
if (PLAY_AUDIO) 
add_definitions( -DPLAY_AUDIO) 
endif()

set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")
add_definitions(-DUSE_ROS) 
set(USE_ROS ON CACHE BOOL "use ros") 

find_package(catkin REQUIRED COMPONENTS roco_assembly)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  roscpp_nodewrap
  starleth_description
  locomotion_controller_msgs
  roco
  robot_utils
  robot_utils_ros
  robot_model
  roco_freeze
  roco_assembly
  ${CATKIN_ROCO_PKGS}
)




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Kindr REQUIRED)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs#   sensor_msgs#   starleth_msgs#   std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES locomotion_controller model
 CATKIN_DEPENDS roscpp_nodewrap locomotion_controller_msgs starleth_description roco robot_utils robot_utils_ros robot_model roco_freeze
# DEPENDS system_lib
)

###########
## Build ##
###########




#include(tasks_add_definitions.cmake)
#add_subdirectory($ENV{LAB_ROOT}/starlethTask starlethTask)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Kindr_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
#include_directories($ENV{LAB_ROOT}/starlethTask/include)
#include_directories(${TASK_INCL})



add_library(model
  src/Model.cpp
)
target_link_libraries(model
   ${catkin_LIBRARIES}
)


add_library(locomotion_controller
  src/ControllerManager.cpp
  src/add_locomotion_controllers.cpp
  src/LocomotionController.cpp
)

add_dependencies(locomotion_controller locomotion_controller_msgs_generate_messages_cpp roco robot_utils robot_utils_ros robot_model roco_freeze)


target_link_libraries(locomotion_controller
    model
   ${catkin_LIBRARIES}
)

add_executable(locomotion_controller_node src/locomotion_controller_node.cpp)

target_link_libraries(locomotion_controller_node
   model
   locomotion_controller
   ${catkin_LIBRARIES}
)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(locomotion_controller_node locomotion_controller_generate_messages_cpp)


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
