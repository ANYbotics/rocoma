cmake_minimum_required(VERSION 2.8.3)
project(locomotion_controller)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  sensor_msgs
  starleth_msgs
  std_msgs
  nodelet
  roscpp_nodewrap
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
add_service_files(
   FILES
   SwitchController.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
#   geometry_msgs#   sensor_msgs#   starleth_msgs#   std_msgs
)

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
 LIBRARIES locomotion_controller
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs starleth_msgs std_msgs roscpp_nodewrap
#  DEPENDS system_lib
)

###########
## Build ##
###########


# Exclude dynamics in robot model
if (NOT RM_EXCLUDE_DYNAMICS) 
add_definitions( -DRM_EXCLUDE_DYNAMICS) 
endif()
add_definitions( -DDISPLAY)
add_definitions( -DAPP_STARLETH)
add_definitions( -DSTANDALONE)
add_definitions( -DSIMANDCONTROL_CMAKEBUILD)
add_definitions( -DSIMANDCONTROL_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}" )
add_definitions( -Wwrite-strings)
set(ARCH x86_64)
set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/locomotion_controllers.cmake)

#set(EXCLUDE_ROBOTBASE ON CACHE BOOL "exclude robot base")

add_subdirectory($ENV{LAB_ROOT}/starlethTask starlethTask)
set(EXTRA_LIBS ${EXTRA_LIBS} starlethTask starlethRobotModel robotUtils ${TASK_LIBS} ${ROBOTMODEL_LIBS} ${UTILS_LIBS})
include_directories(${TASK_INCL} ${ROBOTMODEL_INCL} ${UTILS_INCL})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(model
  src/Model.cpp
)
target_link_libraries(model
   ${EXTRA_LIBS}
   ${catkin_LIBRARIES}
)


add_library(locomotion_controller
  src/ControllerManager.cpp
  src/locomotion_controllers.cpp
  src/LocomotionController.cpp
)

target_link_libraries(locomotion_controller
    model
   ${catkin_LIBRARIES}
   ${EXTRA_LIBS}
)

add_executable(locomotion_controller_node src/locomotion_controller_node.cpp)

target_link_libraries(locomotion_controller_node
   model
   locomotion_controller
   ${catkin_LIBRARIES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(locomotion_controller_node locomotion_controller_generate_messages_cpp)


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
