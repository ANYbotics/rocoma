cmake_minimum_required(VERSION 2.8.3)
project(rocoma_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    roco
    rocoma
    rocoma_ros
    rocoma_plugin
    rocoma_msgs
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      roco
      rocoma
      rocoma_ros
      rocoma_plugin
      rocoma_msgs
    DEPENDS doxygen
  )

  # Find roco for documentation
  find_path (SOURCE_ROCO_DIR NAMES roco/controllers/Controller.hpp PATHS ${roco_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCO_DIR}")
    message (STATUS "Found roco at: ${SOURCE_ROCO_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find roco! ${SOURCE_ROCO_DIR}")
  endif()

  # Find rocoma for documentation
  find_path (SOURCE_ROCOMA_DIR NAMES rocoma/ControllerManager.hpp PATHS ${rocoma_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_DIR}")
    message (STATUS "Found rocoma at: ${SOURCE_ROCOMA_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma! ${SOURCE_ROCOMA_DIR}")
  endif()

  # Find rocoma_ros for documentation
  find_path (SOURCE_ROCOMA_ROS_DIR NAMES rocoma_ros/ControllerManagerRos.hpp PATHS ${rocoma_ros_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_ROS_DIR}")
    message (STATUS "Found rocoma_ros at: ${SOURCE_ROCOMA_ROS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma_ros! ${SOURCE_ROCOMA_ROS_DIR}")
  endif()

  # Find rocoma_plugin for documentation
  find_path (SOURCE_ROCOMA_PLUGIN_DIR NAMES rocoma_plugin/plugins/ControllerPlugin.hpp PATHS ${rocoma_plugin_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_PLUGIN_DIR}")
    message (STATUS "Found rocoma_plugin at: ${SOURCE_ROCOMA_PLUGIN_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma_plugin! ${SOURCE_ROCOMA_PLUGIN_DIR}")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_ROCO_DIR}/../\" \"${SOURCE_ROCOMA_DIR}/../\" \"${SOURCE_ROCOMA_ROS_DIR}/../\" \"${SOURCE_ROCOMA_PLUGIN_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${SOURCE_ROCO_DIR} CACHE)
  unset(${SOURCE_ROCOMA_DIR} CACHE)
  unset(${SOURCE_ROCOMA_ROS_DIR} CACHE)
  unset(${SOURCE_ROCOMA_PLUGIN_DIR} CACHE)
ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
