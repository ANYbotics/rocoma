# Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
# Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required (VERSION 2.8)

#--------------------------#
# Documentation



FIND_PACKAGE(Doxygen)
IF(DOXYGEN)

  FIND_PACKAGE(catkin REQUIRED COMPONENTS
    rocoma
    rocoma_ros
    rocoma_plugin
    rocoma_msgs
  )

  find_path (SOURCE_ROCOMA_DIR NAMES rocoma/ControllerManager.hpp PATHS ${rocoma_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_DIR}")
    message (STATUS "Found rocoma at: ${SOURCE_ROCOMA_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma! ${SOURCE_ROCOMA_DIR}")
  endif()

  find_path (SOURCE_ROCOMA_ROS_DIR NAMES rocoma_ros/ControllerManagerRos.hpp PATHS ${rocoma_ros_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_ROS_DIR}")
    message (STATUS "Found rocoma_ros at: ${SOURCE_ROCOMA_ROS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma_ros! ${SOURCE_ROCOMA_ROS_DIR}")
  endif()

  find_path (SOURCE_ROCOMA_PLUGIN_DIR NAMES rocoma_plugin/plugins/ControllerPlugin.hpp PATHS ${rocoma_plugin_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_ROCOMA_PLUGIN_DIR}")
    message (STATUS "Found rocoma_plugin at: ${SOURCE_ROCOMA_PLUGIN_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find rocoma_plugin! ${SOURCE_ROCOMA_PLUGIN_DIR}")
  endif()

  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  SET(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  SET(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_ROCOMA_DIR}/../\" \"${SOURCE_ROCOMA_ROS_DIR}/../\" \"${SOURCE_ROCOMA_PLUGIN_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")

  #SET(HTML_EXTRA_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../cheatsheet/cheatsheet_latest.pdf")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  #SET(DOXYGEN_QUIET YES)
  MAKE_DIRECTORY(${HTML_DIR})
  #SET( HAVE_DOT YES )

  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  unset(${SOURCE_ROCOMA_DIR} CACHE)
  unset(${SOURCE_ROCOMA_ROS_DIR} CACHE)
  unset(${SOURCE_ROCOMA_PLUGIN_DIR} CACHE)

ENDIF(DOXYGEN)

#-------------------------#
