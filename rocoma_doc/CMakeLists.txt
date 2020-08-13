cmake_minimum_required(VERSION 3.5.1)
project(rocoma_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    roco
    roco_ros
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
      roco_ros
      rocoma
      rocoma_ros
      rocoma_plugin
      rocoma_msgs
    DEPENDS doxygen
  )

  # Set some environment variables
  set(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../rocoma/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../rocoma_plugin/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../rocoma_ros/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../../roco/roco/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../../roco/roco_ros/\"")
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen.
  add_custom_target(
    ${PROJECT_NAME} ALL
    COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config
    WORKING_DIRECTORY ${DOCSOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating API documentation."
    VERBATIM
  )
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
