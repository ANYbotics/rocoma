# Project configuration
cmake_minimum_required (VERSION 2.8)
project(rocoma_test)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  rocoma
  rocoma_plugin
  message_logger
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    rocoma
    rocoma_plugin
    message_logger
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    src/BasicTests.cpp
    src/BehaviourTests.cpp
    src/test_main.cpp)

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${catkin_LIBRARIES} )
  endif()

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)

endif()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
