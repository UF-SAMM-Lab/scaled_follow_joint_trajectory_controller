cmake_minimum_required(VERSION 3.0.2)
project(scaled_fjt_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  cnr_logger
  roscpp
  trajectory_msgs
  rosdyn_core
  name_sorting
  controller_interface
  cnr_controller_interface
  actionlib
  control_msgs
  rosdyn_chain_state
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scaled_fjt_controller
  CATKIN_DEPENDS actionlib cnr_controller_interface control_msgs roscpp name_sorting cnr_controller_interface control_msgs controller_interface rosdyn_chain_state
  DEPENDS
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/scaled_fjt_pos_controller.cpp
   src/${PROJECT_NAME}/scaled_fjt_posveleff_controller.cpp
   src/${PROJECT_NAME}/microinterpolator.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
