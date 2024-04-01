################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(adol_preview_walking_pattern_generator)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  robotis_math
)

find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adol_preview_walking_pattern_generator
  CATKIN_DEPENDS roscpp robotis_math 
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(adol_preview_walking_pattern_generator
  src/online_pelvis_xy_calculator.cpp
  src/online_endpoint_calculator.cpp
  src/preview_gains_calculator.cpp
  src/online_walking_pattern_generator.cpp
)

add_dependencies(adol_preview_walking_pattern_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(adol_preview_walking_pattern_generator
  ${catkin_LIBRARIES}
)