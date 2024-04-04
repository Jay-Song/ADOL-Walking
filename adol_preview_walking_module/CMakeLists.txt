################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(adol_preview_walking_module)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  robotis_framework_common
  robotis_controller_msgs
  adol_preview_walking_pattern_generator
  #heroehs_pd_balance_controller
  op3_kinematics_dynamics
  #alice_walking_module_msgs
)
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
#  INCLUDE_DIRS include
#  LIBRARIES adol_preview_walking_module
#  CATKIN_DEPENDS roscpp heroehs_online_walking_pattern_generator heroehs_pd_balance_controller alice_kinematics_dynamics
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/adol_preview_walking.cpp
  #src/online_walking_module.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
