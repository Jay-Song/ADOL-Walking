cmake_minimum_required(VERSION 3.0.2)
project(adol_walking_tester)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller_msgs
  adol_foot_step_generator
  adol_preview_walking_module_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES adol_walking_tester
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(adol_walking_tester_terminal src/adol_walking_tester_terminal.cpp)

add_dependencies(adol_walking_tester_terminal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(adol_walking_tester_terminal
  ${catkin_LIBRARIES}
)
