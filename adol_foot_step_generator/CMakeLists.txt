cmake_minimum_required(VERSION 2.8.3)
project(adol_foot_step_generator)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  std_msgs
  geometry_msgs
  message_generation
  robotis_controller_msgs
  op3_preview_walking_module_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
   FILES
   FootStepCommand.msg
   Step2D.msg
   Step2DArray.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs robotis_controller_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(adol_foot_step_generator
   src/adol_foot_step_generator.cpp
   src/message_callback.cpp
   src/main.cpp
)

add_dependencies(adol_foot_step_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(adol_foot_step_generator
   yaml-cpp
   ${catkin_LIBRARIES}
)