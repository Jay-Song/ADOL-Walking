################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(adol_preview_walking_module_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  PoseXYZRPY.msg
  PoseZRPY.msg
  StepTimeData.msg
  StepPositionData.msg
  StepData.msg
  BalanceParam.msg
  DampingBalanceParam.msg
  RobotPose.msg
  JointFeedBackGain.msg
  WalkingJointStatesStamped.msg
)

add_service_files(
  FILES
  IsRunning.srv
  SetBalanceParam.srv
  SetDampingBalanceParam.srv
  GetReferenceStepData.srv
  AddStepDataArray.srv
  StartWalking.srv
  RemoveExistingStepData.srv
  SetJointFeedBackGain.srv
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
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
