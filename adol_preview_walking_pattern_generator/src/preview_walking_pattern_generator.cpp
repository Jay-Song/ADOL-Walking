/*
 * online_walking_pattern_generator.cpp
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */


#include "adol_preview_walking_pattern_generator/preview_walking_pattern_generator.h"

using namespace adol;

PreviewWalkingPatternGenerator::PreviewWalkingPatternGenerator()
{
  current_balancing_index_ = 0;
  switching_ratio_ = 0;
}

PreviewWalkingPatternGenerator::~PreviewWalkingPatternGenerator()
{ }

void PreviewWalkingPatternGenerator::initialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
  ep_calculator_.initialize(lipm_height_m, preview_time_sec, control_time_sec);
  process();
}

void PreviewWalkingPatternGenerator::process()
{
  ep_calculator_.calcDesiredPose();

  switching_ratio_ = ep_calculator_.switching_ratio_;

  current_balancing_index_ = ep_calculator_.current_balancing_index_;

  x_lipm_ = ep_calculator_.x_lipm_;
  y_lipm_ = ep_calculator_.y_lipm_;

  mat_g_to_rfoot_  = robotis_framework::getTransformationXYZRPY(ep_calculator_.present_right_foot_pose_.x, ep_calculator_.present_right_foot_pose_.y, ep_calculator_.present_right_foot_pose_.z,
      ep_calculator_.present_right_foot_pose_.roll, ep_calculator_.present_right_foot_pose_.pitch, ep_calculator_.present_right_foot_pose_.yaw);
  mat_g_to_lfoot_  = robotis_framework::getTransformationXYZRPY(ep_calculator_.present_left_foot_pose_.x, ep_calculator_.present_left_foot_pose_.y, ep_calculator_.present_left_foot_pose_.z,
      ep_calculator_.present_left_foot_pose_.roll, ep_calculator_.present_left_foot_pose_.pitch, ep_calculator_.present_left_foot_pose_.yaw);
  mat_g_to_pelvis_ = robotis_framework::getTransformationXYZRPY(ep_calculator_.present_body_pose_.x, ep_calculator_.present_body_pose_.y, ep_calculator_.present_body_pose_.z,
      ep_calculator_.present_body_pose_.roll, ep_calculator_.present_body_pose_.pitch, ep_calculator_.present_body_pose_.yaw);

  pose_g_to_pelvis_ = ep_calculator_.present_body_pose_;
  mat_pelvis_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_pelvis_);

  mat_robot_to_pelvis_ = robotis_framework::getRotation4d(ep_calculator_.present_body_pose_.roll, ep_calculator_.present_body_pose_.pitch, 0);
  mat_pelvis_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_pelvis_);

  mat_g_to_robot_ = mat_g_to_pelvis_* mat_pelvis_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);
}

void PreviewWalkingPatternGenerator::start()
{
  ep_calculator_.start();
}

void PreviewWalkingPatternGenerator::stop()
{

}

bool PreviewWalkingPatternGenerator::isRunning()
{
  return ep_calculator_.isRunning();
}

void PreviewWalkingPatternGenerator::addStepData(robotis_framework::StepData& step_data)
{
  ep_calculator_.addStepData(step_data);
}

void PreviewWalkingPatternGenerator::eraseLastStepData()
{
  ep_calculator_.eraseLastStepData();
}

int  PreviewWalkingPatternGenerator::getNumofRemainingUnreservedStepData()
{
  return ep_calculator_.getNumofRemainingUnreservedStepData();
}

void PreviewWalkingPatternGenerator::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  ep_calculator_.getReferenceStepDatafotAddition(ref_step_data_for_addition);
}

void PreviewWalkingPatternGenerator::setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis)
{
  ep_calculator_.setInitialPose(r_foot, l_foot, pelvis);
  mat_g_to_rfoot_  = robotis_framework::getTransformationXYZRPY(r_foot.x, r_foot.y, r_foot.z, r_foot.roll, r_foot.pitch, r_foot.yaw);
  mat_g_to_lfoot_  = robotis_framework::getTransformationXYZRPY(l_foot.x, l_foot.y, l_foot.z, l_foot.roll, l_foot.pitch, l_foot.yaw);
  mat_g_to_pelvis_ = robotis_framework::getTransformationXYZRPY(pelvis.x, pelvis.y, pelvis.z, pelvis.roll, pelvis.pitch, pelvis.yaw);

  mat_pelvis_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_pelvis_);

  mat_robot_to_pelvis_ = robotis_framework::getRotation4d(pelvis.roll, pelvis.pitch, 0);
  mat_pelvis_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_pelvis_);

  mat_g_to_robot_ = mat_g_to_pelvis_* mat_pelvis_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);
}


