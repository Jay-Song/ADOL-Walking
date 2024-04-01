/*
 * alice_foot_step_generator_node.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include <cmath>
#include "adol_foot_step_generator/adol_foot_step_generator.h"

using namespace adol;

#define RAD2DEG  (M_PI/180.0)

double sign(double n)
{
  if(n < 0)
    return -1;
  else if(n > 0)
    return 1;
  else
    return 0;
}

FootStepGenerator::FootStepGenerator()
{
  num_of_step_             = 2*2 + 2;
  fb_step_length_m_        = 0.1;
  rl_step_length_m_        = 0.07;
  rotate_step_angle_rad_   = 10.0*RAD2DEG;
  step_time_sec_ = 1.0;

  dsp_ratio_ = 0.2;
  foot_z_swap_m_ = 0.1;
  body_z_swap_m_ = 0.01;

  y_zmp_convergence_m_ = 0.0;

  start_end_time_sec_ = 1.6;
  default_y_feet_offset_m_ = 0.18;

  previous_step_type_ = STOP_WALKING;


  kick_height_m_  = 0.05;
  kick_far_m_     = 0.18;
  kick_pitch_rad_ = 15.0*M_PI/180.0;
  kick_time_sec_  = 1.6;
}


FootStepGenerator::~FootStepGenerator()
{    }

void FootStepGenerator::initialize()
{
  previous_step_type_ = STOP_WALKING;
  //step_data_array_.clear();
}

Eigen::MatrixXd FootStepGenerator::getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw)
{
  double sr = sin(roll), cr = cos(roll);
  double sp = sin(pitch), cp = cos(pitch);
  double sy = sin(yaw), cy = cos(yaw);

  Eigen::MatrixXd mat_roll(4,4);
  Eigen::MatrixXd mat_pitch(4,4);
  Eigen::MatrixXd mat_yaw(4,4);

  mat_roll <<
      1, 0, 0, 0,
      0, cr, -sr, 0,
      0, sr, cr, 0,
      0, 0, 0, 1;

  mat_pitch <<
      cp, 0, sp, 0,
      0, 1, 0, 0,
      -sp, 0, cp, 0,
      0, 0, 0, 1;

  mat_yaw <<
      cy, -sy, 0, 0,
      sy, cy, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::MatrixXd mat_xyzrpy = (mat_yaw*mat_pitch)*mat_roll;

  mat_xyzrpy.coeffRef(0, 3) = position_x;
  mat_xyzrpy.coeffRef(1, 3) = position_y;
  mat_xyzrpy.coeffRef(2, 3) = position_z;


  return mat_xyzrpy;
}

void FootStepGenerator::getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw)
{
  *position_x = matTransform.coeff(0, 3);
  *position_y = matTransform.coeff(1, 3);
  *position_z = matTransform.coeff(2, 3);
  *roll       = atan2( matTransform.coeff(2,1), matTransform.coeff(2,2));
  *pitch      = atan2(-matTransform.coeff(2,0), sqrt(matTransform.coeff(2,1)*matTransform.coeff(2,1) + matTransform.coeff(2,2)*matTransform.coeff(2,2)) );
  *yaw        = atan2( matTransform.coeff(1,0), matTransform.coeff(0,0));
}

adol_preview_walking_module_msgs::PoseXYZRPY FootStepGenerator::getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform)
{
	adol_preview_walking_module_msgs::PoseXYZRPY pose;

  double pose_x     = 0;
  double pose_y     = 0;
  double pose_z     = 0;
  double pose_roll  = 0;
  double pose_pitch = 0;
  double pose_yaw   = 0;

  getPosefromTransformMatrix(matTransform, &pose_x, &pose_y, &pose_z, &pose_roll, &pose_pitch, &pose_yaw);

  pose.x     = pose_x;
  pose.y     = pose_y;
  pose.z     = pose_z;
  pose.roll  = pose_roll;
  pose.pitch = pose_pitch;
  pose.yaw   = pose_yaw;

  return pose;
}

Eigen::MatrixXd FootStepGenerator::getInverseTransformation(Eigen::MatrixXd transform)
{
  // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

  Eigen::Vector3d vec_boa;
  Eigen::Vector3d vec_x, vec_y, vec_z;
  Eigen::MatrixXd inv_t(4,4);

  vec_boa(0) = -transform(0,3);
  vec_boa(1) = -transform(1,3);
  vec_boa(2) = -transform(2,3);

  vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
  vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
  vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

  inv_t <<
      vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
      vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
      vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
      0, 0, 0, 1;

  return inv_t;
}

void FootStepGenerator::getStepData(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array, const adol_preview_walking_module_msgs::StepData& ref_step_data, int desired_step_type)
{
  step_data_array->clear();
  step_data_array_.clear();

  if(calcStep(ref_step_data, previous_step_type_, desired_step_type))
  {
    previous_step_type_ = desired_step_type;
    for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
    {
      step_data_array->push_back(step_data_array_[stp_idx]);
    }
  }
  else
  {
    return;
  }
}



void FootStepGenerator::getStepDataFromStepData2DArray(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const adol_preview_walking_module_msgs::StepData& ref_step_data,
    const adol_foot_step_generator::Step2DArray::ConstPtr& request_step_2d)
{
  step_data_array->clear();

  adol_preview_walking_module_msgs::StepData stp_data;

  stp_data = ref_step_data;
  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.time_data.dsp_ratio = dsp_ratio_;
  stp_data.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  stp_data.time_data.start_time_delay_ratio_x     = 0.0;
  stp_data.time_data.start_time_delay_ratio_y     = 0.0;
  stp_data.time_data.start_time_delay_ratio_z     = 0.0;
  stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

  stp_data.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  stp_data.position_data.foot_z_swap = 0;
  stp_data.position_data.body_z_swap = 0;

  step_data_array->push_back(stp_data);

  for(unsigned int stp_idx = 0; stp_idx < request_step_2d->footsteps_2d.size(); stp_idx++)
  {
    stp_data.time_data.abs_step_time += step_time_sec_;
    stp_data.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;

    if(request_step_2d->footsteps_2d[stp_idx].moving_foot == adol_foot_step_generator::Step2D::LEFT_FOOT_SWING)
    {
      stp_data.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data.position_data.body_z_swap = body_z_swap_m_;
      stp_data.position_data.foot_z_swap = foot_z_swap_m_;
      stp_data.position_data.left_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
      stp_data.position_data.left_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
      stp_data.position_data.left_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;

    }
    else if(request_step_2d->footsteps_2d[stp_idx].moving_foot == adol_foot_step_generator::Step2D::RIGHT_FOOT_SWING)
    {
      stp_data.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data.position_data.body_z_swap = body_z_swap_m_;
      stp_data.position_data.foot_z_swap = foot_z_swap_m_;
      stp_data.position_data.right_foot_pose.x   = request_step_2d->footsteps_2d[stp_idx].step2d.x;
      stp_data.position_data.right_foot_pose.y   = request_step_2d->footsteps_2d[stp_idx].step2d.y;
      stp_data.position_data.right_foot_pose.yaw = request_step_2d->footsteps_2d[stp_idx].step2d.theta;
    }
    else
    {
      ROS_ERROR("Invalid Step2D");
      step_data_array->clear();
      return;
    }

    if(fabs(stp_data.position_data.right_foot_pose.yaw - stp_data.position_data.left_foot_pose.yaw) > M_PI)
    {
      stp_data.position_data.body_pose.yaw = 0.5*(stp_data.position_data.right_foot_pose.yaw + stp_data.position_data.left_foot_pose.yaw)
                                - sign(0.5*(stp_data.position_data.right_foot_pose.yaw - stp_data.position_data.left_foot_pose.yaw))*M_PI;
    }
    else
    {
      stp_data.position_data.body_pose.yaw = 0.5*(stp_data.position_data.right_foot_pose.yaw
          + stp_data.position_data.left_foot_pose.yaw);
    }

    step_data_array->push_back(stp_data);
  }

  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.time_data.dsp_ratio = dsp_ratio_;
  stp_data.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data.time_data.start_time_delay_ratio_x     = 0.0;
  stp_data.time_data.start_time_delay_ratio_y     = 0.0;
  stp_data.time_data.start_time_delay_ratio_z     = 0.0;
  stp_data.time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data.time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data.time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data.time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data.time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data.time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data.time_data.finish_time_advance_ratio_yaw   = 0.0;

  stp_data.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  stp_data.position_data.foot_z_swap = 0;
  stp_data.position_data.body_z_swap = 0;

  step_data_array->push_back(stp_data);
}

//
bool FootStepGenerator::calcStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type)
{
  int direction = 0;
  adol_preview_walking_module_msgs::StepData stp_data[2];

  adol_preview_walking_module_msgs::PoseXYZRPY poseGtoRF, poseGtoLF;
  adol_preview_walking_module_msgs::PoseXYZRPY poseLtoRF, poseLtoLF;

  poseGtoRF = ref_step_data.position_data.right_foot_pose;
  poseGtoLF = ref_step_data.position_data.left_foot_pose;

  Eigen::MatrixXd mat_g_to_rf = getTransformationXYZRPY(poseGtoRF.x, poseGtoRF.y, poseGtoRF.z, 0, 0, poseGtoRF.yaw);
  Eigen::MatrixXd mat_g_to_lf = getTransformationXYZRPY(poseGtoLF.x, poseGtoLF.y, poseGtoLF.z, 0, 0, poseGtoLF.yaw);

  //the local coordinate is set as below.
  //the below local does not means real local coordinate.
  //it is just for calculating step data.
  //the local coordinate will be decide by the moving foot of ref step data

  Eigen::MatrixXd mat_lf_to_local = getTransformationXYZRPY(0, -0.5*default_y_feet_offset_m_, 0, 0, 0, 0);
  Eigen::MatrixXd mat_rf_to_local = getTransformationXYZRPY(0,  0.5*default_y_feet_offset_m_, 0, 0, 0, 0);
  Eigen::MatrixXd mat_global_to_local, mat_local_to_global;

  if(ref_step_data.position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
  {
    mat_global_to_local = mat_g_to_rf*mat_rf_to_local;
    mat_local_to_global = getInverseTransformation(mat_global_to_local);
    mat_lf_to_local     = getInverseTransformation(mat_g_to_lf) * mat_global_to_local;
  }
  else
  {
    mat_global_to_local = mat_g_to_lf * mat_lf_to_local;;
    mat_local_to_global = getInverseTransformation(mat_global_to_local);
    mat_rf_to_local     = getInverseTransformation(mat_g_to_rf) * mat_global_to_local;
  }

  Eigen::MatrixXd mat_local_to_rf = mat_local_to_global * mat_g_to_rf;
  Eigen::MatrixXd mat_local_to_lf = mat_local_to_global * mat_g_to_lf;

  poseLtoRF = getPosefromTransformMatrix(mat_local_to_rf);
  poseLtoLF = getPosefromTransformMatrix(mat_local_to_lf);

  if((desired_step_type == FORWARD_WALKING) || (desired_step_type == LEFTWARD_WALKING) || (desired_step_type == LEFT_ROTATING_WALKING))
    direction = 1;
  else if((desired_step_type == BACKWARD_WALKING ) || (desired_step_type == RIGHTWARD_WALKING) || (desired_step_type == RIGHT_ROTATING_WALKING))
    direction = -1;
  else if(desired_step_type == STOP_WALKING)
    direction = 0;
  else
    return false;


  stp_data[0] = ref_step_data;
  stp_data[0].position_data.torso_yaw_angle_rad = 0.0*M_PI;

  stp_data[0].position_data.right_foot_pose = poseLtoRF;
  stp_data[0].position_data.left_foot_pose = poseLtoLF;
  stp_data[0].time_data.start_time_delay_ratio_x     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_y     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_z     = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_roll  = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_pitch = 0.0;
  stp_data[0].time_data.start_time_delay_ratio_yaw   = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_x     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_y     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_z     = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_roll  = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_pitch = 0.0;
  stp_data[0].time_data.finish_time_advance_ratio_yaw   = 0.0;


  if(stp_data[0].time_data.walking_state != adol_preview_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      calcFBStep(stp_data[0], direction);
    else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      calcRLStep(stp_data[0], direction);
    else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      calcRoStep(stp_data[0], direction);
    else if(desired_step_type == STOP_WALKING)
      calcStopStep(stp_data[0], direction);
    else
      return false;
  }
  else
  {
    if(desired_step_type != previous_step_type)
    {
      stp_data[0].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
      if((fabs(poseLtoRF.yaw - poseLtoLF.yaw) > 0)
          || (fabs(poseLtoRF.y - poseLtoLF.y) > default_y_feet_offset_m_)
          || (fabs(poseLtoRF.x - poseLtoLF.x) > 0))
      {
        stp_data[0].time_data.abs_step_time += step_time_sec_;
        if(ref_step_data.position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
        {
          stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
          stp_data[0].position_data.right_foot_pose.x   = stp_data[0].position_data.left_foot_pose.x;
          stp_data[0].position_data.right_foot_pose.y   = stp_data[0].position_data.left_foot_pose.y - default_y_feet_offset_m_;
          stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw;
        }
        else
        {
          stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
          stp_data[0].position_data.left_foot_pose.x   = stp_data[0].position_data.right_foot_pose.x;
          stp_data[0].position_data.left_foot_pose.y   = stp_data[0].position_data.right_foot_pose.y + default_y_feet_offset_m_;
          stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw;
        }
        step_data_array_.push_back(stp_data[0]);
      }

      stp_data[1] = stp_data[0];
      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING || desired_step_type == STOP_WALKING)
      {

      }
      else if(desired_step_type == LEFTWARD_WALKING || desired_step_type == LEFT_ROTATING_WALKING)
      {
        if(stp_data[0].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
        {
          stp_data[1].time_data.abs_step_time += step_time_sec_;
          stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
          step_data_array_.push_back(stp_data[1]);
        }
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == RIGHT_ROTATING_WALKING)
      {
        if(stp_data[0].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
        {
          stp_data[1].time_data.abs_step_time += step_time_sec_;
          stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
          step_data_array_.push_back(stp_data[1]);
        }
      }
      else
      {
        return false;
      }

      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      {
        calcFBStep(stp_data[1], direction);
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      {
        calcRLStep(stp_data[1], direction);
      }
      else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      {
        calcRoStep(stp_data[1], direction);
      }
      else if(desired_step_type == STOP_WALKING)
      {
        calcStopStep(stp_data[1], direction);
      }
      else
      {
        return false;
      }
    }
    else
    {
      if(desired_step_type == FORWARD_WALKING || desired_step_type == BACKWARD_WALKING )
      {
        calcFBStep(stp_data[0], direction);
      }
      else if(desired_step_type == RIGHTWARD_WALKING || desired_step_type == LEFTWARD_WALKING )
      {
        calcRLStep(stp_data[0], direction);
      }
      else if(desired_step_type == LEFT_ROTATING_WALKING || desired_step_type == RIGHT_ROTATING_WALKING )
      {
        calcRoStep(stp_data[0], direction);
      }
      else if(desired_step_type == STOP_WALKING)
      {
        calcStopStep(stp_data[0], direction);
      }
      else
      {
        return false;
      }
    }
  }


  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    Eigen::MatrixXd mat_r_foot = getTransformationXYZRPY(step_data_array_[stp_idx].position_data.right_foot_pose.x,
        step_data_array_[stp_idx].position_data.right_foot_pose.y,
        step_data_array_[stp_idx].position_data.right_foot_pose.z,
        step_data_array_[stp_idx].position_data.right_foot_pose.roll,
        step_data_array_[stp_idx].position_data.right_foot_pose.pitch,
        step_data_array_[stp_idx].position_data.right_foot_pose.yaw);

    Eigen::MatrixXd mat_l_foot = getTransformationXYZRPY(step_data_array_[stp_idx].position_data.left_foot_pose.x,
        step_data_array_[stp_idx].position_data.left_foot_pose.y,
        step_data_array_[stp_idx].position_data.left_foot_pose.z,
        step_data_array_[stp_idx].position_data.left_foot_pose.roll,
        step_data_array_[stp_idx].position_data.left_foot_pose.pitch,
        step_data_array_[stp_idx].position_data.left_foot_pose.yaw);

    step_data_array_[stp_idx].position_data.right_foot_pose = getPosefromTransformMatrix(mat_global_to_local * mat_r_foot);
    step_data_array_[stp_idx].position_data.left_foot_pose  = getPosefromTransformMatrix(mat_global_to_local * mat_l_foot);


    if(fabs(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
    {
      step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw + step_data_array_[stp_idx].position_data.left_foot_pose.yaw)
          - sign(0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw - step_data_array_[stp_idx].position_data.left_foot_pose.yaw))*M_PI;
    }
    else
    {
      step_data_array_[stp_idx].position_data.body_pose.yaw = 0.5*(step_data_array_[stp_idx].position_data.right_foot_pose.yaw
          + step_data_array_[stp_idx].position_data.left_foot_pose.yaw);
    }

    if(step_data_array_[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      step_data_array_[stp_idx].position_data.y_zmp_shift = y_zmp_convergence_m_;
    else if(step_data_array_[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      step_data_array_[stp_idx].position_data.y_zmp_shift = -y_zmp_convergence_m_;
    else
      step_data_array_[stp_idx].position_data.y_zmp_shift = 0;
  }

  return true;
}

void FootStepGenerator::calcFBStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction)
{
	adol_preview_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;

  if(ref_step_data.time_data.walking_state == adol_preview_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
    if(stp_data[0].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.x = stp_data[0].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
    }
    else
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.x = stp_data[0].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      stp_data[stp_idx].time_data.dsp_ratio = dsp_ratio_;
      stp_data[stp_idx].position_data.body_z_swap = body_z_swap_m_;
      stp_data[stp_idx].position_data.foot_z_swap = foot_z_swap_m_;

      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.x = stp_data[stp_idx].position_data.left_foot_pose.x + (double)direction*fb_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.x = stp_data[stp_idx].position_data.right_foot_pose.x + (double)direction*fb_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x = stp_data[num_of_step_-2].position_data.left_foot_pose.x;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x = stp_data[num_of_step_-2].position_data.right_foot_pose.x;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }

  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
  }
}

void FootStepGenerator::calcRLStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction)
{
	adol_preview_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;

  if(ref_step_data.time_data.walking_state == adol_preview_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;
    if(stp_data[0].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.y = stp_data[0].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
    }
    else
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.y = stp_data[0].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
    }

    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;


    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += step_time_sec_;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

    if(direction < 0)
    {
      stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.y = stp_data[1].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
    }
    else
    {
      stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.y = stp_data[1].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.y = stp_data[stp_idx].position_data.right_foot_pose.y + (double)direction*rl_step_length_m_;
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.y = stp_data[stp_idx].position_data.left_foot_pose.y + (double)direction*rl_step_length_m_;
      }
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = stp_data[num_of_step_-2].position_data.left_foot_pose.y - default_y_feet_offset_m_;
    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.y = stp_data[num_of_step_-2].position_data.right_foot_pose.y + default_y_feet_offset_m_;
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }

  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
  }
}

void FootStepGenerator::calcRoStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction)
{
	adol_preview_walking_module_msgs::StepData stp_data[num_of_step_];
  stp_data[0] = ref_step_data;
  if(ref_step_data.time_data.walking_state == adol_preview_walking_module_msgs::StepTimeData::IN_WALKING)
  {
    stp_data[0].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[0].time_data.abs_step_time += step_time_sec_;
    stp_data[0].time_data.dsp_ratio = dsp_ratio_;
    stp_data[0].position_data.body_z_swap = body_z_swap_m_;
    stp_data[0].position_data.foot_z_swap = foot_z_swap_m_;

    if(stp_data[0].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[0].position_data.right_foot_pose.yaw = stp_data[0].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[0].position_data.right_foot_pose.yaw) > 2.0*M_PI)
        stp_data[0].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[0].position_data.right_foot_pose.yaw);

      stp_data[0].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.right_foot_pose.yaw);
      stp_data[0].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.right_foot_pose.yaw);
    }
    else
    {
      stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[0].position_data.left_foot_pose.yaw = stp_data[0].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[0].position_data.left_foot_pose.yaw) > 2.0*M_PI)
        stp_data[0].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[0].position_data.left_foot_pose.yaw);

      stp_data[0].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[0].position_data.left_foot_pose.yaw);
      stp_data[0].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[0].position_data.left_foot_pose.yaw);
    }


    for(int stp_idx = 1; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);

        stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
        stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);

        stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
        stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

      }

      if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);
      stp_data[num_of_step_-2].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
      stp_data[num_of_step_-2].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;
  }
  else
  {
    stp_data[0].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
    stp_data[0].time_data.abs_step_time += 0.1;//start_end_time_sec_;
    stp_data[0].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[0].position_data.body_z_swap = 0;
    stp_data[0].position_data.foot_z_swap = 0;


    stp_data[1] = stp_data[0];
    stp_data[1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    stp_data[1].time_data.abs_step_time += step_time_sec_;
    stp_data[1].time_data.dsp_ratio = dsp_ratio_;
    stp_data[1].position_data.body_z_swap = body_z_swap_m_;
    stp_data[1].position_data.foot_z_swap = foot_z_swap_m_;

    if(direction < 0)
    {
      stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[1].position_data.right_foot_pose.yaw  = stp_data[1].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[1].position_data.right_foot_pose.yaw) > 2.0*M_PI)
        stp_data[1].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[1].position_data.right_foot_pose.yaw);

      stp_data[1].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.left_foot_pose.yaw);
      stp_data[1].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.left_foot_pose.yaw);
    }
    else
    {
      stp_data[1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[1].position_data.left_foot_pose.yaw  = stp_data[1].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

      if(fabs(stp_data[1].position_data.left_foot_pose.yaw) > 2.0*M_PI)
        stp_data[1].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[1].position_data.left_foot_pose.yaw);


      stp_data[1].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[1].position_data.left_foot_pose.yaw);
      stp_data[1].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[1].position_data.left_foot_pose.yaw);
    }

    for(int stp_idx = 2; stp_idx < num_of_step_-2; stp_idx++)
    {
      stp_data[stp_idx] = stp_data[stp_idx-1];
      stp_data[stp_idx].time_data.abs_step_time += step_time_sec_;
      if(stp_data[stp_idx].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
        stp_data[stp_idx].position_data.right_foot_pose.yaw = stp_data[stp_idx].position_data.right_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.right_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);

        stp_data[stp_idx].position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.right_foot_pose.yaw);
        stp_data[stp_idx].position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      }
      else
      {
        stp_data[stp_idx].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
        stp_data[stp_idx].position_data.left_foot_pose.yaw = stp_data[stp_idx].position_data.left_foot_pose.yaw + (double)direction*rotate_step_angle_rad_;

        if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > 2.0*M_PI)
          stp_data[stp_idx].position_data.left_foot_pose.yaw += -2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);

        stp_data[stp_idx].position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(stp_data[stp_idx].position_data.left_foot_pose.yaw);
        stp_data[stp_idx].position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(stp_data[stp_idx].position_data.left_foot_pose.yaw);

      }

      if(fabs(stp_data[stp_idx].position_data.right_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.right_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.right_foot_pose.yaw);
      if(fabs(stp_data[stp_idx].position_data.left_foot_pose.yaw) > M_PI)
        stp_data[stp_idx].position_data.left_foot_pose.yaw -= 2.0*M_PI*sign(stp_data[stp_idx].position_data.left_foot_pose.yaw);
    }

    stp_data[num_of_step_-2] = stp_data[num_of_step_-3];
    stp_data[num_of_step_-2].time_data.abs_step_time += step_time_sec_;
    if(stp_data[num_of_step_-2].position_data.moving_foot == adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING)
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.right_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.left_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.right_foot_pose.x =  0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);
      stp_data[num_of_step_-2].position_data.right_foot_pose.y = -0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.left_foot_pose.yaw);

    }
    else
    {
      stp_data[num_of_step_-2].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
      stp_data[num_of_step_-2].position_data.left_foot_pose.yaw  = stp_data[num_of_step_-2].position_data.right_foot_pose.yaw;
      stp_data[num_of_step_-2].position_data.left_foot_pose.x =  -0.5*default_y_feet_offset_m_*sin(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
      stp_data[num_of_step_-2].position_data.left_foot_pose.y =   0.5*default_y_feet_offset_m_*cos(stp_data[num_of_step_-2].position_data.right_foot_pose.yaw);
    }

    stp_data[num_of_step_-1] = stp_data[num_of_step_-2];
    stp_data[num_of_step_-1].time_data.abs_step_time += start_end_time_sec_;
    stp_data[num_of_step_-1].time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
    stp_data[num_of_step_-1].position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
    stp_data[num_of_step_-1].position_data.body_z_swap = 0;

  }

  for(int stp_idx = 0; stp_idx < num_of_step_; stp_idx++)
  {
    step_data_array_.push_back(stp_data[stp_idx]);
  }
}

void FootStepGenerator::calcStopStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction)
{
	adol_preview_walking_module_msgs::StepData stp_data;
  stp_data = ref_step_data;
  stp_data.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data.time_data.abs_step_time += start_end_time_sec_;
  stp_data.position_data.body_z_swap = 0;
  stp_data.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;

  step_data_array_.push_back(stp_data);
}

void FootStepGenerator::calcRightKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const adol_preview_walking_module_msgs::StepData& ref_step_data)
{

  adol_preview_walking_module_msgs::StepData step_data_msg;

  kick_height_m_  = 0.08;
  kick_far_m_     = 0.23;
  kick_pitch_rad_ = 15.0*M_PI/180.0;

  step_data_msg = ref_step_data;

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 1.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 0.4;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = -0.1;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);


  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 0.2;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = kick_far_m_;
  step_data_msg.position_data.right_foot_pose.pitch = -kick_pitch_rad_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.right_foot_pose.pitch = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  step_data_msg.position_data.right_foot_pose.x = 0;
  step_data_msg.position_data.right_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.right_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //StepData 8 End
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
  }
}

void FootStepGenerator::calcLeftKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const adol_preview_walking_module_msgs::StepData& ref_step_data)
{
  adol_preview_walking_module_msgs::StepData step_data_msg;

  kick_height_m_  = 0.08;
  kick_far_m_     = 0.23;
  kick_pitch_rad_ = 15.0*M_PI/180.0;

  step_data_msg = ref_step_data;

  step_data_array->clear();
  step_data_array_.clear();

  //Start 1 Step Data
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 1.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_msg.position_data.body_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 2 move zmp to left foot
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_array_.push_back(step_data_msg);


  //StepData 3 kick - 1st : raise foot
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.z += kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0.0;
  step_data_msg.position_data.foot_z_swap = 0;
  step_data_array_.push_back(step_data_msg);


  //StepData 4 kick - 2nd : move right foot back
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 0.4;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = -0.1;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);


  //StepData 5 kick - 3rd : kick
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 0.2;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = kick_far_m_;
  step_data_msg.position_data.left_foot_pose.pitch = -kick_pitch_rad_;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 6 kick - 4th : move right foot to x0
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_msg.position_data.foot_z_swap = 0.0;
  step_data_array_.push_back(step_data_msg);

  //StepData 7 move right foot to original place
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
  step_data_msg.time_data.dsp_ratio = 0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  step_data_msg.position_data.left_foot_pose.x = 0;
  step_data_msg.position_data.left_foot_pose.z -= kick_height_m_;
  step_data_msg.position_data.left_foot_pose.pitch = 0;
  step_data_array_.push_back(step_data_msg);

  //StepData 8 End
  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
  step_data_msg.time_data.dsp_ratio = 0.0;

  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
  step_data_msg.position_data.y_zmp_shift = 0;
  step_data_array_.push_back(step_data_msg);

  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
  {
    step_data_array->push_back(step_data_array_[stp_idx]);
  }
}


void FootStepGenerator::calcTurnLeftAndRightKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const adol_preview_walking_module_msgs::StepData& ref_step_data)
{
	  adol_preview_walking_module_msgs::StepData step_data_msg;

	  kick_height_m_  = 0.08;
	  kick_far_m_     = 0.23;
	  kick_pitch_rad_ = 15.0*M_PI/180.0;

    double balance_param_to_apply = 0.03;

	  step_data_msg = ref_step_data;

	  step_data_array->clear();
	  step_data_array_.clear();

	  //Start 1 Step Data
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
	  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = 1.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_msg.position_data.body_z_swap = 0;
	  step_data_array_.push_back(step_data_msg);

	  //StepData1 Turn Left
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += step_time_sec_; //kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = dsp_ratio_;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.y_zmp_shift = y_zmp_convergence_m_;
	  step_data_msg.position_data.foot_z_swap = foot_z_swap_m_;
	  step_data_msg.position_data.body_z_swap = body_z_swap_m_;

	  step_data_msg.position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.yaw =  rotate_step_angle_rad_;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_array_.push_back(step_data_msg);


	  //StepData 2 move zmp to left foot
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.y_zmp_shift = 0;
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_msg.position_data.body_z_swap = 0;
	  step_data_array_.push_back(step_data_msg);

    //StepData 2 move zmp to left foot
    step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
    step_data_msg.time_data.dsp_ratio = 0;

    step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
    step_data_msg.position_data.y_zmp_shift = balance_param_to_apply;
    step_data_msg.position_data.foot_z_swap = 0;
    step_data_msg.position_data.body_z_swap = 0;
    step_data_array_.push_back(step_data_msg);


	  //StepData 3 kick - 1st : raise foot
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.right_foot_pose.z += kick_height_m_;
	  step_data_msg.position_data.right_foot_pose.pitch = 0.0;
	  step_data_msg.position_data.right_foot_pose.yaw = rotate_step_angle_rad_;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_array_.push_back(step_data_msg);


	  //StepData 4 kick - 2nd : move right foot back
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 0.4;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(rotate_step_angle_rad_) - 0.1*cos(rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(rotate_step_angle_rad_) - 0.1*sin(rotate_step_angle_rad_);
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0.0;
	  step_data_array_.push_back(step_data_msg);


	  //StepData 5 kick - 3rd : kick
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 0.2;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(rotate_step_angle_rad_) + kick_far_m_*cos(rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(rotate_step_angle_rad_) + kick_far_m_*sin(rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.pitch = -kick_pitch_rad_;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_array_.push_back(step_data_msg);

	  //StepData 6 kick - 4th : move right foot to x0
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.right_foot_pose.x   =  0.5*default_y_feet_offset_m_*sin(rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.y   = -0.5*default_y_feet_offset_m_*cos(rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.pitch = 0;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0.0;
	  step_data_array_.push_back(step_data_msg);

	  //StepData 7 move right foot to original place
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.right_foot_pose.z -= kick_height_m_;
	  step_data_msg.position_data.right_foot_pose.pitch = 0;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_array_.push_back(step_data_msg);

    //StepData 2 move zmp to left foot
    step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
    step_data_msg.time_data.dsp_ratio = 0;

    step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
    step_data_msg.position_data.y_zmp_shift = 0;
    step_data_msg.position_data.foot_z_swap = 0;
    step_data_msg.position_data.body_z_swap = 0;
    step_data_array_.push_back(step_data_msg);

	  //StepData 8 End
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
	  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
	  step_data_msg.position_data.y_zmp_shift = 0;
	  step_data_array_.push_back(step_data_msg);

	  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
	  {
	    step_data_array->push_back(step_data_array_[stp_idx]);
	  }
}

void FootStepGenerator::calcTurnRightAndLeftKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
    const adol_preview_walking_module_msgs::StepData& ref_step_data)
{
	  adol_preview_walking_module_msgs::StepData step_data_msg;

	  kick_height_m_  = 0.08;
	  kick_far_m_     = 0.23;
	  kick_pitch_rad_ = 15.0*M_PI/180.0;

	  double balance_param_to_apply = -0.03;

	  step_data_msg = ref_step_data;

	  step_data_array->clear();
	  step_data_array_.clear();

	  //Start 1 Step Data
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
	  step_data_msg.time_data.abs_step_time += 0.1; //kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = 1.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_msg.position_data.body_z_swap = 0;
	  step_data_array_.push_back(step_data_msg);

	  //StepData1 Turn Right
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += step_time_sec_; //kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = dsp_ratio_;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
	  step_data_msg.position_data.y_zmp_shift = -y_zmp_convergence_m_;
	  step_data_msg.position_data.foot_z_swap = foot_z_swap_m_;
	  step_data_msg.position_data.body_z_swap = body_z_swap_m_;

	  step_data_msg.position_data.right_foot_pose.x   =   0.5*default_y_feet_offset_m_*sin(-rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.y   =  -0.5*default_y_feet_offset_m_*cos(-rotate_step_angle_rad_);
	  step_data_msg.position_data.right_foot_pose.yaw = -rotate_step_angle_rad_;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_array_.push_back(step_data_msg);



	  //StepData 2 move zmp to left foot
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.y_zmp_shift = 0;
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_msg.position_data.body_z_swap = 0;

    //StepData 2 move zmp to left foot
    step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
    step_data_msg.time_data.dsp_ratio = 0;

    step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
    step_data_msg.position_data.y_zmp_shift = balance_param_to_apply;
    step_data_msg.position_data.foot_z_swap = 0;
    step_data_msg.position_data.body_z_swap = 0;


	  //StepData 3 kick - 1st : raise foot
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.left_foot_pose.z += kick_height_m_;
	  step_data_msg.position_data.left_foot_pose.pitch = 0.0;
	  step_data_msg.position_data.left_foot_pose.yaw = -rotate_step_angle_rad_;

	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0;
	  step_data_array_.push_back(step_data_msg);


	  //StepData 4 kick - 2nd : move right foot back
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 0.4;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(-rotate_step_angle_rad_) - 0.1*cos(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(-rotate_step_angle_rad_) - 0.1*sin(-rotate_step_angle_rad_);

	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0.0;
	  step_data_array_.push_back(step_data_msg);


	  //StepData 5 kick - 3rd : kick
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 0.2;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(-rotate_step_angle_rad_) + kick_far_m_*cos(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(-rotate_step_angle_rad_) + kick_far_m_*sin(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.pitch = -kick_pitch_rad_;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0.0;
	  step_data_array_.push_back(step_data_msg);

	  //StepData 6 kick - 4th : move right foot to x0
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 2.0;//kick_time_sec_*0.5;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.pitch = 0;
	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_msg.position_data.foot_z_swap = 0.0;
	  step_data_array_.push_back(step_data_msg);

	  //StepData 7 move right foot to original place
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
	  step_data_msg.time_data.abs_step_time += 1.5;//kick_time_sec_*1.5;
	  step_data_msg.time_data.dsp_ratio = 0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
	  step_data_msg.position_data.left_foot_pose.x   = -0.5*default_y_feet_offset_m_*sin(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.y   =  0.5*default_y_feet_offset_m_*cos(-rotate_step_angle_rad_);
	  step_data_msg.position_data.left_foot_pose.z -= kick_height_m_;
	  step_data_msg.position_data.left_foot_pose.pitch = 0;

	  step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
	  step_data_array_.push_back(step_data_msg);

    //StepData 2 move zmp to left foot
    step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING;
    step_data_msg.time_data.abs_step_time += 1.0;//kick_time_sec_*0.3;
    step_data_msg.time_data.dsp_ratio = 0;

    step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
    step_data_msg.position_data.y_zmp_shift = 0;
    step_data_msg.position_data.foot_z_swap = 0;
    step_data_msg.position_data.body_z_swap = 0;
    step_data_msg.position_data.body_pose.yaw = 0.5*(step_data_msg.position_data.left_foot_pose.yaw + step_data_msg.position_data.right_foot_pose.yaw);
    step_data_array_.push_back(step_data_msg);


	  //StepData 8 End
	  step_data_msg.time_data.walking_state = adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
	  step_data_msg.time_data.abs_step_time += 1.6;// kick_time_sec_*1.8;
	  step_data_msg.time_data.dsp_ratio = 0.0;

	  step_data_msg.position_data.moving_foot = adol_preview_walking_module_msgs::StepPositionData::STANDING;
	  step_data_msg.position_data.y_zmp_shift = 0;
	  step_data_array_.push_back(step_data_msg);

	  for(unsigned int stp_idx = 0; stp_idx < step_data_array_.size(); stp_idx++)
	  {
	    step_data_array->push_back(step_data_array_[stp_idx]);
	  }
}
