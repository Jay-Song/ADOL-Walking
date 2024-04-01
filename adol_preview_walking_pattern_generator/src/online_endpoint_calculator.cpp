/*
 * online_endpoint_calculator.cpp
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */


#include "adol_preview_walking_pattern_generator/online_endpoint_calculator.h"

using namespace adol;

static const int NO_STEP_IDX = -1;

static const int IN_WALKING_STARTING = 0;
static const int IN_WALKING = 1;
static const int IN_WALKING_ENDING = 2;

static const int LEFT_FOOT_SWING  = 1;
static const int RIGHT_FOOT_SWING = 2;
static const int STANDING = 3;


//L -- O -- R
//1 -- 0 -- -1
static const int BalancingPhase0 = 0; // DSP : START
static const int BalancingPhase1 = 1; // DSP : R--O->L
static const int BalancingPhase2 = 2; // SSP : L_BALANCING1
static const int BalancingPhase3 = 3; // SSP : L_BALANCING2
static const int BalancingPhase4 = 4; // DSP : R--O<-L
static const int BalancingPhase5 = 5; // DSP : R<-O--L
static const int BalancingPhase6 = 6; // SSP : R_BALANCING1
static const int BalancingPhase7 = 7; // SSP : R_BALANCING2
static const int BalancingPhase8 = 8; // DSP : R->O--L
static const int BalancingPhase9 = 9; // DSP : END

static const int StepDataStatus1 = 1; //
static const int StepDataStatus2 = 2; //
static const int StepDataStatus3 = 3; //
static const int StepDataStatus4 = 4; //

OnlineEndpointCalculator::OnlineEndpointCalculator()
{
  present_right_foot_pose_.x = 0.0;    present_right_foot_pose_.y = -0.105;
  present_right_foot_pose_.z = -0.55;
  present_right_foot_pose_.roll = 0.0; present_right_foot_pose_.pitch = 0.0; present_right_foot_pose_.yaw = 0.0;

  present_left_foot_pose_.x = 0.0;    present_left_foot_pose_.y = 0.105;
  present_left_foot_pose_.z = -0.55;
  present_left_foot_pose_.roll = 0.0; present_left_foot_pose_.pitch = 0.0; present_left_foot_pose_.yaw = 0.0;

  present_body_pose_.x = 0.0;    present_body_pose_.y = 0.0;     present_body_pose_.z = 0.0;
  present_body_pose_.roll = 0.0; present_body_pose_.pitch = 0.0; present_body_pose_.yaw = 0;

  previous_step_right_foot_pose_ = initial_right_foot_pose_ = present_right_foot_pose_;
  previous_step_left_foot_pose_  = initial_left_foot_pose_  = present_left_foot_pose_;
  previous_step_body_pose_     = initial_body_pose_     = present_body_pose_;

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_roll_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = 0.0;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = present_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 1.6;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  current_step_data_status_ = StepDataStatus4;
  current_balancing_index_ = BalancingPhase0;

  walking_time_ = 0; reference_time_ = 0;
  control_time_sec_ = 0.008;

  preview_time_sec_ = 1.6;
  preview_size_ = round(preview_time_sec_/control_time_sec_);

  running = false;

  switching_ratio_ = 0;
}

OnlineEndpointCalculator::~OnlineEndpointCalculator()
{  }

void OnlineEndpointCalculator::setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot,
    robotis_framework::Pose3D pelvis)
{
  previous_step_right_foot_pose_ = r_foot;
  previous_step_left_foot_pose_  = l_foot;
  previous_step_body_pose_     = pelvis;


  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_     = previous_step_body_pose_;

  return;
}

void OnlineEndpointCalculator::initialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  if(running)
    return;

  smooth_tra_.changeTrajectory(0,0,0,0,1,1,0,0);

  xy_calculator_.initialize(lipm_height_m, preview_time_sec, control_time_sec);

  step_data_mutex_lock_.lock();
  added_step_data_.clear();

  //Initialize Time
  control_time_sec_ = control_time_sec;
  preview_time_sec_ = preview_time_sec;
  walking_time_ = 0; reference_time_ = 0;

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_     = previous_step_body_pose_;

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_roll_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = 0.0;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = present_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 1.6;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  current_step_data_status_ = StepDataStatus4;
  current_balancing_index_ = BalancingPhase0;

  preview_size_ = round(preview_time_sec_/control_time_sec_);

  step_idx_data_.resize(preview_size_);
  step_idx_data_.fill(NO_STEP_IDX);

  reference_zmp_x_.resize(preview_size_, 1);
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.resize(preview_size_, 1);
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));

  step_data_mutex_lock_.unlock();

  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}

void OnlineEndpointCalculator::reInitialize()
{
  walking_time_ = 0; reference_time_ = 0;

  Eigen::Matrix4d mat_g_to_pelvis = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);
  Eigen::Matrix4d mat_g_to_rfoot = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  Eigen::Matrix4d mat_g_to_lfoot = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  Eigen::Matrix4d mat_robot_to_pelvis = robotis_framework::getRotation4d(previous_step_body_pose_.roll, previous_step_body_pose_.pitch, 0);
  Eigen::Matrix4d mat_pelvis_to_g = robotis_framework::getInverseTransformation(mat_g_to_pelvis);

  Eigen::Matrix4d mat_robot_to_rfoot = (mat_robot_to_pelvis * mat_pelvis_to_g) * mat_g_to_rfoot;
  Eigen::Matrix4d mat_robot_to_lfoot = (mat_robot_to_pelvis * mat_pelvis_to_g) * mat_g_to_lfoot;


  previous_step_body_pose_       = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_pelvis);
  previous_step_right_foot_pose_ = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_rfoot);
  previous_step_left_foot_pose_  = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_lfoot);

  present_body_pose_       = previous_step_body_pose_;
  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;

  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;

  step_idx_data_.fill(NO_STEP_IDX);
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));

  xy_calculator_.reInitialize();
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}

void OnlineEndpointCalculator::addStepData(robotis_framework::StepData step_data)
{
  step_data_mutex_lock_.lock();
  added_step_data_.push_back(step_data);

  calcStepIdxData();
  step_data_mutex_lock_.unlock();
}

void OnlineEndpointCalculator::eraseLastStepData()
{
  step_data_mutex_lock_.lock();
  if(getNumofRemainingUnreservedStepData() != 0)
  {
    added_step_data_.pop_back();
  }
  step_data_mutex_lock_.unlock();
}

int  OnlineEndpointCalculator::getNumofRemainingUnreservedStepData()
{
  int step_idx = step_idx_data_(preview_size_ - 1);
  int remain_step_num = 0;
  if(step_idx != NO_STEP_IDX)
  {
    remain_step_num = (added_step_data_.size() - 1 - step_idx);
  }
  else
  {
    remain_step_num = 0;
  }
  return remain_step_num;
}

void OnlineEndpointCalculator::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  reference_step_data_for_addition_.position_data.x_zmp_shift = 0;
  reference_step_data_for_addition_.position_data.y_zmp_shift = 0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;
  (*ref_step_data_for_addition) = reference_step_data_for_addition_;
}

void OnlineEndpointCalculator::calcStepIdxData()
{
  unsigned int step_idx = 0, previous_step_idx = 0;
  unsigned int step_data_size = added_step_data_.size();
  if(added_step_data_.size() == 0)
  {
    step_idx_data_.fill(NO_STEP_IDX);
    current_step_data_status_ = StepDataStatus4;
    running = false;
  }
  else
  {
    if(walking_time_ >= added_step_data_[0].time_data.abs_step_time - 0.5*0.001)
    {
      //set previous step data
      previous_step_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      previous_step_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
      previous_step_body_pose_ = added_step_data_[0].position_data.body_pose;
      previous_step_body_pose_.x = present_body_pose_.x;
      previous_step_body_pose_.y = present_body_pose_.y;
      reference_time_ = added_step_data_[0].time_data.abs_step_time;
      added_step_data_.erase(added_step_data_.begin());
      if(added_step_data_.size() == 0)
      {
        step_idx_data_.fill(NO_STEP_IDX);
        current_step_data_status_ = StepDataStatus4;
        reInitialize();
        running = false;
      }
      else
      {
        for(int idx = 0; idx < preview_size_; idx++)
        {
          //Get STepIDx
          if(walking_time_ + (idx+1)*control_time_sec_ > added_step_data_[step_data_size - 1].time_data.abs_step_time)
            step_idx_data_(idx) = NO_STEP_IDX;
          else
          {
            for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
            {
              if(walking_time_ + (idx+1)*control_time_sec_ <= added_step_data_[step_idx].time_data.abs_step_time)
                break;
            }
            step_idx_data_(idx) = step_idx;
            previous_step_idx = step_idx;
          }
        }
      }
    }
    else
    {
      for(int idx = 0; idx < preview_size_; idx++)
      {
        //Get StepIdx
        if(walking_time_ + (idx+1)*control_time_sec_ > added_step_data_[step_data_size -1].time_data.abs_step_time)
          step_idx_data_(idx) = NO_STEP_IDX;
        else
        {
          for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
          {
            if(walking_time_ + (idx+1)*control_time_sec_ <= added_step_data_[step_idx].time_data.abs_step_time)
              break;
          }
          step_idx_data_(idx) = step_idx;
          previous_step_idx = step_idx;
        }
      }
    }
  }

  if(step_idx_data_(preview_size_ - 1) != NO_STEP_IDX)
  {
    if(getNumofRemainingUnreservedStepData() != 0)
    {
      current_step_data_status_ = StepDataStatus1;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
    else
    {
      current_step_data_status_ = StepDataStatus2;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
  }
  else
  {
    if(step_idx_data_(0) != NO_STEP_IDX)
    {
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(0)];
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time += preview_time_sec_;

      current_step_data_status_ = StepDataStatus3;
    }
    else
    {
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time = walking_time_ + preview_time_sec_;
      current_step_data_status_ = StepDataStatus4;
    }
  }
}

void OnlineEndpointCalculator::calcRefZMP()
{
  int ref_zmp_idx = 0;
  int step_idx = 0;

  double support_foot_x = 0;
  double support_foot_y = 0;
  double support_foot_yaw = 0;

  double desired_zmp_x = 0, desired_zmp_y = 0;

  if(walking_time_ == 0)
  {
    if((step_idx_data_(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
    {
      reference_zmp_x_.fill((present_left_foot_pose_.x + present_right_foot_pose_.x)*0.5);
      reference_zmp_y_.fill((present_left_foot_pose_.y + present_right_foot_pose_.y)*0.5);
      return;
    }

    for(ref_zmp_idx = 0; ref_zmp_idx < preview_size_;  ref_zmp_idx++)
    {
      step_idx = step_idx_data_(ref_zmp_idx);
      if(step_idx == NO_STEP_IDX)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = reference_zmp_x_(ref_zmp_idx - 1, 0);
        reference_zmp_y_(ref_zmp_idx, 0) = reference_zmp_y_(ref_zmp_idx - 1, 0);
      }
      else
      {
        if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
        {
          if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
          {
            support_foot_x = added_step_data_[step_idx].position_data.left_foot_pose.x;
            support_foot_y = added_step_data_[step_idx].position_data.left_foot_pose.y;
            support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

            desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                            - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
            desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                            + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
          {
            support_foot_x = added_step_data_[step_idx].position_data.right_foot_pose.x;
            support_foot_y = added_step_data_[step_idx].position_data.right_foot_pose.y;
            support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

            desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                            - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
            desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                            + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
          else
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
    }
  }
  else
  {
    step_idx = step_idx_data_(preview_size_ - 1);

    for(ref_zmp_idx = 1; ref_zmp_idx < preview_size_; ref_zmp_idx++)
    {
      reference_zmp_x_(ref_zmp_idx - 1, 0) = reference_zmp_x_(ref_zmp_idx, 0);
      reference_zmp_y_(ref_zmp_idx - 1, 0) = reference_zmp_y_(ref_zmp_idx, 0);
    }

    ref_zmp_idx = preview_size_ - 1;

    if(step_idx == NO_STEP_IDX)
    {
      reference_zmp_x_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.x + reference_step_data_for_addition_.position_data.left_foot_pose.x);
      reference_zmp_y_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.y + reference_step_data_for_addition_.position_data.left_foot_pose.y);
    }
    else
    {
      if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
      {
        if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
        {
          support_foot_x = added_step_data_[step_idx].position_data.left_foot_pose.x;
          support_foot_y = added_step_data_[step_idx].position_data.left_foot_pose.y;
          support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

          desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                          - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
          desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                          + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

          reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
          reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
        {
          support_foot_x = added_step_data_[step_idx].position_data.right_foot_pose.x;
          support_foot_y = added_step_data_[step_idx].position_data.right_foot_pose.y;
          support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

          desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                          - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
          desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                          + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

          reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
          reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
    }
  }
}

void OnlineEndpointCalculator::calcSmoothRefZMP()
{
  int ref_zmp_idx = 0;
  int step_idx = 0;

  double support_foot_x = 0;
  double support_foot_y = 0;
  double support_foot_yaw = 0;

  double feet_center_x = 0;
  double feet_center_y = 0;

  double zmp_moving_ratio = 0;

  double desired_zmp_x = 0, desired_zmp_y = 0;

  double period_time, ssp_time_start, ssp_time_end, calc_curr_time, calc_ref_time;

  if(walking_time_ == 0)
  {
    if((step_idx_data_(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
    {
      reference_zmp_x_.fill((present_left_foot_pose_.x + present_right_foot_pose_.x)*0.5);
      reference_zmp_y_.fill((present_left_foot_pose_.y + present_right_foot_pose_.y)*0.5);
      return;
    }

    for(ref_zmp_idx = 0; ref_zmp_idx < preview_size_;  ref_zmp_idx++)
    {
      step_idx = step_idx_data_(ref_zmp_idx);
      if(step_idx == NO_STEP_IDX)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = reference_zmp_x_(ref_zmp_idx - 1, 0);
        reference_zmp_y_(ref_zmp_idx, 0) = reference_zmp_y_(ref_zmp_idx - 1, 0);
      }
      else
      {
        if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
        {
          if(step_idx == 0)
          {
            period_time = added_step_data_[step_idx].time_data.abs_step_time - reference_time_;
            calc_ref_time = reference_time_;
          }
          else
          {
            period_time = added_step_data_[step_idx].time_data.abs_step_time - added_step_data_[step_idx-1].time_data.abs_step_time;
            calc_ref_time = added_step_data_[step_idx-1].time_data.abs_step_time;
          }

          added_step_data_[step_idx].time_data.dsp_ratio;

          ssp_time_start = added_step_data_[step_idx].time_data.dsp_ratio * period_time*0.5 + calc_ref_time;
          ssp_time_end = (2 - added_step_data_[step_idx].time_data.dsp_ratio)*period_time*0.5 + calc_ref_time;

          calc_curr_time = walking_time_ + ref_zmp_idx*control_time_sec_;
          //std::cout << walking_time_<< " " << calc_curr_time <<  " " << calc_ref_time  <<  " " << ssp_time_start  <<  " " << ssp_time_end << " " << added_step_data_[step_idx].time_data.abs_step_time << std::endl;
          if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
          {
            support_foot_x = added_step_data_[step_idx].position_data.left_foot_pose.x;
            support_foot_y = added_step_data_[step_idx].position_data.left_foot_pose.y;
            support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

            desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                            - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
            desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                            + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

            if(calc_curr_time < ssp_time_start)
            {
              if(step_idx == 0)
              {
                feet_center_x = 0.5*(previous_step_right_foot_pose_.x + previous_step_left_foot_pose_.x);
                feet_center_y = 0.5*(previous_step_right_foot_pose_.y + previous_step_left_foot_pose_.y);
              }
              else
              {
                feet_center_x = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.x + added_step_data_[step_idx-1].position_data.right_foot_pose.x);
                feet_center_y = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.y + added_step_data_[step_idx-1].position_data.right_foot_pose.y);
              }

              zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - calc_ref_time)/(ssp_time_start - calc_ref_time));

              reference_zmp_x_(ref_zmp_idx, 0) = feet_center_x + (desired_zmp_x - feet_center_x)*zmp_moving_ratio;
              reference_zmp_y_(ref_zmp_idx, 0) = feet_center_y + (desired_zmp_y - feet_center_y)*zmp_moving_ratio;
            }
            else if(calc_curr_time <= ssp_time_end)
            {
              reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
              reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
            }
            else
            {
              feet_center_x = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x);
              feet_center_y = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y);

              zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - ssp_time_end)/(added_step_data_[step_idx].time_data.abs_step_time - ssp_time_end));

              reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x + (feet_center_x - desired_zmp_x)*zmp_moving_ratio;
              reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y + (feet_center_y - desired_zmp_y)*zmp_moving_ratio;
            }
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
          {
            support_foot_x = added_step_data_[step_idx].position_data.right_foot_pose.x;
            support_foot_y = added_step_data_[step_idx].position_data.right_foot_pose.y;
            support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

            desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                            - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
            desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                            + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

            if(calc_curr_time < ssp_time_start)
            {
              if(step_idx == 0)
              {
                feet_center_x = 0.5*(previous_step_right_foot_pose_.x + previous_step_left_foot_pose_.x);
                feet_center_y = 0.5*(previous_step_right_foot_pose_.y + previous_step_left_foot_pose_.y);
              }
              else
              {
                feet_center_x = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.x + added_step_data_[step_idx-1].position_data.right_foot_pose.x);
                feet_center_y = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.y + added_step_data_[step_idx-1].position_data.right_foot_pose.y);
              }

              zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - calc_ref_time)/(ssp_time_start - calc_ref_time));

              reference_zmp_x_(ref_zmp_idx, 0) = feet_center_x + (desired_zmp_x - feet_center_x)*zmp_moving_ratio;
              reference_zmp_y_(ref_zmp_idx, 0) = feet_center_y + (desired_zmp_y - feet_center_y)*zmp_moving_ratio;
            }
            else if(calc_curr_time <= ssp_time_end)
            {
              reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
              reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
            }
            else
            {
              feet_center_x = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x);
              feet_center_y = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y);

              zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - ssp_time_end)/(added_step_data_[step_idx].time_data.abs_step_time - ssp_time_end));

              reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x + (feet_center_x - desired_zmp_x)*zmp_moving_ratio;
              reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y + (feet_center_y - desired_zmp_y)*zmp_moving_ratio;
            }
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
          else
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
    }
  }
  else
  {
    step_idx = step_idx_data_(preview_size_ - 1);

    for(ref_zmp_idx = 1; ref_zmp_idx < preview_size_; ref_zmp_idx++)
    {
      reference_zmp_x_(ref_zmp_idx - 1, 0) = reference_zmp_x_(ref_zmp_idx, 0);
      reference_zmp_y_(ref_zmp_idx - 1, 0) = reference_zmp_y_(ref_zmp_idx, 0);
    }

    ref_zmp_idx = preview_size_ - 1;

    if(step_idx == NO_STEP_IDX)
    {
      reference_zmp_x_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.x + reference_step_data_for_addition_.position_data.left_foot_pose.x);
      reference_zmp_y_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.y + reference_step_data_for_addition_.position_data.left_foot_pose.y);
    }
    else
    {
      if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
      {
        if(step_idx == 0)
        {
          period_time = added_step_data_[step_idx].time_data.abs_step_time - reference_time_;
          calc_ref_time = reference_time_;
        }
        else
        {
          period_time = added_step_data_[step_idx].time_data.abs_step_time - added_step_data_[step_idx-1].time_data.abs_step_time;
          calc_ref_time = added_step_data_[step_idx-1].time_data.abs_step_time;
        }

        added_step_data_[step_idx].time_data.dsp_ratio;

        ssp_time_start = added_step_data_[step_idx].time_data.dsp_ratio * period_time*0.5 + calc_ref_time;
        ssp_time_end = (2 - added_step_data_[step_idx].time_data.dsp_ratio)*period_time*0.5 + calc_ref_time;

        calc_curr_time = walking_time_ + ref_zmp_idx*control_time_sec_;
        //std::cout << walking_time_<< " " << calc_curr_time <<  " " << calc_ref_time  <<  " " << ssp_time_start  <<  " " << ssp_time_end << " " << added_step_data_[step_idx].time_data.abs_step_time << std::endl;

        if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
        {
          support_foot_x = added_step_data_[step_idx].position_data.left_foot_pose.x;
          support_foot_y = added_step_data_[step_idx].position_data.left_foot_pose.y;
          support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

          desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                          - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
          desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                          + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

          //std::cout << support_foot_x << " " << support_foot_y  << " " <<  desired_zmp_x << " " << desired_zmp_y  << " " <<  added_step_data_[step_idx].position_data.x_zmp_shift << " " << added_step_data_[step_idx].position_data.y_zmp_shift << std::endl;

          if(calc_curr_time < ssp_time_start)
          {
            if(step_idx == 0)
            {
              feet_center_x = 0.5*(previous_step_right_foot_pose_.x + previous_step_left_foot_pose_.x);
              feet_center_y = 0.5*(previous_step_right_foot_pose_.y + previous_step_left_foot_pose_.y);
            }
            else
            {
              feet_center_x = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.x + added_step_data_[step_idx-1].position_data.right_foot_pose.x);
              feet_center_y = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.y + added_step_data_[step_idx-1].position_data.right_foot_pose.y);
            }

            zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - calc_ref_time)/(ssp_time_start - calc_ref_time));

            reference_zmp_x_(ref_zmp_idx, 0) = feet_center_x + (desired_zmp_x - feet_center_x)*zmp_moving_ratio;
            reference_zmp_y_(ref_zmp_idx, 0) = feet_center_y + (desired_zmp_y - feet_center_y)*zmp_moving_ratio;
          }
          else if(calc_curr_time <= ssp_time_end)
          {
            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
          }
          else
          {
            feet_center_x = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x);
            feet_center_y = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y);

            zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - ssp_time_end)/(added_step_data_[step_idx].time_data.abs_step_time - ssp_time_end));

            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x + (feet_center_x - desired_zmp_x)*zmp_moving_ratio;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y + (feet_center_y - desired_zmp_y)*zmp_moving_ratio;
          }
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
        {
          support_foot_x = added_step_data_[step_idx].position_data.right_foot_pose.x;
          support_foot_y = added_step_data_[step_idx].position_data.right_foot_pose.y;
          support_foot_yaw = added_step_data_[step_idx].position_data.left_foot_pose.yaw;

          desired_zmp_x = added_step_data_[step_idx].position_data.x_zmp_shift*cos(support_foot_yaw)
                          - added_step_data_[step_idx].position_data.y_zmp_shift*sin(support_foot_yaw) + support_foot_x;
          desired_zmp_y = added_step_data_[step_idx].position_data.x_zmp_shift*sin(support_foot_yaw)
                          + added_step_data_[step_idx].position_data.y_zmp_shift*cos(support_foot_yaw) + support_foot_y;

          //std::cout << support_foot_x << " " << support_foot_y  << " " <<  desired_zmp_x << " " << desired_zmp_y  << " " <<  added_step_data_[step_idx].position_data.x_zmp_shift << " " << added_step_data_[step_idx].position_data.y_zmp_shift << std::endl;

          if(calc_curr_time < ssp_time_start)
          {
            if(step_idx == 0)
            {
              feet_center_x = 0.5*(previous_step_right_foot_pose_.x + previous_step_left_foot_pose_.x);
              feet_center_y = 0.5*(previous_step_right_foot_pose_.y + previous_step_left_foot_pose_.y);
            }
            else
            {
              feet_center_x = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.x + added_step_data_[step_idx-1].position_data.right_foot_pose.x);
              feet_center_y = 0.5*(added_step_data_[step_idx-1].position_data.left_foot_pose.y + added_step_data_[step_idx-1].position_data.right_foot_pose.y);
            }

            zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - calc_ref_time)/(ssp_time_start - calc_ref_time));

            reference_zmp_x_(ref_zmp_idx, 0) = feet_center_x + (desired_zmp_x - feet_center_x)*zmp_moving_ratio;
            reference_zmp_y_(ref_zmp_idx, 0) = feet_center_y + (desired_zmp_y - feet_center_y)*zmp_moving_ratio;
          }
          else if(calc_curr_time <= ssp_time_end)
          {
            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y;
          }
          else
          {
            feet_center_x = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x);
            feet_center_y = 0.5*(added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y);

            zmp_moving_ratio = smooth_tra_.getPosition((calc_curr_time - ssp_time_end)/(added_step_data_[step_idx].time_data.abs_step_time - ssp_time_end));

            reference_zmp_x_(ref_zmp_idx, 0) = desired_zmp_x + (feet_center_x - desired_zmp_x)*zmp_moving_ratio;
            reference_zmp_y_(ref_zmp_idx, 0) = desired_zmp_y + (feet_center_y - desired_zmp_y)*zmp_moving_ratio;
          }
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
    }
  }
}

void OnlineEndpointCalculator::calcEndPoint()
{
  if(running == false)
    return;

  if((added_step_data_.size() == 0) || !running)
    return;

  double period_time, ssp_ratio, foot_move_period_time, ssp_time_start, ssp_time_end;
  robotis_framework::StepTimeData curr_step_time_data = added_step_data_[0].time_data;
  period_time = added_step_data_[0].time_data.abs_step_time - reference_time_;
  ssp_ratio = 1 - curr_step_time_data.dsp_ratio;
  foot_move_period_time = ssp_ratio*period_time;

  ssp_time_start = curr_step_time_data.dsp_ratio*period_time/2.0 + reference_time_;
  ssp_time_end = (1 + ssp_ratio)*period_time / 2.0 + reference_time_;

  if( (walking_time_ - reference_time_) < control_time_sec_)
  {
    body_z_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.z, 0, 0,
        added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.z, 0, 0);
    body_roll_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.roll, 0, 0,
        added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.roll, 0, 0);
    body_pitch_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.pitch, 0, 0,
        added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.pitch, 0, 0);

    double pelvis_yaw_move_amp = added_step_data_[0].position_data.body_pose.yaw - previous_step_body_pose_.yaw;

    if(pelvis_yaw_move_amp >= M_PI)
      pelvis_yaw_move_amp -= 2.0*M_PI;
    else if(pelvis_yaw_move_amp <= -M_PI)
      pelvis_yaw_move_amp += 2.0*M_PI;

    body_yaw_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.yaw, 0, 0,
        added_step_data_[0].time_data.abs_step_time, pelvis_yaw_move_amp + previous_step_body_pose_.yaw, 0, 0);

    body_z_swap_tra_.changeTrajectory(reference_time_,
        0, 0, 0,
        0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
        added_step_data_[0].position_data.body_z_swap, 0, 0);

    if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
    {
      foot_x_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_x*foot_move_period_time,
          previous_step_right_foot_pose_.x, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_x*foot_move_period_time,
          added_step_data_[0].position_data.right_foot_pose.x, 0, 0);
      foot_y_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_y*foot_move_period_time,
          previous_step_right_foot_pose_.y, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_y*foot_move_period_time,
          added_step_data_[0].position_data.right_foot_pose.y, 0, 0);
      foot_z_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_z*foot_move_period_time,
          previous_step_right_foot_pose_.z, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_z*foot_move_period_time,
          added_step_data_[0].position_data.right_foot_pose.z, 0, 0);
      foot_roll_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_roll*foot_move_period_time,
          previous_step_right_foot_pose_.roll, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_roll*foot_move_period_time,
          added_step_data_[0].position_data.right_foot_pose.roll, 0, 0);
      foot_pitch_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_pitch*foot_move_period_time,
          previous_step_right_foot_pose_.pitch, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_pitch*foot_move_period_time,
          added_step_data_[0].position_data.right_foot_pose.pitch, 0, 0);

      double foot_yaw_move_amp = added_step_data_[0].position_data.right_foot_pose.yaw - previous_step_right_foot_pose_.yaw;
      if(foot_yaw_move_amp >= M_PI)
        foot_yaw_move_amp -= 2.0*M_PI;
      else if(foot_yaw_move_amp <= -M_PI)
        foot_yaw_move_amp += 2.0*M_PI;

      foot_yaw_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_yaw*foot_move_period_time,
          previous_step_right_foot_pose_.yaw, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_yaw*foot_move_period_time,
          foot_yaw_move_amp + previous_step_right_foot_pose_.yaw, 0, 0);

      foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
          0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);
    }
    else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
    {
      foot_x_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_x*foot_move_period_time,
          previous_step_left_foot_pose_.x, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_x*foot_move_period_time,
          added_step_data_[0].position_data.left_foot_pose.x, 0, 0);
      foot_y_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_y*foot_move_period_time,
          previous_step_left_foot_pose_.y, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_y*foot_move_period_time,
          added_step_data_[0].position_data.left_foot_pose.y, 0, 0);
      foot_z_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_z*foot_move_period_time,
          previous_step_left_foot_pose_.z, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_z*foot_move_period_time,
          added_step_data_[0].position_data.left_foot_pose.z, 0, 0);
      foot_roll_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_roll*foot_move_period_time,
          previous_step_left_foot_pose_.roll, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_roll*foot_move_period_time,
          added_step_data_[0].position_data.left_foot_pose.roll, 0, 0);
      foot_pitch_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_pitch*foot_move_period_time,
          previous_step_left_foot_pose_.pitch, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_pitch*foot_move_period_time,
          added_step_data_[0].position_data.left_foot_pose.pitch, 0, 0);

      double foot_yaw_move_amp = added_step_data_[0].position_data.left_foot_pose.yaw - previous_step_left_foot_pose_.yaw;
      if(foot_yaw_move_amp >= M_PI)
        foot_yaw_move_amp -= 2.0*M_PI;
      else if(foot_yaw_move_amp <= -M_PI)
        foot_yaw_move_amp += 2.0*M_PI;

      foot_yaw_tra_.changeTrajectory(ssp_time_start + curr_step_time_data.start_time_delay_ratio_yaw*foot_move_period_time,
          previous_step_left_foot_pose_.yaw, 0, 0,
          ssp_time_end - curr_step_time_data.finish_time_advance_ratio_yaw*foot_move_period_time,
          foot_yaw_move_amp + previous_step_left_foot_pose_.yaw, 0, 0);

      foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
          0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);
    }
    else
    {
      foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
          ssp_time_end, 0, 0, 0);
    }
  }

  double z_swap = body_z_swap_tra_.getPosition(walking_time_);
  double bz_move = body_z_tra_.getPosition(walking_time_);
  double ba_move = body_roll_tra_.getPosition(walking_time_);
  double bb_move = body_pitch_tra_.getPosition(walking_time_);
  double bc_move = body_yaw_tra_.getPosition(walking_time_);

  present_body_pose_.z = bz_move + z_swap;
  present_body_pose_.roll = ba_move;
  present_body_pose_.pitch = bb_move;
  present_body_pose_.yaw = bc_move;

  //Feet
  double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;
  if( walking_time_ < ssp_time_start)
  {
    x_move = foot_x_tra_.getPosition(ssp_time_start);
    y_move = foot_y_tra_.getPosition(ssp_time_start);
    z_move = foot_z_tra_.getPosition(ssp_time_start);
    a_move = foot_roll_tra_.getPosition(ssp_time_start);
    b_move = foot_pitch_tra_.getPosition(ssp_time_start);
    c_move = foot_yaw_tra_.getPosition(ssp_time_start);

    z_vibe = foot_z_swap_tra_.getPosition(ssp_time_start);

    switching_ratio_ = smooth_tra_.getPosition((walking_time_ - reference_time_)/(ssp_time_start - reference_time_));

    if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
    {
      current_balancing_index_ = BalancingPhase1;
      switching_ratio_ =  1.0*switching_ratio_ ;
    }
    else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
    {
      current_balancing_index_ = BalancingPhase5;
      switching_ratio_ =  -1.0*switching_ratio_ ;
    }
    else
    {
      current_balancing_index_ = BalancingPhase0;
      switching_ratio_ = 0;
    }
  }
  else if( walking_time_ <= ssp_time_end)
  {
    x_move = foot_x_tra_.getPosition(walking_time_);
    y_move = foot_y_tra_.getPosition(walking_time_);
    z_move = foot_z_tra_.getPosition(walking_time_);
    a_move = foot_roll_tra_.getPosition(walking_time_);
    b_move = foot_pitch_tra_.getPosition(walking_time_);
    c_move = foot_yaw_tra_.getPosition(walking_time_);

    if(added_step_data_[0].position_data.moving_foot != STANDING)
    {
      if((walking_time_ >= 0.5*(ssp_time_start + ssp_time_end))
          && (walking_time_ < (0.5*(ssp_time_start + ssp_time_end) + control_time_sec_)))
      {
        body_z_swap_tra_.changeTrajectory(0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
            added_step_data_[0].position_data.body_z_swap, 0, 0,
            added_step_data_[0].time_data.abs_step_time,
            0, 0, 0);

        foot_z_swap_tra_.changeTrajectory(0.5*(ssp_time_start + ssp_time_end),
            added_step_data_[0].position_data.foot_z_swap, 0, 0,
            ssp_time_end, 0, 0, 0);
      }
    }

    z_vibe = foot_z_swap_tra_.getPosition(walking_time_);

    if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
    {
      if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
        current_balancing_index_ = BalancingPhase2;
      else
        current_balancing_index_ = BalancingPhase3;

      switching_ratio_ =  1;
    }
    else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
    {
      if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
        current_balancing_index_ = BalancingPhase6;
      else
        current_balancing_index_ = BalancingPhase7;

      switching_ratio_ =  -1;
    }
    else
    {
      current_balancing_index_ = BalancingPhase0;
      switching_ratio_ =  0;
    }
  }
  else
  {
    x_move = foot_x_tra_.getPosition(ssp_time_end);
    y_move = foot_y_tra_.getPosition(ssp_time_end);
    z_move = foot_z_tra_.getPosition(ssp_time_end);
    a_move = foot_roll_tra_.getPosition(ssp_time_end);
    b_move = foot_pitch_tra_.getPosition(ssp_time_end);
    c_move = foot_yaw_tra_.getPosition(ssp_time_end);

    z_vibe = foot_z_swap_tra_.getPosition(ssp_time_end);

    switching_ratio_ = smooth_tra_.getPosition((walking_time_ - ssp_time_end)/(added_step_data_[0].time_data.abs_step_time - ssp_time_end));
    switching_ratio_ = 1 - switching_ratio_;

    if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
    {
      current_balancing_index_ = BalancingPhase4;
      switching_ratio_ =  1.0*switching_ratio_ ;
    }
    else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
    {
      current_balancing_index_ = BalancingPhase8;
      switching_ratio_ =  -1.0*switching_ratio_ ;
    }
    else
    {
      current_balancing_index_ = BalancingPhase0;
      switching_ratio_ =  0.0;
    }
  }


  if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
  {
    present_right_foot_pose_.x = x_move;
    present_right_foot_pose_.y = y_move;
    present_right_foot_pose_.z = z_move + z_vibe;
    present_right_foot_pose_.roll = a_move;
    present_right_foot_pose_.pitch = b_move;
    present_right_foot_pose_.yaw = c_move;

    present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
  }
  else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
  {
    present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;

    present_left_foot_pose_.x = x_move;
    present_left_foot_pose_.y = y_move;
    present_left_foot_pose_.z = z_move + z_vibe;
    present_left_foot_pose_.roll = a_move;
    present_left_foot_pose_.pitch = b_move;
    present_left_foot_pose_.yaw = c_move;
  }
  else
  {
    present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
    present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
  }

  //std::cout << walking_time_ << " " << added_step_data_[0].time_data.abs_step_time << " " << ssp_time_start << " " << ssp_time_end << " " << switching_ratio_ << " " << current_balancing_index_ << std::endl;
  walking_time_ += control_time_sec_;

//  if(walking_time_ > added_step_data_[added_step_data_.size() - 1].time_data.abs_step_time - 0.5*0.001)
//  {
//    running = false;
//    //calcStepIdxData();
//  }
}

void OnlineEndpointCalculator::calcDesiredPose()
{
  //std::cout << walking_time_ << " ";
  step_data_mutex_lock_.lock();
  calcStepIdxData();
  calcEndPoint();
  //calcRefZMP();
  calcSmoothRefZMP();
  step_data_mutex_lock_.unlock();

  xy_calculator_.calcNextPelvisXY(reference_zmp_x_, reference_zmp_y_);
  present_body_pose_.x = xy_calculator_.x_lipm_.coeff(0);
  present_body_pose_.y = xy_calculator_.y_lipm_.coeff(0);

  x_lipm_ = xy_calculator_.x_lipm_;
  y_lipm_ = xy_calculator_.y_lipm_;

  //std::cout << reference_zmp_x_.coeff(0,0) << " " << reference_zmp_y_.coeff(0,0) << " " << present_body_pose_ << " " << present_right_foot_pose_ << " " << present_left_foot_pose_ <<" ";//<< std::endl;

  //std::cout << switching_ratio_ << std::endl;
}

void OnlineEndpointCalculator::start()
{
  running = true;
}

bool OnlineEndpointCalculator::isRunning()
{
  return running;
}

