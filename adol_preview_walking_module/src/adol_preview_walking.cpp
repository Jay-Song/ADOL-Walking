/*
 * adol_preview_walking.cpp
 *
 *  Created on: 2024. 04. 01.
 *      Author: Jay SOng
 */

#include "adol_preview_walking_module/adol_preview_walking.h"

using namespace adol;


PIDController::PIDController(double control_time_sec)
{
  p_gain_ = 0;
  i_gain_ = 0;
  d_gain_ = 0;

  control_time_sec_ = control_time_sec;
  curr_err_ = 0;
  prev_err_ = 0;
  sum_err_ = 0;
}

PIDController::~PIDController()
{ }

double PIDController::getFeedBack(double desired, double current)
{
  prev_err_ = curr_err_;
  curr_err_ = desired - current;
  sum_err_ += curr_err_;
  
  return p_gain_*curr_err_ + i_gain_*sum_err_*control_time_sec_ + d_gain_*(curr_err_ - prev_err_)/control_time_sec_;
}


PreviewWalking::PreviewWalking()
{
  op3_kd_ = 0;
  mat_pelvis_to_rhip_ = robotis_framework::getTransformationXYZRPY(0, -0.035, 0, 0, 0, 0);
  mat_pelvis_to_lhip_ = robotis_framework::getTransformationXYZRPY(0,  0.035, 0, 0, 0, 0);

  mat_rhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0,  0.035, 0, 0, 0, 0);
  mat_lhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0, -0.035, 0, 0, 0, 0);

  //balance_error_ = heroehs::BalanceControlError::NoError;

  //mat_imu_frame_ref_ = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5*M_PI);
  //mat_imu_frame_ref_inv_ = mat_imu_frame_ref_.transpose();


  // sensor values
  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  current_right_fx_N_  =  current_right_fy_N_  = current_right_fz_N_  =0;
  current_right_tx_Nm_ =  current_right_ty_Nm_ = current_right_tz_Nm_ =0;
  current_left_fx_N_   = current_left_fy_N_    = current_left_fz_N_   =0;
  current_left_tx_Nm_  = current_left_ty_Nm_   = current_left_tz_Nm_  =0;

  //balance_index_ = 0;

  mat_g_to_acc_.resize(4, 1);
  mat_g_to_acc_.fill(0);

}

PreviewWalking::~PreviewWalking()
{
  if (op3_kd_ != 0)
    delete op3_kd_;
}

void PreviewWalking::setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis)
{
  walking_pattern_.setInitialPose(r_foot, l_foot, pelvis);
}

void PreviewWalking::initialize(double lipm_height_m, double preview_time_sec, double control_cycle_sec)
{
  op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);

  total_robot_mass_ = op3_kd_->calcTotalMass(0);
  right_dsp_fz_N_ = -0.5* total_robot_mass_ * 9.8;
  left_dsp_fz_N_  = -0.5* total_robot_mass_ * 9.8;
  right_ssp_fz_N_ = -total_robot_mass_ * 9.8;
  left_ssp_fz_N_  = -total_robot_mass_ * 9.8;

  walking_pattern_.initialize(lipm_height_m, preview_time_sec, control_cycle_sec);

  // initialize balance
  //balance_ctrl_.initialize(control_cycle_sec);
  //balance_ctrl_.setGyroBalanceEnable(true);
  //balance_ctrl_.setOrientationBalanceEnable(true);
  //balance_ctrl_.setForceTorqueBalanceEnable(true);

  mat_right_force_.resize(4,1);  mat_left_force_.resize(4,1);
  mat_right_torque_.resize(4,1); mat_left_torque_.resize(4,1);

  mat_right_force_.fill(0);  mat_left_force_.fill(0);
  mat_right_torque_.fill(0); mat_left_torque_.fill(0);

  for(int feed_forward_idx = 0; feed_forward_idx < 12; feed_forward_idx++)
  {
    leg_angle_feed_back_[feed_forward_idx].control_time_sec_ = control_cycle_sec;
    leg_angle_feed_back_[feed_forward_idx].p_gain_ = 1.0;
    leg_angle_feed_back_[feed_forward_idx].i_gain_ = 0;
    leg_angle_feed_back_[feed_forward_idx].d_gain_ = 0;
  }

  mat_g_to_acc_.resize(4, 1);
  mat_g_to_acc_.fill(0);
}

void PreviewWalking::start()
{
  walking_pattern_.start();
}

void PreviewWalking::addStepData(robotis_framework::StepData& step_data)
{
  walking_pattern_.addStepData(step_data);
}

void PreviewWalking::eraseLastStepData()
{
  walking_pattern_.eraseLastStepData();
}

int  PreviewWalking::getNumofRemainingUnreservedStepData()
{
  return walking_pattern_.getNumofRemainingUnreservedStepData();
}

void PreviewWalking::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  walking_pattern_.getReferenceStepDatafotAddition(ref_step_data_for_addition);
}

void PreviewWalking::process()
{
  walking_pattern_.process();

  // std::cout << walking_pattern_.mat_g_to_rfoot_ << std::endl << std::endl;
  // std::cout << walking_pattern_.mat_g_to_lfoot_ << std::endl << std::endl;
  // std::cout << walking_pattern_.mat_g_to_pelvis_ << std::endl << std::endl;

  mat_g_to_pelvis_ = walking_pattern_.mat_g_to_pelvis_;
  mat_pelvis_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_pelvis_);
  mat_g_to_rfoot_ = walking_pattern_.mat_g_to_rfoot_;
  mat_g_to_lfoot_ = walking_pattern_.mat_g_to_lfoot_;
  mat_robot_to_pelvis_ = walking_pattern_.mat_robot_to_pelvis_;

  mat_robot_to_rfoot_ = (mat_robot_to_pelvis_*mat_pelvis_to_g_)*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = (mat_robot_to_pelvis_*mat_pelvis_to_g_)*mat_g_to_lfoot_;

  //balance
  imu_data_mutex_lock_.lock();
  //balance_ctrl_.setCurrentGyroSensorOutput(current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_);
  //balance_ctrl_.setCurrentOrientationSensorOutput(current_imu_roll_rad_, current_imu_pitch_rad_);
  imu_data_mutex_lock_.unlock();
  ft_data_mutex_lock_.lock();
  //balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force_.coeff(0,0),  mat_right_force_.coeff(1,0),  mat_right_force_.coeff(2,0),
  //                                                    mat_right_torque_.coeff(0,0), mat_right_torque_.coeff(1,0), mat_right_torque_.coeff(2,0),
  //                                                    mat_left_force_.coeff(0,0),   mat_left_force_.coeff(1,0),   mat_left_force_.coeff(2,0),
  //                                                    mat_left_torque_.coeff(0,0),  mat_left_torque_.coeff(1,0),  mat_left_torque_.coeff(2,0));
  ft_data_mutex_lock_.unlock();

  //balance_index_ = walking_pattern_.current_balancing_index_;
  double r_target_fx_N = 0;
  double l_target_fx_N = 0;
  double r_target_fy_N = 0;
  double l_target_fy_N = 0;
  double r_target_fz_N = right_dsp_fz_N_;
  double l_target_fz_N = left_dsp_fz_N_;
  double target_fz_N  = 0;

  mat_g_to_acc_.coeffRef(0,0) = walking_pattern_.x_lipm_.coeff(2,0);
  mat_g_to_acc_.coeffRef(1,0) = walking_pattern_.y_lipm_.coeff(2,0);
  mat_robot_to_acc_ = (mat_robot_to_pelvis_* mat_pelvis_to_g_) * mat_g_to_acc_;

  // switch(balance_index_)
  //  {
  //  case 0:
  //    //fprintf(stderr, "DSP : START\n");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_dsp_fz_N_;
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  case 1:
  //    //fprintf(stderr, "DSP : R--O->L\n");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_dsp_fz_N_;
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  case 2:
  //    //fprintf(stderr, "SSP : L_BALANCING1\n");
  //    r_target_fx_N = 0;
  //    r_target_fy_N = 0;
  //    r_target_fz_N = 0;

  //    l_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    l_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    l_target_fz_N = left_ssp_fz_N_;
  //    target_fz_N = left_ssp_fz_N_;
  //    break;
  //  case 3:
  //    //fprintf(stderr, "SSP : L_BALANCING2\n");_completed
  //    r_target_fx_N = 0;
  //    r_target_fy_N = 0;
  //    r_target_fz_N = 0;

  //    l_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    l_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    l_target_fz_N = left_ssp_fz_N_;
  //    target_fz_N = left_ssp_fz_N_;
  //    break;
  //  case 4:
  //    //fprintf(stderr, "DSP : R--O<-L\n");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);_completed
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  case 5:
  //    //fprintf(stderr, "DSP : R<-O--L\n");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_dsp_fz_N_;
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  case 6:
  //    //fprintf(stderr, "SSP : R_BALANCING1\n");
  //    r_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_ssp_fz_N_;

  //    l_target_fx_N = 0;
  //    l_target_fy_N = 0;
  //    l_target_fz_N = 0;
  //    target_fz_N = -right_ssp_fz_N_;
  //    break;
  //  case 7:
  //    //fprintf(stderr, "SSP : R_BALANCING2\n");
  //    r_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_ssp_fz_N_;

  //    l_target_fx_N = 0;
  //    l_target_fy_N = 0;
  //    l_target_fz_N = 0;
  //    target_fz_N =  -right_ssp_fz_N_;
  //    break;
  //  case 8:
  //    //fprintf(stderr, "DSP : R->O--L");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_dsp_fz_N_;
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  case 9:
  //    //fprintf(stderr, "DSP : END");
  //    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
  //    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
  //    r_target_fz_N = right_dsp_fz_N_;
  //    l_target_fz_N = left_dsp_fz_N_;
  //    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
  //    break;
  //  default:
  //    break;
  //  }

  //std::cout << l_target_fz_N << " " << r_target_fz_N <<" ";

  l_target_fz_N = walking_pattern_.switching_ratio_*left_dsp_fz_N_ + left_dsp_fz_N_;
  r_target_fz_N = right_ssp_fz_N_ - l_target_fz_N;

//  std::cout << walking_pattern_.x_lipm_.coeff(0) << " " << walking_pattern_.x_lipm_.coeff(1) << " " << walking_pattern_.x_lipm_.coeff(2) << " "
//      << walking_pattern_.y_lipm_.coeff(0) << " " << walking_pattern_.y_lipm_.coeff(1) << " " << walking_pattern_.y_lipm_.coeff(2) << " "
//      << l_target_fx_N <<" " << l_target_fy_N<< " " << l_target_fz_N << " "
//      << r_target_fx_N <<" " << r_target_fy_N << " " << r_target_fz_N << " " << std::endl;

  //balance_ctrl_.setDesiredCOBGyro(0,0);
  //balance_ctrl_.setDesiredCOBOrientation(walking_pattern_.pose_g_to_pelvis_.roll, walking_pattern_.pose_g_to_pelvis_.pitch);
  //balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
  //                                        l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
  //balance_ctrl_.setDesiredPose(mat_robot_to_pelvis_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

  //balance_ctrl_.process(&balance_error_, &mat_robot_to_pelvis_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  //mat_pelvis_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_pelvis_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_pelvis_) * mat_pelvis_to_g_*mat_g_to_rfoot_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_pelvis_) * mat_pelvis_to_g_*mat_g_to_lfoot_);
  
  //rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_pelvis_ * mat_pelvis_to_robot_modified_) * mat_robot_to_rf_modified_);
  //lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_pelvis_ * mat_pelvis_to_robot_modified_) * mat_robot_to_lf_modified_);

  if( op3_kd_->calcInverseKinematicsForRightLeg(r_leg_out_angle_rad_, rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z,
      rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    ROS_ERROR("IK for RIGHT LEG is failed");
  }
  if( op3_kd_->calcInverseKinematicsForLeftLeg(l_leg_out_angle_rad_, lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z,
      lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    ROS_ERROR("IK for LEFT LEG is failed");
  }

  for(int i = 0; i < 6; i++)
  {
	  out_angle_rad_[i+0] = r_leg_out_angle_rad_[i];
	  out_angle_rad_[i+6] = l_leg_out_angle_rad_[i];
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = out_angle_rad_[angle_idx+0] + leg_angle_feed_back_[angle_idx+0].getFeedBack(out_angle_rad_[angle_idx],   curr_angle_rad_[angle_idx]);
    out_angle_rad_[angle_idx+6] = out_angle_rad_[angle_idx+6] + leg_angle_feed_back_[angle_idx+6].getFeedBack(out_angle_rad_[angle_idx+6], curr_angle_rad_[angle_idx+6]);
  }
}

bool PreviewWalking::isRunning()
{
  return walking_pattern_.isRunning();
}


void PreviewWalking::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_.x() = quat_x;
  quat_current_imu_.y() = quat_y;
  quat_current_imu_.z() = quat_z;
  quat_current_imu_.w() = quat_w;

  //mat_current_imu_ = (mat_imu_frame_ref_ * quat_current_imu_.toRotationMatrix()) * mat_imu_frame_ref_inv_;

  //current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  //current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));

  //std::cout << "gx : " << current_gyro_roll_rad_per_sec_ << " gy : " << current_gyro_pitch_rad_per_sec_
      //<< " x : " << current_imu_roll_rad_ << " y : " << current_imu_pitch_rad_ << std::endl;

  imu_data_mutex_lock_.unlock();
}

void PreviewWalking::setCurrentFTSensorOutput(double rfx, double rfy, double rfz, double rtx, double rty, double rtz,
    double lfx, double lfy, double lfz, double ltx, double lty, double ltz)
{
  ft_data_mutex_lock_.lock();
  current_right_fx_N_ = rfx;  current_right_fy_N_ = rfy;  current_right_fz_N_ = rfz;
  current_right_tx_Nm_= rtx; current_right_ty_Nm_= rty; current_right_tz_Nm_= rtz;

  current_left_fx_N_ = lfx;  current_left_fy_N_ = lfy;  current_left_fz_N_ = lfz;
  current_left_tx_Nm_ = ltx; current_left_ty_Nm_ = lty; current_left_tz_Nm_ = ltz;

  mat_right_force_(0,0) = current_right_fx_N_;
  mat_right_force_(1,0) = current_right_fy_N_;
  mat_right_force_(2,0) = current_right_fz_N_;
  mat_left_force_(0,0)  = current_left_fx_N_;
  mat_left_force_(1,0)  = current_left_fy_N_;
  mat_left_force_(2,0)  = current_left_fz_N_;

  mat_right_torque_(0,0) = current_right_tx_Nm_;
  mat_right_torque_(1,0) = current_right_ty_Nm_;
  mat_right_torque_(2,0) = current_right_tz_Nm_;
  mat_left_torque_(0,0) = current_left_tx_Nm_;
  mat_left_torque_(1,0) = current_left_ty_Nm_;
  mat_left_torque_(2,0) = current_left_tz_Nm_;

  mat_right_force_  = mat_robot_to_rfoot_*mat_right_force_;
  mat_right_torque_ = mat_robot_to_rfoot_*mat_right_torque_;

  mat_left_force_  = mat_robot_to_lfoot_*mat_left_force_;
  mat_left_torque_ = mat_robot_to_lfoot_*mat_left_torque_;
  ft_data_mutex_lock_.unlock();
}