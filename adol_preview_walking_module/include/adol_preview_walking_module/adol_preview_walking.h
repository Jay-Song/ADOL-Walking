/*
 * aodl_preview_walking.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef ADOL_PREVIEW_WALKING_MODULE_PREVIEW_WALKING_H_
#define ADOL_PREVIEW_WALKING_MODULE_PREVIEW_WALKING_H_

#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "adol_preview_walking_pattern_generator/preview_walking_pattern_generator.h"
//#include "heroehs_pd_balance_controller/heroehs_pd_balance_controller.h"

#include "robotis_framework_common/motion_module.h"

namespace adol
{

class PreviewWalking 
{
public:
  PreviewWalking();
  virtual ~PreviewWalking();

  void setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis);

  void initialize(double lipm_height_m, double preview_time_sec, double control_cycle_sec);
  void start();

  void process();
  bool isRunning();

  void addStepData(robotis_framework::StepData& step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();
  void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

  Eigen::Matrix4d mat_pelvis_to_rhip_, mat_rhip_to_pelvis_;
  Eigen::Matrix4d mat_pelvis_to_lhip_, mat_lhip_to_pelvis_;
  Eigen::Matrix4d mat_g_to_pelvis_, mat_g_to_rfoot_, mat_g_to_lfoot_;
  Eigen::Matrix4d mat_robot_to_pelvis_, mat_robot_to_rfoot_, mat_robot_to_lfoot_;
  Eigen::Matrix4d mat_pelvis_to_g_;

  Eigen::MatrixXd mat_robot_to_pelvis_modified_, mat_robot_to_rf_modified_, mat_robot_to_lf_modified_;
  Eigen::MatrixXd mat_pelvis_to_robot_modified_;

  robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;

  double r_leg_out_angle_rad_[6];
  double l_leg_out_angle_rad_[6];
  double out_angle_rad_[12];
  double curr_angle_rad_[12];
  double curr_torque_Nm_[12];

  //heroehs::PDController leg_angle_feed_back_[12];

  // balance control
  //int balance_index_;
  //int balance_error_;
  //heroehs::BalanceControlUsingPDController balance_ctrl_;

  void setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w);
  void setCurrentFTSensorOutput(double rfx, double rfy, double rfz, double rtx, double rty, double rtz,
      double lfx, double lfy, double lfz, double ltx, double lty, double ltz);

  // sensor value
  //imu
  Eigen::Quaterniond quat_current_imu_;
  Eigen::Matrix3d mat_current_imu_;
  Eigen::Matrix3d mat_imu_frame_ref_, mat_imu_frame_ref_inv_;
  double current_imu_roll_rad_, current_imu_pitch_rad_;
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  Eigen::MatrixXd mat_right_force_, mat_left_force_;
  Eigen::MatrixXd mat_right_torque_, mat_left_torque_;
  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

 Eigen::MatrixXd mat_g_to_acc_, mat_robot_to_acc_;

//private:
  PreviewWalkingPatternGenerator walking_pattern_;
  robotis_op::OP3KinematicsDynamics* op3_kd_;

  double total_robot_mass_;
  double right_dsp_fz_N_, left_dsp_fz_N_;
  double right_ssp_fz_N_, left_ssp_fz_N_;

  boost::mutex imu_data_mutex_lock_;
  boost::mutex ft_data_mutex_lock_;
};

}

#endif /* ADOL_PREVIEW_WALKING_MODULE_PREVIEW_WALKING_H_ */