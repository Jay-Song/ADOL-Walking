/*
 * online_endpoint_calculator.h
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */

#ifndef ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_ENDPOINT_CALCULATOR_H_
#define ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_ENDPOINT_CALCULATOR_H_

#include <boost/thread.hpp>
#include "adol_preview_walking_pattern_generator/online_pelvis_xy_calculator.h"

namespace adol
{

class OnlineEndpointCalculator
{
public:
  OnlineEndpointCalculator();
  ~OnlineEndpointCalculator();

  void initialize(double lipm_height_m, double preview_time_sec, double control_time_sec);
  void reInitialize();

  void addStepData(robotis_framework::StepData step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();
  void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

  void setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis);

  void calcDesiredPose();

  void start();
  bool isRunning();

  Eigen::VectorXd reference_zmp_x_, reference_zmp_y_;
  robotis_framework::Pose3D present_right_foot_pose_, present_left_foot_pose_, present_body_pose_;

  int current_balancing_index_;
  int current_step_data_status_;

  double switching_ratio_;

  Eigen::Vector3d x_lipm_, y_lipm_;

private:
  OnlinePelvisXYCalculator xy_calculator_;

  void calcStepIdxData();
  void calcRefZMP();
  void calcSmoothRefZMP();
  void calcEndPoint();

  int preview_size_;
  double walking_time_;    //Absolute Time
  double reference_time_;  //Absolute Time
  double preview_time_sec_;
  double control_time_sec_;

  std::vector<robotis_framework::StepData> added_step_data_;
  robotis_framework::StepData current_step_data_;
  robotis_framework::StepData reference_step_data_for_addition_;
  robotis_framework::Pose3D initial_right_foot_pose_, initial_left_foot_pose_, initial_body_pose_;
  robotis_framework::Pose3D previous_step_right_foot_pose_, previous_step_left_foot_pose_, previous_step_body_pose_;


  robotis_framework::FifthOrderPolynomialTrajectory foot_x_tra_,    foot_y_tra_,     foot_z_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_roll_tra_, foot_pitch_tra_, foot_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_roll_tra_, body_pitch_tra_, body_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_swap_tra_;

  Eigen::VectorXi step_idx_data_;

  bool running;

  boost::mutex step_data_mutex_lock_;

  robotis_framework::FifthOrderPolynomialTrajectory smooth_tra_;
};

}

#endif /* ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_ENDPOINT_CALCULATOR_H_ */
