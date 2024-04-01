/*
 * online_pelvis_xy_calculator.h
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */

#ifndef ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_PELVIS_XY_CALCULATOR_H_
#define ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_PELVIS_XY_CALCULATOR_H_

#include "robotis_math/robotis_math.h"
#include "adol_preview_walking_pattern_generator/preview_gains_calculator.h"

namespace adol
{

class OnlinePelvisXYCalculator
{

public:
  OnlinePelvisXYCalculator();
  ~OnlinePelvisXYCalculator();

  void initialize(double lipm_height_m, double preview_time_sec, double control_time_sec);
  void reInitialize(double lipm_height_m, double preview_time_sec, double control_time_sec);
  void reInitialize();

  void calcNextPelvisXY(const Eigen::VectorXd& reference_zmp_x,  const Eigen::VectorXd& reference_zmp_y);
  Eigen::Vector3d x_lipm_, y_lipm_;

private:
  double lipm_height_m_;
  double control_time_sec_;
  double preview_time_sec_;

  int preview_size_;

  Eigen::MatrixXd A_, b_, c_;
  Eigen::RowVector3d k_x_;
  Eigen::RowVectorXd f_;

  Eigen::MatrixXd u_x, u_y;

  double k_s_;
  double sum_of_zmp_x_ ;
  double sum_of_zmp_y_ ;
  double sum_of_cx_ ;
  double sum_of_cy_ ;

  PreviewControlGainCalculator* prev_gains_;
};


}

#endif /* ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_ONLINE_PELVIS_XY_CALCULATOR_H_ */
