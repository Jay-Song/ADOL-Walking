/*
 * preview_gains_calculator.h
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */

#ifndef ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_PREVIEW_GAINS_CALCULATOR_H_
#define ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_PREVIEW_GAINS_CALCULATOR_H_

#include "robotis_math/robotis_math.h"

namespace adol
{

class DLQR
{
public:
  DLQR();
  ~DLQR();

  void computeDLQRGains(Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Q, Eigen::MatrixXd R);

  int MAX_ITERATION_NUM;
  double TARGET_ERROR;

  Eigen::MatrixXd K;
  Eigen::MatrixXd P;
};


class PreviewControlGainCalculator
{
public:

  PreviewControlGainCalculator(double preview_time_sec, double control_time_sec, double lipm_height_m);

  ~PreviewControlGainCalculator();

  void initialize();
  void computePreviewControlGains();

  DLQR dlqr_;

  uint32_t preview_size_;
  double preview_time_sec_, control_time_sec_, lipm_height_m_;

  Eigen::Matrix3d mat_A;
  Eigen::Vector3d vec_b;
  Eigen::RowVector3d rvec_c;

  Eigen::Matrix4d A_, Q_;
  Eigen::Vector4d b_;
  Eigen::RowVector4d c_;
  Eigen::MatrixXd R_;

  Eigen::Matrix4d P_;
  Eigen::RowVector4d K_;

  Eigen::RowVectorXd f_;
};

}

#endif /* ADOL_PREVIEW_WALKING_PATTERN_GENERATOR_PREVIEW_GAINS_CALCULATOR_H_ */
