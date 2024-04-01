/*
 * online_pelvis_xy_calculator.cpp
 *
 *  Created on: April 1, 2024
 *      Author: Jay Song
 */

#include "adol_preview_walking_pattern_generator/online_pelvis_xy_calculator.h"

using namespace adol;


OnlinePelvisXYCalculator::OnlinePelvisXYCalculator()
{
  lipm_height_m_ = 0.7;

  control_time_sec_ = 0.008;
  preview_time_sec_ = 1.6;

  preview_size_ = round(preview_time_sec_/control_time_sec_);

  k_s_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;

  prev_gains_ = 0;
}

OnlinePelvisXYCalculator::~OnlinePelvisXYCalculator()
{  }

void OnlinePelvisXYCalculator::initialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  lipm_height_m_ = lipm_height_m;
  preview_time_sec_ = preview_time_sec;
  control_time_sec_ = control_time_sec;

  double t = control_time_sec_;
  double g = 9.8;
  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t, t*t/2.0,
      0,  1,   t,
      0,  0,   1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -lipm_height_m_/g;

  preview_size_ = round(preview_time_sec_/control_time_sec_);

  k_s_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;

  prev_gains_ = new PreviewControlGainCalculator(1.6, control_time_sec, 0.2);
  prev_gains_->initialize();
  prev_gains_->computePreviewControlGains();

  k_s_ = prev_gains_->K_.coeff(0,0);
  k_x_ << prev_gains_->K_.coeff(0,1), prev_gains_->K_.coeff(0,2), prev_gains_->K_.coeff(0,3);

  f_ = prev_gains_->f_;

  u_x.resize(1,1);   u_y.resize(1,1);
  u_x.fill(0);       u_y.fill(0);

  x_lipm_.fill(0.0); y_lipm_.fill(0.0);
}

void OnlinePelvisXYCalculator::reInitialize(double lipm_height_m, double preview_time_sec, double control_time_sec)
{
  initialize(lipm_height_m, preview_time_sec, control_time_sec);
}

void OnlinePelvisXYCalculator::reInitialize()
{
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);
}


void OnlinePelvisXYCalculator::calcNextPelvisXY(const Eigen::VectorXd& reference_zmp_x,  const Eigen::VectorXd& reference_zmp_y)
{
  //  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + f_*reference_zmp_x;
  //  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + f_*reference_zmp_y;

  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + (f_*reference_zmp_x).coeff(0,0);
  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + (f_*reference_zmp_y).coeff(0,0);

  x_lipm_ = A_*x_lipm_ + b_*u_x;
  y_lipm_ = A_*y_lipm_ + b_*u_y;

  sum_of_cx_ += c_(0,0)*x_lipm_(0,0) +  c_(0,1)*x_lipm_(1,0) +  c_(0,2)*x_lipm_(2,0);
  sum_of_cy_ += c_(0,0)*y_lipm_(0,0) +  c_(0,1)*y_lipm_(1,0) +  c_(0,2)*y_lipm_(2,0);

  sum_of_zmp_x_ += reference_zmp_x.coeff(0);
  sum_of_zmp_y_ += reference_zmp_y.coeff(0);
}



