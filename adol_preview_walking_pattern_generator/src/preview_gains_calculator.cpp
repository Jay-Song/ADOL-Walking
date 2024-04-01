/*
 * online_walking_pattern_generator.cpp
 *
 *  Created on: 2024. 4. 1.
 *      Author: Jay-Song
 */

#include "adol_preview_walking_pattern_generator/preview_gains_calculator.h"

using namespace adol;

DLQR::DLQR()
{
  MAX_ITERATION_NUM = 5000;
  TARGET_ERROR = 1.0e-11;
}

DLQR::~DLQR()
{  }

void DLQR::computeDLQRGains(Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Q, Eigen::MatrixXd R)
{
  Eigen::MatrixXd P_new;
  Eigen::MatrixXd temp, temp_inv;

  // std::cout << A.size() << std::endl;
  // std::cout << b.size() << std::endl;
  // std::cout << Q.size() << std::endl;
  // std::cout << R.size() << std::endl;

  P = Q;

  for (int iter = 0; iter < MAX_ITERATION_NUM; iter++)
  {
    temp = R + (b.transpose() * P) * b;
    temp_inv = temp.inverse();
    P_new = Q + (A.transpose() * P) * A - (((((A.transpose() * P) * b) * temp_inv) * b.transpose()) * P) * A;

    double error = (P_new - P).maxCoeff();

    P = P_new;

    if (error < TARGET_ERROR)
    {
      std::cout << iter << " " << error << std::endl
                << std::endl;
      break;
    }
  }

  P = P_new;
  temp = R + (b.transpose() * P) * b;
  temp_inv = temp.inverse();
  K = ((temp_inv * b.transpose()) * P) * A;
}

PreviewControlGainCalculator::PreviewControlGainCalculator(double preview_time_sec, double control_time_sec, double lipm_height_m)
{
  preview_time_sec_ = preview_time_sec;
  control_time_sec_ = control_time_sec;
  lipm_height_m_ = lipm_height_m;

  preview_size_ = floor(preview_time_sec_ / control_time_sec_);
}

PreviewControlGainCalculator::~PreviewControlGainCalculator()
{ }

void PreviewControlGainCalculator::initialize()
{
  double t = control_time_sec_;
  mat_A << 1.0, t, t * t / 2,
      0, 1.0, t,
      0, 0, 1.0;

  vec_b << t * t * t / 6,
      t * t / 2,
      t;

  rvec_c << 1.0, 0.0, -lipm_height_m_ / 9.81;

  A_.coeffRef(0, 0) = 1.0;
  A_.coeffRef(1, 0) = 0.0;
  A_.coeffRef(2, 0) = 0.0;
  A_.coeffRef(3, 0) = 0.0;

  A_.block<1, 3>(0, 1) = rvec_c * mat_A;
  A_.block<3, 3>(1, 1) = mat_A;

  b_.block<1, 1>(0, 0) = rvec_c * vec_b;
  b_.block<3, 1>(1, 0) = vec_b;

  c_ << 1, 0, 0, 0;

  R_.resize(1, 1);
  R_ << 1.0e-6;

  Q_ = Eigen::Matrix4d::Identity();
  Q_.coeffRef(3, 3) = 0.0;

  f_.resize(1, preview_size_);
  f_.fill(0);
}

void PreviewControlGainCalculator::computePreviewControlGains()
{
  dlqr_.computeDLQRGains(A_, b_, Q_, R_);
  K_ = dlqr_.K;
  P_ = dlqr_.P;

  // std::cout << P_ << std::endl
  //           << std::endl;
  // std::cout << K_ << std::endl
  //           << std::endl;

  Eigen::MatrixXd temp = R_ + (b_.transpose() * P_) * b_;
  Eigen::MatrixXd temp_inv = temp.inverse();

  Eigen::MatrixXd temp1 = temp.inverse() * b_.transpose();

  Eigen::Matrix4d temp2 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d temp3 = (A_ - b_ * K_);
  temp3.transposeInPlace();

  Eigen::Matrix4d temp4 = P_ * c_.transpose();

  f_.block<1, 1>(0, 0) = (temp1 * temp2) * temp4;
  for (int idx = 1; idx < preview_size_; idx++)
  {
    temp2 = temp2 * temp3;
    f_.block<1, 1>(0, idx) = (temp1 * temp2) * temp4;
  }

  // std::cout << f_ << std::endl;
}