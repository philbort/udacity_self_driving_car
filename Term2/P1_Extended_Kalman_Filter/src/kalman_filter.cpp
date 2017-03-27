#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

KalmanFilter::KalmanFilter(int n)
: x_(n)
, P_(4, 4)
{
}

void KalmanFilter::Init
( 
  const VectorXd &x_in, 
  const MatrixXd &P_in
) 
{
  x_ = x_in;
  P_ = P_in;
}

void KalmanFilter::Predict
(
  const MatrixXd &F,
  const MatrixXd &Q
) 
{
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::Update
(
  const VectorXd &z,
  const Eigen::MatrixXd &H_,
  const Eigen::MatrixXd &R_
) 
{
  const VectorXd y = z - H_ * x_;
  const MatrixXd S = H_ * P_ * H_.transpose() + R_;
  const MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF
(
  const VectorXd &z,
  const Eigen::MatrixXd &H_,
  const Eigen::MatrixXd &R_
) 
{
  const float px = x_(0);
  const float py = x_(1);
  const float vx = x_(2);
  const float vy = x_(3);

  const float rho = sqrt(px * px + py * py);
  const float phi = atan(py / px);
  const float rho_dot = (px * vx + py * vy) / rho;

  VectorXd hx(3);
  hx << rho, phi, rho_dot;

  const VectorXd y = z - hx;
  const MatrixXd S = H_ * P_ * H_.transpose() + R_;
  const MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}