#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init
(
    VectorXd &x_in, 
    MatrixXd &P_in, 
    MatrixXd &F_in,
    MatrixXd &H_in, 
    MatrixXd &R_in, 
    MatrixXd &Q_in
) 
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict(long delta_T) {
    if (delta_T != delta_T_)
    {

    }
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    const VectorXd y = z - H_ * x_;
    const MatrixXd S = H_ * P_ * H_.transpose() + R_;
    const MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    x_ += K * y;
    P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
