#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// -----------------------------------------------------------------------------
// @brief  Constructor
//
// @param[in] n    State vector dimension
// -----------------------------------------------------------------------------
KalmanFilter::KalmanFilter(int n)
: x_(n)
, P_(n, n)
{
}

// -----------------------------------------------------------------------------
// @brief  Init
//
// Initialize Kalman filter state and state covariance matrix
//
// @param[in] x_in    Initial state vector
// @param[in] P_in    Initial state vector covariance matrix
// -----------------------------------------------------------------------------
void KalmanFilter::Init(const VectorXd &x_in, 
                        const MatrixXd &P_in) 
{
  x_ = x_in;
  P_ = P_in;
}

// -----------------------------------------------------------------------------
// @brief  Predict
//
// Kalman filter time update (prediction)
//
// @param[in] F    State transition matrix
// @param[in] Q    State covariance matrix
// -----------------------------------------------------------------------------
void KalmanFilter::Predict(const MatrixXd &F,
                           const MatrixXd &Q) 
{
  // Propagate state vector
  x_ = F * x_;

  // Propagate stave covariance matrix
  P_ = F * P_ * F.transpose() + Q;
}

// -----------------------------------------------------------------------------
// @brief  Update
//
// Linear Kalman filter measurement update
//
// @param[in] z    Measurement vector
// @param[in] H    Measurement design matrix
// @param[in] R    Measurement noise covariance matrix
// -----------------------------------------------------------------------------
void KalmanFilter::Update(const VectorXd &z,
                          const Eigen::MatrixXd &H,
                          const Eigen::MatrixXd &R) 
{
  // Measurement innovation
  const VectorXd y = z - H * x_;

  // Measurement innovation covariance matrix
  const MatrixXd S = H * P_ * H.transpose() + R;

  // Kalman gain
  const MatrixXd K = P_ * H.transpose() * S.inverse();

  // Update state vector
  x_ += K * y;

  // Update state covariance matrix
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
}

// -----------------------------------------------------------------------------
// @brief  Update
//
// Extended Kalman filter measurement update
//
// @param[in] z    Measurement vector
// @param[in] H    Measurement design matrix
// @param[in] R    Measurement noise covariance matrix
// -----------------------------------------------------------------------------
void KalmanFilter::UpdateEKF(const VectorXd &z,
                             const Eigen::MatrixXd &H,
                             const Eigen::MatrixXd &R) 
{
  // Get the individual states
  const float px = x_(0);
  const float py = x_(1);
  const float vx = x_(2);
  const float vy = x_(3);

  // Form the linearization points
  const float rho = sqrt(px * px + py * py);
  const float phi = atan(py / px);
  const float rho_dot = (px * vx + py * vy) / rho;

  VectorXd hx(3);
  hx << rho, phi, rho_dot;

  // Measurement innovation
  const VectorXd y = z - hx;

  // Measurement innovation covariance matrix
  const MatrixXd S = H * P_ * H.transpose() + R;

  // Kalman gain
  const MatrixXd K = P_ * H.transpose() * S.inverse();

  // Update the state vector
  x_ += K * y;

  // Update the state covariance matrix
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
}
