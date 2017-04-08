#include <iostream>
#include "RadarMeasurement.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// -----------------------------------------------------------------------------
// @brief  Constructor
// -----------------------------------------------------------------------------
RadarMeasurement::RadarMeasurement()
: H_(3, 4)
, R_(3, 3)
{
  R_ << 0.09,      0,    0,
           0, 0.0009,    0,
           0,      0, 0.09;
}

// -----------------------------------------------------------------------------
// @brief  Update
//
// Measurement update for radar
//
// @param[in/out] ekf    Extended Kalman filter used for update
// -----------------------------------------------------------------------------
void RadarMeasurement::Update(KalmanFilter &ekf) 
{
  H_ = CalculateJacobian(ekf.x_);
  ekf.UpdateEKF(raw_measurements_, H_, R_);
}

// -----------------------------------------------------------------------------
// @brief  CalculateJacobian
//
// Calculate Jacobian for the radar measurement
//
// @param[in] x_state   Kalman filter state vector
// -----------------------------------------------------------------------------
MatrixXd RadarMeasurement::CalculateJacobian(const VectorXd &x_state) 
{
  // Get the state vector elements
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);

  //check division by zero
  if(px+py == 0) 
  {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return MatrixXd::Zero(3, 4);
  }

  MatrixXd Hj(3,4);

  //compute the Jacobian matrix
  const double rho_squre = px * px + py * py;
  const double rho = sqrt(rho_squre);
  const double rho_cube_sqrt = pow(rho_squre, 3.0/2.0);

  Hj <<                           px/rho,                           py/rho,      0,      0,
                           -py/rho_squre,                     px/rho_squre,      0,      0,
        (py*(vx*py-vy*px))/rho_cube_sqrt, (px*(vy*px-vx*py))/rho_cube_sqrt, px/rho, py/rho;

  return Hj;
}
