#include <iostream>
#include "FusionEKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// -----------------------------------------------------------------------------
// @brief  Constructor
//
// @param[in] n    State vector dimension
// -----------------------------------------------------------------------------
FusionEKF::FusionEKF
(
  int n
)
: is_initialized_(false)
, previous_timestamp_(0)
, F_(n, n)
, Q_(n, n)
, noise_ax_(9.0)
, noise_ay_(9.0)
, ekf_(n)
{
}

// -----------------------------------------------------------------------------
// @brief  ProcessMeasurement
//
// Process the measurement package.
//
// @param[in/out] measurement_pack    Measurement package
// -----------------------------------------------------------------------------
bool FusionEKF::ProcessMeasurement
(
  MeasurementPackage &measurement_pack
) 
{
  const VectorXd z = measurement_pack.raw_measurements_;

  // prevent devision by zero
  if (!z(0) || !z(1)) return false;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    // Initialize the state vector with the first measurement.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      const double rho = z(0);
      const double phi = z(1);
      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;
    } 
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      ekf_.x_ << z(0), z(1), 0, 0;
    }

    // Initialize the state covariance matrix
    ekf_.P_ << 1, 0,    0,    0,
               0, 1,    0,    0,
               0, 0, 1000,    0,
               0, 0,    0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;

    return true;
  }

  // Time Update
  const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt2 * dt2;

  previous_timestamp_ = measurement_pack.timestamp_;

  F_ << 1, 0, dt,  0,
        0, 1,  0, dt,
        0, 0,  1,  0,
        0, 0,  0,  1;

  Q_ << (dt4 / 4.0) * noise_ax_,                       0, (dt3 / 2.0) * noise_ax_,                       0,
                              0, (dt4 / 4.0) * noise_ay_,                       0, (dt3 / 2.0) * noise_ay_,
        (dt3 / 2.0) * noise_ax_,                       0,         dt2 * noise_ax_,                       0,
                              0, (dt3 / 2.0) * noise_ay_,                       0,         dt2 * noise_ay_;

  ekf_.Predict(F_, Q_);

  // Measurement Update
  measurement_pack.Update(ekf_);

  return true;
}

// -----------------------------------------------------------------------------
// @brief  CalculateRMSE
//
// Function to calculate the root-mean-squared error.
//
// @param[in] estimations    estimation vector
// @param[in] ground_truth   ground truth vector
// -----------------------------------------------------------------------------
VectorXd FusionEKF::CalculateRMSE
(
  const vector<VectorXd> &estimations,
  const vector<VectorXd> &ground_truth
) 
{
  const size_t n = estimations.size();
  const size_t m = ground_truth.size();

  if(n + m == 0)
  {
    cout << "estimations and ground truth vectors must not be empty" << endl;
    return VectorXd::Zero(4);
  }
  if(n != m)
  {
    cout << "estimations and ground truth vectors ust be of equal dimensions." << endl;
    return VectorXd::Zero(4);
  }

  // Squared error initialized to zeros
  VectorXd se = VectorXd::Zero(4);

  // Accumulate the squared errors
  for (int i = 0; i < n; ++i) 
    se += static_cast<VectorXd> ((estimations[i]-ground_truth[i]).array().square());

  // Return root-mean-squared error
  return (se.array()/n).sqrt();
}

