#include "FusionEKF.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEKF::FusionEKF
(
  int n
)
: is_initialized_(false)
, previous_timestamp_(0)
, F_(n, n)
, Q_(n, n)
, noise_ax(9.0)
, noise_ay(9.0)
, ekf_(n)
{
}

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
    /**
      * Initialize the state ekf_.x_ with the first measurement.
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // Convert radar from polar to cartesian coordinates and initialize state
      // output the estimation in the cartesian coordinates. We do the conversion to
      // cartesian because we always want x_ in cartesian coordinates.
      const double rho = z(0);
      const double phi = z(1);
      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;
    } 
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      ekf_.x_ << z(0), z(1), 0, 0;
    }

    ekf_.P_ << 1, 0,    0,    0,
               0, 1,    0,    0,
               0, 0, 1000,    0,
               0, 0,    0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;

    return true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt2 * dt2;

  previous_timestamp_ = measurement_pack.timestamp_;

  F_ << 1, 0, dt,  0,
             0, 1,  0, dt,
             0, 0,  1,  0,
             0, 0,  0,  1;

  Q_ << (dt4 / 4.0) * noise_ax,                      0, (dt3 / 2.0) * noise_ax,                      0,
                             0, (dt4 / 4.0) * noise_ay,                      0, (dt3 / 2.0) * noise_ay,
        (dt3 / 2.0) * noise_ax,                      0,         dt2 * noise_ax,                      0,
                             0, (dt3 / 2.0) * noise_ay,                      0,         dt2 * noise_ay;

  ekf_.Predict(F_, Q_);

  /*****************************************************************************
   *  Measurement Update
   ****************************************************************************/
  measurement_pack.Update(ekf_);

  return true;
}
