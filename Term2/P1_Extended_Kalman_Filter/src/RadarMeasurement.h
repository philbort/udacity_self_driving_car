#ifndef EXTENDEDKF_RADAR_MEASUREMENT_H
#define EXTENDEDKF_RADAR_MEASUREMENT_H


#include "measurement_package.h"
#include "tools.h"
#include "kalman_filter.h"

class RadarMeasurement : public MeasurementPackage 
{
public:
  // -----------------------------------------------------------------------------
  // @brief  Constructor
  // -----------------------------------------------------------------------------
  RadarMeasurement();

  // -----------------------------------------------------------------------------
  // @brief  Destructor
  // -----------------------------------------------------------------------------
  ~RadarMeasurement() {}

  // -----------------------------------------------------------------------------
  // @brief  Update
  //
  // Measurement update for radar
  //
  // @param[in/out] ekf    Extended Kalman filter used for update
  // -----------------------------------------------------------------------------
  void Update(KalmanFilter &ekf);

  // -----------------------------------------------------------------------------
  // @brief  CalculateJacobian
  //
  // Calculate Jacobian for the radar measurement
  //
  // @param[in] x_state   Kalman filter state vector
  // -----------------------------------------------------------------------------
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

private:
  // Measurement noise covariance matrix
  Eigen::MatrixXd R_;

  // Meausrement design matrix
  Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_RADAR_MEASUREMENT_H
