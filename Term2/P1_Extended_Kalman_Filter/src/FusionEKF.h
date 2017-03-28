#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF 
{
public:
  // -----------------------------------------------------------------------------
  // @brief  Constructor
  //
  // @param[in] n    State vector dimension
  // -----------------------------------------------------------------------------
  FusionEKF(int n);

  // -----------------------------------------------------------------------------
  // @brief  Destructor
  // -----------------------------------------------------------------------------
  virtual ~FusionEKF() {}

  // -----------------------------------------------------------------------------
  // @brief  ProcessMeasurement
  //
  // Process the measurement package.
  //
  // @param[in/out] measurement_pack    Measurement package
  // -----------------------------------------------------------------------------
  bool ProcessMeasurement(MeasurementPackage &measurement_pack);

  // Kalman Filter 
  KalmanFilter ekf_;

private:

  // Initialization flag
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute RMSE
  Tools tools;

  // State transition matrix
  Eigen::MatrixXd F_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // Acceleration noise components
  double noise_ax;
  double noise_ay;
};

#endif /* FusionEKF_H_ */
