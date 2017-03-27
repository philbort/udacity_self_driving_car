#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF 
{
public:
  /**
  * Constructor.
  */
  FusionEKF(int n);

  /**
  * Destructor.
  */
  ~FusionEKF() {}

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  bool ProcessMeasurement(MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  // State transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  //acceleration noise components
  double noise_ax;
  double noise_ay;
};

#endif /* FusionEKF_H_ */
