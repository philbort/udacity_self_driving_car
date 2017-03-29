#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"

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

  // -----------------------------------------------------------------------------
  // @brief  CalculateRMSE
  //
  // Function to calculate the root-mean-squared error.
  //
  // @param[in] estimations    estimation vector
  // @param[in] ground_truth   ground truth vector
  // -----------------------------------------------------------------------------
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  // Kalman Filter 
  KalmanFilter ekf_;

private:

  // Initialization flag
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // State transition matrix
  Eigen::MatrixXd F_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // Acceleration noise components
  double noise_ax_;
  double noise_ay_;
};

#endif /* FusionEKF_H_ */
