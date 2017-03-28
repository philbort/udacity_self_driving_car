#ifndef EXTENDEDKF_LASER_MEASUREMENT_H
#define EXTENDEDKF_LASER_MEASUREMENT_H


#include "measurement_package.h"
#include "kalman_filter.h"

class LaserMeasurement : public MeasurementPackage 
{
public:
  // -----------------------------------------------------------------------------
  // @brief  Constructor
  // -----------------------------------------------------------------------------
  LaserMeasurement();

  // -----------------------------------------------------------------------------
  // @brief  Destructor
  // -----------------------------------------------------------------------------
  ~LaserMeasurement() {}

  // -----------------------------------------------------------------------------
  // @brief  Update
  //
  // Measurement update for laser
  //
  // @param[in/out] ekf    Extended Kalman filter used for update
  // -----------------------------------------------------------------------------
  void Update(KalmanFilter &ekf);

private:
  // Measurement noise covariance matrix
  Eigen::MatrixXd R_;

  // Measurement design matrix
  Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_LASER_MEASUREMENT_H
