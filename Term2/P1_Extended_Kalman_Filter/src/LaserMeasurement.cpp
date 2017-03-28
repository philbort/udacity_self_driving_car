#include "LaserMeasurement.h"

using Eigen::MatrixXd;

// -----------------------------------------------------------------------------
// @brief  Constructor
// -----------------------------------------------------------------------------
LaserMeasurement::LaserMeasurement()
: R_(2, 2)
, H_(2, 4) 
{
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  R_ << 0.0225,      0,
             0, 0.0225;
}

// -----------------------------------------------------------------------------
// @brief  Update
//
// Measurement update for laser
//
// @param[in/out] ekf    Extended Kalman filter used for update
// -----------------------------------------------------------------------------
void LaserMeasurement::Update
(
  KalmanFilter &ekf
) 
{
  ekf.Update(raw_measurements_, H_, R_);
}
