#include "LaserMeasurement.h"

using Eigen::MatrixXd;

LaserMeasurement::LaserMeasurement()
: R_(2, 2)
, H_(2, 4) 
{
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  R_ << 0.0225,      0,
             0, 0.0225;
}

void LaserMeasurement::Update
(
	KalmanFilter &ekf
) 
{
  ekf.Update(raw_measurements_, H_, R_);
}
