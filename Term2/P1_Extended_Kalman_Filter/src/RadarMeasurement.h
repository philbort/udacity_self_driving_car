#ifndef EXTENDEDKF_RADAR_MEASUREMENT_H
#define EXTENDEDKF_RADAR_MEASUREMENT_H


#include "measurement_package.h"
#include "tools.h"
#include "kalman_filter.h"

class RadarMeasurement : public MeasurementPackage 
{
public:
    RadarMeasurement();

    ~RadarMeasurement() {}

    void Update(KalmanFilter &ekf);

    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

private:
    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_RADAR_MEASUREMENT_H
