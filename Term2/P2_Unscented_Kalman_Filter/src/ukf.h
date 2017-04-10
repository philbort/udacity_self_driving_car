#ifndef UKF_H
#define UKF_H

#include <vector>
#include "measurement_package.h"
#include "Eigen/Dense"

class UKF 
{
public:

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // Normalised Innovation Squared (NIS) for radar
  double NIS_radar_;

  // Normalised Innovation Squared (NIS) for laser
  double NIS_laser_;

  // Design matrix for laser
  Eigen::MatrixXd H_laser_;

  // Measurement covariance matrix for laser
  Eigen::MatrixXd R_laser_;

  /*----------------------------------------------------------------------------
    @brief  Constructor

    Initialize all class variables and start the minimum path finder function.

    @param[in] n             Size of the state vector
    @param[in] use_laser     Flag to use laser
    @param[in] user_radar    Flag to use radar
   ----------------------------------------------------------------------------*/
  UKF(const int n = 5, 
      const bool use_laser = true, 
      const bool user_radar = true);

  /*-----------------------------------------------------------------------------
    @brief  Destructor
   ----------------------------------------------------------------------------*/
  virtual ~UKF() {}

  /*----------------------------------------------------------------------------
    @brief  ProcessMeasurement

    Process the filter update given a new measurement.

    @param[in] meas_package    The current measurement package

    @return True if successful, false otherwise.
  ----------------------------------------------------------------------------*/
  bool ProcessMeasurement(const MeasurementPackage meas_package);

  /*----------------------------------------------------------------------------
    @brief  Prediction

    Unscented Kalman filter prediction (time update)

    @param[in] delta_t    Time interval between k and k+1 [s]
  ----------------------------------------------------------------------------*/
  void Prediction(const double delta_t);

  /*----------------------------------------------------------------------------
    @brief  UpdateLidar

    Linear Kalman filter Lidar measurement update

    @param[in] z    Lidar measurement
  ----------------------------------------------------------------------------*/
  void UpdateLidar(const Eigen::VectorXd & z);

  /*----------------------------------------------------------------------------
    @brief  UpdateRadar

    Unscented Kalman filter Radar measurement update

    @param[in] z    Radar measurement
  ----------------------------------------------------------------------------*/
  void UpdateRadar(const Eigen::VectorXd & z);


  /*----------------------------------------------------------------------------
    @brief  CalculateRMSE

    Calculate root-mean-squared error.

    @param[in]  estimations    State estimate vector
    @param[in]  ground_truth   Gound truth vector
    
    @return  RMSE
  ----------------------------------------------------------------------------*/
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);
  
  /*----------------------------------------------------------------------------
    @brief  NormAngle

    Normalize an angle

    @param[in]  angle    Input angle value
    
    @return  Normalized angle value
  ----------------------------------------------------------------------------*/
  double NormAngle(const double angle);
};

#endif /* UKF_H */
