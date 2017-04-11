#include <iostream>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*----------------------------------------------------------------------------
  @brief  Constructor

  Initialize all class variables and start the minimum path finder function.

  @param[in] n             Size of the state vector
  @param[in] use_laser     Flag to use laser
  @param[in] user_radar    Flag to use radar
 ----------------------------------------------------------------------------*/
UKF::UKF(const int  n,
         const bool use_laser,
         const bool user_radar)
: is_initialized_(false)
, use_laser_(use_laser)
, use_radar_(user_radar)
, n_x_(n)
, n_aug_(n_x_ + 2)
, lambda_(3 - n_x_)
, x_(n)
, P_(n, n)
, Xsig_pred_(n, 2 * (n + 2) + 1)
, time_us_(0)
, std_a_(0.3)
, std_yawdd_(1.0)
, std_laspx_(0.15)
, std_laspy_(0.15)
, std_radr_(0.3)
, std_radphi_(0.03)
, std_radrd_(0.3)
, weights_(2 * (n + 2) + 1)
, H_laser_(2, n)
, R_laser_(2, 2)
{
  // Set weights
  weights_.fill(1.0 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  // Set design and covariance matrix for laser
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
}


/*----------------------------------------------------------------------------
  @brief  ProcessMeasurement

  Process the filter update given a new measurement.

  @param[in] meas_package    The current measurement package

  @return True if successful, false otherwise.
----------------------------------------------------------------------------*/
bool UKF::ProcessMeasurement(const MeasurementPackage meas_package) 
{
  if( (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) ||
      (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) )
    return false;

  const VectorXd z = meas_package.raw_measurements_;

  if(!z(0) && !z(1))  return false;

  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      const double rho = z(0);
      const double phi = z(1);
      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }
    else
      x_ << z(0), z(1), 0.0, 0.0, 0.0;

    P_  = MatrixXd::Identity(n_x_, n_x_);

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return true;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1.0e6;

  // Reducing the prediction step helps filter convergence
  // Big prediction step violates yaw angle dynamic model assumption
  // (need to revisit this later)
  while (dt > 0.1)
  {
    Prediction(0.05);
    dt -= 0.05;
  }

  Prediction(dt);

  time_us_ = meas_package.timestamp_;

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    UpdateRadar(z);
  else
    UpdateLidar(z);

  return true;
}

/*----------------------------------------------------------------------------
  @brief  Prediction

  Unscented Kalman filter prediction (time update)

  @param[in] delta_t    Time interval between k and k+1 [s]
----------------------------------------------------------------------------*/
void UKF::Prediction(const double delta_t) 
{

  /****************************************************
   * 1. Generate sigma points
   ****************************************************/

  // Create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  
  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  
  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // Create augmented mean state
  x_aug.head(n_x_) = x_;
  
  // Create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;
  
  // Create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  
  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = 
    (sqrt(lambda_ + n_aug_) * A_aug).colwise() + x_aug;
  Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = 
    (-1 * sqrt(lambda_ + n_aug_) * A_aug).colwise() + x_aug;
  
  /****************************************************
   * 2. Predict sigma points
   ****************************************************/

  double p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd, 
         px_p, py_p, v_p, yaw_p, yawd_p;

  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Extract values for better readability
    const double p_x = Xsig_aug(0, i);
    const double p_y = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yawd = Xsig_aug(4, i);
    const double nu_a = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    // Avoid division by zero
    if(fabs(yawd) > 0.001) 
    {
        px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else 
    {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    // Add process noise
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v + nu_a * delta_t;

    yaw_p = yaw + yawd * delta_t + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd + nu_yawdd * delta_t;

    // Write predicted sigma point into the right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /****************************************************
   * 3. Predict mean and covariance
   ****************************************************/
  
  // Predict state mean
  x_ = Xsig_pred_ * weights_;
    
  // Predict state covariance matrix
  P_.fill(0.0);

  // Iterate over sigma points
  for(int i = 0; i < 2 * n_aug_ + 1; i++) 
  { 
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff(3) = NormAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

/*----------------------------------------------------------------------------
  @brief  UpdateLidar

  Linear Kalman filter Lidar measurement update

  @param[in] z    Lidar measurement
----------------------------------------------------------------------------*/
void UKF::UpdateLidar(const VectorXd & z) 
{
  // Measurement innovation
  const VectorXd y = z - H_laser_ * x_;

  // Measurement innovation covariance matrix
  const MatrixXd S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
  const MatrixXd S_inv = S.inverse();

  // Kalman gain
  const MatrixXd K = P_ * H_laser_.transpose() * S_inv;

  // Update state vector
  x_ += K * y;

  // Update state covariance matrix
  P_ = (MatrixXd::Identity(n_x_, n_x_) - K * H_laser_) * P_;

  // Calculate NIS
  NIS_laser_ = y.transpose() * S_inv * y;

}

/*----------------------------------------------------------------------------
  @brief  UpdateRadar

  Unscented Kalman filter Radar measurement update

  @param[in] z    Radar measurement
----------------------------------------------------------------------------*/
void UKF::UpdateRadar(const VectorXd & z) 
{

  /****************************************************
   * 1. Predict measurements
   ****************************************************/

  // Dimension of a radar measurement
  const int n_z = 3;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // Transform sigma points into measurement space
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    const double p_x = Xsig_pred_(0, i);
    const double p_y = Xsig_pred_(1, i);
    const double v   = Xsig_pred_(2, i);
    const double yaw = Xsig_pred_(3, i);

    const double v1  = cos(yaw) * v;
    const double v2  = sin(yaw) * v;

    // Measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2 ) / Zsig(0, i);
  }

  // Predicted measurement mean
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
      z_pred += weights_(i) * Zsig.col(i);

  // Measurement covariance matrix
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Angle normalization
    z_diff(1) = NormAngle(z_diff(1));

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_,                       0,                     0,
                         0, std_radphi_*std_radphi_,                     0,
                         0,                       0, std_radrd_*std_radrd_;
  S += R;

  /****************************************************
   * 2. State update
   ****************************************************/

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Angle normalization
    z_diff(1) = NormAngle(z_diff(1));

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff(1) = NormAngle(x_diff(1));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  const MatrixXd S_inv = S.inverse();

  // Kalman gain
  MatrixXd K = Tc * S_inv;

  // Innovation
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  z_diff(1) = NormAngle(z_diff(1));

  // Update state vector and state covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  // Calculate NIS
  NIS_radar_ = z_diff.transpose() * S_inv * z_diff;

}

/*----------------------------------------------------------------------------
  @brief  CalculateRMSE

  Calculate root-mean-squared error.

  @param[in]  estimations    State estimate vector
  @param[in]  ground_truth   Gound truth vector
  
  @return  RMSE
----------------------------------------------------------------------------*/
VectorXd UKF::CalculateRMSE(const vector<VectorXd> &estimations,
                            const vector<VectorXd> &ground_truth) 
{
  const size_t n = estimations.size();
  const size_t m = ground_truth.size();

  if(n + m == 0)
  {
    cout << "estimations and ground truth vectors must not be empty" << endl;
    return VectorXd::Zero(4);
  }
  if(n != m)
  {
    cout << "estimations and ground truth vectors ust be of equal dimensions." << endl;
    return VectorXd::Zero(4);
  }

  // Squared error initialized to zeros
  VectorXd se = VectorXd::Zero(4);

  // Accumulate the squared errors
  for (int i = 0; i < n; ++i) 
    se += static_cast<VectorXd> ((estimations[i]-ground_truth[i]).array().square());

  // Return root-mean-squared error
  return (se.array()/n).sqrt();
}

/*----------------------------------------------------------------------------
  @brief  NormAngle

  Normalize an angle

  @param[in]  angle    Input angle value
  
  @return  Normalized angle value
----------------------------------------------------------------------------*/
double UKF::NormAngle(const double angle)
{
  return atan2(sin(angle), cos(angle));
}
