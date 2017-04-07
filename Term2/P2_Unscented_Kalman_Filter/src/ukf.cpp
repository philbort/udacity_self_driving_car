#include <iostream>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF
(
  int n
)
: is_initialized_(false)
, use_laser_(true)
, use_radar_(true)
, n_x_(n)
, n_aug_(n_x_ + 2)
, lambda_(3 - n_x_)
, x_(n)
, P_(n, n)
, Xsig_pred_(n, 2 * (n + 2) + 1)
, time_us_(0)
, std_a_(30)
, std_yawdd_(30)
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
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  // Set design and covariance matrix for laser
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement
( 
  const MeasurementPackage meas_package
) 
{
  const VectorXd z = meas_package.raw_measurements_;

  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      const double rho = z(0);
      const double phi = z(1);
      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
      x_ << z(0), z(1), 0, 0, 0;

    P_ <<   1,  0,    0,  0,  0,
            0,  1,    0,  0,  0,
            0,  0, 1000,  0,  0,
            0,  0,    0, 10,  0,
            0,  0,    0,  0, 10;

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1.0e6;
  Prediction(dt);
  time_us_ = meas_package.timestamp_;

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    UpdateRadar(z);
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    UpdateLidar(z);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
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
    //extract values for better readability
    p_x = Xsig_aug(0, i);
    p_y = Xsig_aug(1, i);
    v = Xsig_aug(2, i);
    yaw = Xsig_aug(3, i);
    yawd = Xsig_aug(4, i);
    nu_a = Xsig_aug(5, i);
    nu_yawdd = Xsig_aug(6, i);

    //avoid division by zero
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

    //add noise
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v + nu_a * delta_t;

    yaw_p = yaw + yawd * delta_t + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd + nu_yawdd * delta_t;

    //write predicted sigma point into right column
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

  //iterate over sigma points
  for(int i = 0; i < 2 * n_aug_ + 1; i++) 
  { 
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while(x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
    while(x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {VectorXd} z measurement vector
 */
void UKF::UpdateLidar
(
  const VectorXd & z
) 
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

  // The current NIS
  NIS_laser_ = y.transpose() * S_inv * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {VectorXd} z measurement vector
 */
void UKF::UpdateRadar
(
  const VectorXd & z
) 
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

VectorXd UKF::CalculateRMSE
(
  const vector<VectorXd> &estimations,
  const vector<VectorXd> &ground_truth
) 
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