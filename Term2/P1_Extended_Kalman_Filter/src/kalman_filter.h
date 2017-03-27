#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter 
{
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  /**
   * Constructor
   */
  KalmanFilter(int n);

  /**
   * Destructor
   */
  ~KalmanFilter() {}

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(const Eigen::VectorXd &x_in, 
            const Eigen::MatrixXd &P_in);
  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const Eigen::MatrixXd &F,
               const Eigen::MatrixXd &Q);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z,
              const Eigen::MatrixXd &H_,
              const Eigen::MatrixXd &R_);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z,
                 const Eigen::MatrixXd &H_,
                 const Eigen::MatrixXd &R_);

};

#endif /* KALMAN_FILTER_H_ */
