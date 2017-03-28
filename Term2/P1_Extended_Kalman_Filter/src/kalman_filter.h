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

  // -----------------------------------------------------------------------------
  // @brief  Constructor
  //
  // @param[in] n    State vector dimension
  // -----------------------------------------------------------------------------
  KalmanFilter(int n);

  // -----------------------------------------------------------------------------
  // @brief  Destructor
  // -----------------------------------------------------------------------------
  virtual ~KalmanFilter() {}

  // -----------------------------------------------------------------------------
  // @brief  Init
  //
  // Initialize Kalman filter state and state covariance matrix
  //
  // @param[in] x_in    Initial state vector
  // @param[in] P_in    Initial state vector covariance matrix
  // -----------------------------------------------------------------------------
  void Init(const Eigen::VectorXd &x_in, 
            const Eigen::MatrixXd &P_in);

  // -----------------------------------------------------------------------------
  // @brief  Predict
  //
  // Kalman filter time update (prediction)
  //
  // @param[in] F    State transition matrix
  // @param[in] Q    State covariance matrix
  // -----------------------------------------------------------------------------
  void Predict(const Eigen::MatrixXd &F,
               const Eigen::MatrixXd &Q);

  // -----------------------------------------------------------------------------
  // @brief  Update
  //
  // Linear Kalman filter measurement update
  //
  // @param[in] z    Measurement vector
  // @param[in] H    Measurement design matrix
  // @param[in] R    Measurement noise covariance matrix
  // -----------------------------------------------------------------------------
  void Update(const Eigen::VectorXd &z,
              const Eigen::MatrixXd &H,
              const Eigen::MatrixXd &R);

  // -----------------------------------------------------------------------------
  // @brief  Update
  //
  // Extended Kalman filter measurement update
  //
  // @param[in] z    Measurement vector
  // @param[in] H    Measurement design matrix
  // @param[in] R    Measurement noise covariance matrix
  // -----------------------------------------------------------------------------
  void UpdateEKF(const Eigen::VectorXd &z,
                 const Eigen::MatrixXd &H,
                 const Eigen::MatrixXd &R);

};

#endif /* KALMAN_FILTER_H_ */
