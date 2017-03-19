#include <iostream>
#include <cmath>
#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  int n = ground_truth.size();
  int m = ground_truth[0].size();
  VectorXd rmse(m, 0);

  for(int i = 0; i < n; i++)
  {
    VectorXd current = (estimations[i] - ground_truth[i]).array().square();
    rmse += current;
  }
  
  return (rmse/n).array().sqrt();

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd jacobian;
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    return jacobian;
}
