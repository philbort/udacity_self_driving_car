#include <cassert>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  assert(estimations.size() == ground_truth.size());

  if(estimations.empty())
    return VectorXd();

  const int n = ground_truth.size();
  VectorXd rmse(ground_truth[0].size(), 0);

  for(int i = 0; i < n; i++)
  {
    const VectorXd residual = (estimations[i] - ground_truth[i]).array().square();
    rmse += residual;
  }

  return (rmse/n).array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  MatrixXd jacobian(3, 4);
  
  const float px = x_state(0);
  const float py = x_state(1);
  const float vx = x_state(2);
  const float vy = x_state(3);


  return jacobian;
}
