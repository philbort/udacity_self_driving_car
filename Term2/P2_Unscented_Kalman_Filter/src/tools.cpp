#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd Tools::CalculateRMSE
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
