#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools 
{
public:
  // -----------------------------------------------------------------------------
  // @brief  Constructor
  // -----------------------------------------------------------------------------
  Tools() {}

  // -----------------------------------------------------------------------------
  // @brief  Destructor
  // -----------------------------------------------------------------------------
  virtual ~Tools(){}

  // -----------------------------------------------------------------------------
  // @brief  CalculateRMSE
  //
  // A helper method to calculate the root-mean-squared error.
  //
  // @param[in] estimations    estimation vector
  // @param[in] ground_truth   ground truth vector
  // -----------------------------------------------------------------------------
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);
};

#endif /* TOOLS_H_ */
