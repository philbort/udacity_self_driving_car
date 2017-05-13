#ifndef PID_H
#define PID_H

class PID
{

public:

  /*
  * Constructor
  */
  PID(const double Kp = 0.2, 
      const double Ki = 0.004,
      const double Kd = 3.0);


  /*
  * Destructor.
  */
  virtual ~PID() {}


  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);


  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Errors
  */
  double p_err_;
  double i_err_;
  double d_err_;
};

#endif /* PID_H */
