#include "PID.h"

PID::PID(const double Kp,
         const double Ki, 
         const double Kd)
: Kp_(Kp)
, Ki_(Ki)
, Kd_(Kd)
, p_err_(0.0)
, i_err_(0.0)
, d_err_(0.0)
{

}


void PID::UpdateError(double cte)
{
    d_err_ = cte - p_err_;
    i_err_ += cte;
    p_err_ = cte;
}


double PID::TotalError()
{
    return -Kp_ * p_err_ - Kd_ * d_err_ - Ki_ * i_err_;
}

