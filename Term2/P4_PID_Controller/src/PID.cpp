#include "PID.h"
#include <math.h>

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


void PID::UpdateError(const double cte)
{
    d_err_ = cte - p_err_;
    i_err_ += cte;
    p_err_ = cte;
}


double PID::TotalError()
{
    return sigmoid(-Kp_ * p_err_ - Kd_ * d_err_ - Ki_ * i_err_);
}


double PID::sigmoid(const double value,
                    const double lower,
                    const double upper)
{
    return (upper - lower)/(1 + exp(-value)) + lower;
}
