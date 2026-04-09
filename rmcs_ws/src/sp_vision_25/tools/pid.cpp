#include "pid.hpp"

#include "math_tools.hpp"

float clip(float value, float min, float max) { return std::max(min, std::min(max, value)); }

namespace tools
{
PID::PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular)
: dt_(dt), kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout), angular_(angular)
{
}

float PID::calc(float set, float fdb)
{
  float e = angular_ ? limit_rad(set - fdb) : (set - fdb);
  float de = angular_ ? limit_rad(last_fdb_ - fdb) : (last_fdb_ - fdb);
  last_fdb_ = fdb;

  this->pout = e * kp_;
  this->iout = clip(this->iout + e * dt_ * ki_, -max_iout_, max_iout_);
  this->dout = de / dt_ * kd_;

  return clip(this->pout + this->iout + this->dout, -max_out_, max_out_);
}

}  // namespace tools
