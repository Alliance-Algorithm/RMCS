#ifndef TOOLS__PID_HPP
#define TOOLS__PID_HPP

namespace tools
{
class PID
{
public:
  // dt: 控制周期, 单位: s
  // kp: P项系数
  // ki: I项系数
  // kd: D项系数
  // max_out: PID最大输出值
  // max_iout I项最大输出值
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false);

  float pout = 0.0f;  // P项输出, 用于调试
  float iout = 0.0f;  // I项输出, 用于调试
  float dout = 0.0f;  // D项输出, 用于调试

  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  float calc(float set, float fdb);

private:
  const float dt_;
  const float kp_, ki_, kd_;
  const float max_out_, max_iout_;
  const bool angular_;

  float last_fdb_ = 0.0f;  // 上次反馈值
};

}  // namespace tools

#endif  // TOOLS__PID_HPP