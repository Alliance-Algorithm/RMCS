#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();

  last_fire_t_ = std::chrono::steady_clock::now();
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  if (target.is_unsolve()) return command;

  // 如果子弹速度小于10，将其设为24
  if (bullet_speed < 10) bullet_speed = 24;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    command.yaw = yaw;
    command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      command.control = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      command.control = false;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  if (switch_fanblade_) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

auto_aim::Plan Aimer::mpc_aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
  bool to_now)
{
  auto_aim::Plan plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};
  if (target.is_unsolve()) return plan;

  double bullet_speed;
  // 如果子弹速度小于10，将其设为24
  if (gs.bullet_speed < 10)
    bullet_speed = 24;
  else
    bullet_speed = gs.bullet_speed;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    plan.yaw = yaw;
    plan.pitch = -pitch;  //世界坐标系下的pitch向上为负
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      plan.control = true;
      first_in_aimer_ = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      plan.control = false;

      first_in_aimer_ = true;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      plan.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;

    if (plan.control) {
      if (first_in_aimer_) {
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        first_in_aimer_ = false;
      } else {
        auto dt = predict_time_;
        double last_yaw_mpc, last_pitch_mpc;
        get_send_angle(
          target, predict_time_ * -1, bullet_speed, to_now, last_yaw_mpc, last_pitch_mpc);
        plan.yaw_vel = tools::limit_rad(yaw - last_yaw_mpc) / (2 * dt);
        // plan.yaw_vel = tools::limit_min_max(plan.yaw_vel, -6.28, 6.28);
        plan.yaw_acc = (tools::limit_rad(yaw - gs.yaw) - tools::limit_rad(gs.yaw - last_yaw_mpc)) /
                       std::pow(dt, 2);
        // plan.yaw_acc = tools::limit_min_max(plan.yaw_acc, -50, 50);

        plan.pitch_vel = tools::limit_rad(-pitch + last_pitch_mpc) / (2 * dt);
        // plan.pitch_vel = tools::limit_min_max(plan.pitch_vel, -6.28, 6.28);
        plan.pitch_acc = (-pitch - gs.pitch - (gs.pitch + last_pitch_mpc)) / std::pow(dt, 2);
        // plan.pitch_acc = tools::limit_min_max(plan.pitch_acc, -100, 100);
      }
    }
  }

  if (switch_fanblade_) {
    plan.fire = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    plan.fire = true;
    last_fire_t_ = now;
  }

  return plan;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 考虑detecor所消耗的时间，此外假设aimer的用时可忽略不计
  // 如果 to_now 为 true，则根据当前时间和时间戳预测目标位置,deltatime = 现在时间减去当时照片时间，加上0.1
  target.predict(predict_time);
  // std::cout << "gap: " << detect_now_gap << std::endl;
  angle = target.ekf_x()[5];

  // 计算目标点的空间坐标
  auto aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  double h = aim_in_world[2];

  // 创建弹道对象
  tools::Trajectory trajectory0(bullet_speed, d, h);
  if (trajectory0.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 根据第一个弹道飞行时间预测目标位置
  target.predict(trajectory0.fly_time);
  angle = target.ekf_x()[5];

  // 计算新的目标点的空间坐标
  aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  d = fsqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  h = aim_in_world[2];
  tools::Trajectory trajectory1(bullet_speed, d, h);
  if (trajectory1.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 计算时间误差
  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.01) {  // 如果时间误差过大，返回未命中结果
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    return false;
  }

  // 计算偏航角和俯仰角，并返回命中结果
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  pitch = trajectory1.pitch + pitch_offset_;
  return true;
};

}  // namespace auto_buff