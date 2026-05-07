#include <fmt/format.h>

#include <string>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化云台、相机
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  // 初始化识别器、解算器、追踪器、瞄准器
  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::SmallTarget target;
  // auto_buff::BigTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t);
    auto gs = gimbal.state();
    // recorder.record(img, q, t);

    // -------------- 打符核心逻辑 --------------

    solver.set_R_gimbal2world(q);

    auto power_runes = detector.detect(img);

    solver.solve(power_runes);

    target.get_target(power_runes, t);

    auto target_copy = target;

    auto plan = aimer.mpc_aim(target_copy, t, gs, true);

    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);
    // -------------- 调试输出 --------------

    nlohmann::json data;

    // buff原始观测数据
    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (!target.is_unsolve()) {
      auto & p = power_runes.value();

      // 显示
      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      // 当前帧target更新后buff
      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      // buff瞄准位置(预测)
      double dangle = target.ekf_x()[5] - target_copy.ekf_x()[5];
      auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target.spd;
      }
    }

    // 云台响应情况
    data["gimbal_yaw"] = gs.yaw * 57.3;
    data["gimbal_pitch"] = gs.pitch * 57.3;
    data["gimbal_yaw_vel"] = gs.yaw_vel * 57.3;
    data["gimbal_pitch_vel"] = gs.pitch_vel * 57.3;

    if (plan.control) {
      data["plan_yaw"] = plan.yaw * 57.3;
      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_yaw_vel"] = plan.yaw_vel * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel * 57.3;
      data["plan_yaw_acc"] = plan.yaw_acc * 57.3;
      data["plan_pitch_acc"] = plan.pitch_acc * 57.3;
      data["shoot"] = plan.fire ? 1 : 0;
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("result", img);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}