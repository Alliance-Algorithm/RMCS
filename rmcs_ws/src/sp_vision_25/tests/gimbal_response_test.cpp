#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          8          | yaw轴delta角}"
  "{circle      c  |         0.2         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

double yaw_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T);  // 31是云台yaw初始角度，单位为度
}

double pitch_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18;  // 18是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::CBoard cboard(config_path);

  auto init_angle = 0;
  double slice = circle * 100;  //切片数=周期*帧率
  auto dangle = delta_angle / slice;
  double cmd_angle = init_angle;

  int axis_index = axis == "yaw" ? 0 : 1;  // 0 for yaw, 1 for pitch

  double error = 0;
  int count = 0;

  io::Command init_command{1, 0, 0, 0};
  cboard.send(init_command);
  std::this_thread::sleep_for(5s);  //等待云台归零

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  auto last_t = t;
  double dt = 0.005;  // 5ms, 模拟200fps

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    if (signal_mode == "triangle_wave") {
      if (count == slice) {
        cmd_angle = init_angle;
        command = {1, 0, 0, 0};
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count = 0;

      } else {
        cmd_angle += dangle;
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count++;
      }

      cboard.send(command);
      if (axis_index == 0) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["last_cmd_yaw"] = last_command.yaw * 57.3;
        data["gimbal_yaw"] = eulers[0] * 57.3;
      } else {
        data["cmd_pitch"] = command.pitch * 57.3;
        data["last_cmd_pitch"] = last_command.pitch * 57.3;
        data["gimbal_pitch"] = eulers[1] * 57.3;
      }
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "step") {
      if (count == 300) {
        cmd_angle += delta_angle;
        count = 0;
      }
      command = {1, 0, tools::limit_rad(cmd_angle / 57.3), 0};
      count++;

      cboard.send(command);
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "circle") {
      std::cout << "t: " << t << std::endl;
      command.yaw = yaw_cal(t) / 57.3;
      command.pitch = pitch_cal(t) / 57.3;
      command.control = 1;
      command.shoot = 0;
      t += dt;
      if (t - last_t > 2) {
        t += 2.4;
        last_t = t;
      }
      cboard.send(command);

      data["t"] = t;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
  }
  return 0;
}