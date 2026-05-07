#include "io/gimbal/gimbal.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | | 是否开火}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);

  auto t0 = std::chrono::steady_clock::now();
  auto last_mode = gimbal.mode();
  uint16_t last_bullet_count = 0;

  auto fire = false;
  auto fire_count = 0;
  auto fire_stamp = std::chrono::steady_clock::now();
  auto first_fired = false;

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    auto t = std::chrono::steady_clock::now();
    auto state = gimbal.state();
    auto q = gimbal.q(t);
    auto ypr = tools::eulers(q, 2, 1, 0);

    auto fired = state.bullet_count > last_bullet_count;
    last_bullet_count = state.bullet_count;

    if (!first_fired && fired) {
      first_fired = true;
      tools::logger()->info("Gimbal first fired after: {:.3f}s", tools::delta_time(t, fire_stamp));
    }

    if (fire && fire_count > 20) {
      // 0.2 s
      fire = false;
      fire_count = 0;
    } else if (!fire && fire_count > 100) {
      // 1s
      fire = true;
      fire_count = 0;
      fire_stamp = t;
      first_fired = false;
    }
    fire_count++;

    gimbal.send(true, test_fire && fire, 1, 0, 0, 0, 0, 0);

    nlohmann::json data;
    data["q_yaw"] = ypr[0];
    data["q_pitch"] = ypr[1];
    data["yaw"] = state.yaw;
    data["vyaw"] = state.yaw_vel;
    data["pitch"] = state.pitch;
    data["vpitch"] = state.pitch_vel;
    data["bullet_speed"] = state.bullet_speed;
    data["bullet_count"] = state.bullet_count;
    data["fired"] = fired ? 1 : 0;
    data["fire"] = test_fire && fire ? 1 : 0;
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    std::this_thread::sleep_for(9ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}