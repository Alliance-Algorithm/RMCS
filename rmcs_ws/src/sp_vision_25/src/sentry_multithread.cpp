#include <fmt/core.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::ROS2 ros2;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);
  io::USBCamera usbcam3("video4", config_path);
  io::USBCamera usbcam4("video6", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(&usbcam1, &usbcam2, &usbcam3, &usbcam4, config_path);

  omniperception::DetectionResult switch_target;
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    recorder.record(img, q, timestamp);
    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    decider.set_priority(armors);

    auto detection_queue = perceptron.get_detection_queue();

    decider.sort(detection_queue);

    auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "switching") {
      command.control = switch_target.armors.empty() ? false : true;
      command.shoot = false;
      command.pitch = tools::limit_rad(switch_target.delta_pitch);
      command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
    }

    else if (tracker.state() == "lost") {
      command = decider.decide(detection_queue);
      command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
    }

    else {
      command = aimer.aim(targets, timestamp, cboard.bullet_speed);
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    // command.shoot = false;

    cboard.send(command);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }

  return 0;
}