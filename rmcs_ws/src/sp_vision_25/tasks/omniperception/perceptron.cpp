#include "perceptron.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, io::USBCamera * usbcam3,
  io::USBCamera * usbcam4, const std::string & config_path)
: detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  // 初始化 YOLO 模型
  yolo_parallel1_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel2_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel3_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel4_ = std::make_shared<auto_aim::YOLO>(config_path, false);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  // 创建四个线程进行并行推理
  threads_.emplace_back([&] { parallel_infer(usbcam1, yolo_parallel1_); });
  threads_.emplace_back([&] { parallel_infer(usbcam2, yolo_parallel2_); });
  threads_.emplace_back([&] { parallel_infer(usbcam3, yolo_parallel3_); });
  threads_.emplace_back([&] { parallel_infer(usbcam4, yolo_parallel4_); });

  tools::logger()->info("Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程

  // 等待线程结束
  for (auto & t : threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  // 注意：这里的 pop 不阻塞（假设队列为空时会报错或忽略）
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

// 将并行推理逻辑移动到类成员函数
void Perceptron::parallel_infer(
  io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolov8_parallel)
{
  if (!cam) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }
  try {
    while (true) {
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检查是否需要退出
      }

      cam->read(usb_img, ts);
      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      auto armors = yolov8_parallel->detect(usb_img);
      if (!armors.empty()) {
        auto delta_angle = decider_.delta_angle(armors, cam->device_name);

        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = delta_angle[0] / 57.3;
        dr.delta_pitch = delta_angle[1] / 57.3;
        detection_queue_.push(dr);  // 推入线程安全队列
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
