#ifndef IO__MINDVISION_HPP
#define IO__MINDVISION_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "CameraApi.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class MindVision : public CameraBase
{
public:
  MindVision(double exposure_ms, double gamma, const std::string & vid_pid);
  ~MindVision() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_ms_, gamma_;
  CameraHandle handle_;
  int height_, width_;
  bool quit_, ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  tools::ThreadSafeQueue<CameraData> queue_;
  int vid_, pid_;

  void open();
  void try_open();
  void close();
  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__MINDVISION_HPP