#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "hikcamera/capturer.hpp"
#include "io/camera.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
    HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
    ~HikRobot() override;
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
    void ensure_connected();

    hikcamera::Camera camera_;
    std::atomic<bool> quit_{false};
    std::chrono::steady_clock::time_point next_retry_log_time_ =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
